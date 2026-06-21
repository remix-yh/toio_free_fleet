# Copyright 2026 remix.re.yh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Manage multiple toio cubes from a single process over BLE."""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from typing import Awaitable, Callable

from toio import (
    Color,
    IndicatorParam,
    MultipleToioCoreCubes,
)
from toio.cube.api.battery import Battery, BatteryInformation
from toio.cube.api.id_information import (
    IdInformation,
    PositionId,
    PositionIdMissed,
)
from toio.scanner.ble import UniversalBleScanner


PositionListener = Callable[[str, 'CubeState'], Awaitable[None]]

# Default palette used when a robot has no led_color in config.
_DEFAULT_LED_PALETTE: list[tuple[int, int, int]] = [
    (0xFF, 0x00, 0x00),
    (0x00, 0x00, 0xFF),
    (0x00, 0xFF, 0x00),
    (0xFF, 0xFF, 0x00),
]

# Upper bounds on shutdown teardown. The toio/bleak disconnect has unbounded
# `while is_connected` retry loops; without caps a wedged BLE stack would keep
# the process alive past the launch system's SIGTERM->SIGKILL window, which
# itself leaves the cube connected -- the very state we are trying to avoid.
# Keep the sum well under launch's ~5 s SIGINT grace so we exit cleanly.
MOTOR_STOP_TIMEOUT_S = 1.5
# A connected `bluetoothctl disconnect` blocks ~2-2.6 s; we run cubes in
# parallel, so this caps the whole disconnect step, not per cube.
BLE_DISCONNECT_TIMEOUT_S = 4.0
# Timeout for the bluetoothctl calls used to purge stale connections at startup.
_BLUETOOTHCTL_TIMEOUT_S = 5.0


@dataclass
class RobotSpec:
    """Static config for a single cube in the fleet."""

    name: str
    cube_id: str
    led_color: tuple[int, int, int] | None = None


@dataclass
class CubeState:
    """Latest pose and connectivity state of a single cube."""

    name: str
    mat_x: int | None = None
    mat_y: int | None = None
    mat_angle: int | None = None
    on_mat: bool = False
    last_update_ns: int = 0
    # Battery level 0-100 reported by the cube, or None until first read.
    battery_level: int | None = None


@dataclass
class CubeManager:
    """Own the BLE connections for every cube in the fleet."""

    fleet_name: str
    robots: list[RobotSpec]
    scan_timeout_s: float = 10.0

    def __post_init__(self) -> None:
        """Initialize empty state slots for every robot name."""
        self._cubes_ctx: MultipleToioCoreCubes | None = None
        self._cubes = None
        self._states: dict[str, CubeState] = {
            r.name: CubeState(name=r.name) for r in self.robots
        }
        self._listeners: list[PositionListener] = []

    @property
    def robot_names(self) -> list[str]:
        """Return the configured robot names in declaration order."""
        return [r.name for r in self.robots]

    @property
    def states(self) -> dict[str, CubeState]:
        """Return the live state dict keyed by robot name."""
        return self._states

    def cube(self, name: str):
        """Return the underlying toio-py cube object for ``name``."""
        idx = self.robot_names.index(name)
        return self._cubes[idx]

    def add_position_listener(self, listener: PositionListener) -> None:
        """Register a coroutine called on every Position ID update."""
        self._listeners.append(listener)

    async def __aenter__(self) -> 'CubeManager':
        """Scan for the configured cube IDs, connect, and prime each cube."""
        # A cube left connected by a previous run won't advertise, so the scan
        # below would never find it. Drop any such stale link first.
        await self._purge_stale_connections()
        wanted_ids = {r.cube_id for r in self.robots}
        scanner = UniversalBleScanner()
        infos = await scanner.scan_with_id(
            cube_id=wanted_ids, timeout=self.scan_timeout_s
        )

        # Match each configured cube_id back to a CubeInfo by substring lookup
        # on the BLE local name. Stripping off the prefix (rsplit on "-") is
        # not reliable: different firmware generations advertise as
        #   "toio Core Cube-N7D", "toio-N7D", or even "toio Core Cube-N7D (...)".
        # The configured id (e.g. "N7D") appears as a substring in all of them.
        ordered_infos = []
        used = set()
        for r in self.robots:
            match = next(
                (info for info in infos
                 if id(info) not in used and r.cube_id in (info.name or '')),
                None,
            )
            if match is None:
                found = sorted(info.name or '?' for info in infos)
                raise RuntimeError(
                    f'cube not found during BLE scan: {r.cube_id} '
                    f'(found cubes: {found})'
                )
            ordered_infos.append(match)
            used.add(id(match))
        names = [r.name for r in self.robots]

        self._cubes_ctx = MultipleToioCoreCubes(cubes=ordered_infos, names=names)
        self._cubes = await self._cubes_ctx.__aenter__()
        for i, robot in enumerate(self.robots):
            r, g, b = (
                robot.led_color
                or _DEFAULT_LED_PALETTE[i % len(_DEFAULT_LED_PALETTE)]
            )
            # Some cube generations (e.g. local name "toio-XYZ" without
            # "Core Cube-") don't expose the Light/Indicator characteristic.
            # LED is only used for visual identification of cube_0 vs cube_1
            # at startup, so a missing LED isn't fatal — log and continue.
            try:
                await self._cubes[i].api.indicator.turn_on(
                    IndicatorParam(duration_ms=0, color=Color(r=r, g=g, b=b))
                )
            except Exception as e:
                print(
                    f'[{robot.name}] LED not available '
                    f'({type(e).__name__}: {e}); continuing without indicator.'
                )
            await self._cubes[i].api.id_information.register_notification_handler(
                self._make_handler(robot.name)
            )
            # Battery: seed with an initial read, then keep updated via
            # notifications (the cube pushes on change).
            try:
                info = await self._cubes[i].api.battery.read()
                if isinstance(info, BatteryInformation):
                    self._states[robot.name].battery_level = info.battery_level
                await self._cubes[i].api.battery.register_notification_handler(
                    self._make_battery_handler(robot.name)
                )
            except Exception as e:
                print(
                    f'[{robot.name}] battery not available '
                    f'({type(e).__name__}: {e}); reporting unknown.'
                )
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        """Stop motors, then force every cube to disconnect at the BlueZ level.

        We deliberately bypass toio-py's MultipleToioCoreCubes.disconnect():
        bleak >= 1.0 changed BleakClient.connect() to return None instead of
        True, and toio-py stores that in BleCube.connected, so its disconnect()
        short-circuits to a silent no-op that never drops the link (and when
        forced past that, it hangs on an unbounded `while is_connected` loop).
        That broken path is exactly what left cubes connected after exit. A
        direct `bluetoothctl disconnect` is reliable and fast on the project's
        BlueZ target.

        Each step is independently guarded and time-bounded: a failed motor
        stop must not skip the disconnect, and neither may outlive the launch
        system's shutdown grace (or we get SIGKILLed mid-teardown, leaving the
        cube connected -- the very state we're avoiding).
        """
        # Stop motors first, while the link is still up, so a cube can't keep
        # driving on its last command after we drop the connection.
        try:
            await asyncio.wait_for(self.stop_all(), timeout=MOTOR_STOP_TIMEOUT_S)
        except Exception as e:
            print(f'[{self.fleet_name}] motor stop during shutdown failed '
                  f'({type(e).__name__}: {e}); continuing to disconnect')
        try:
            await asyncio.wait_for(
                self._force_disconnect_matching(), timeout=BLE_DISCONNECT_TIMEOUT_S
            )
        except Exception as e:
            print(f'[{self.fleet_name}] BlueZ disconnect during shutdown failed '
                  f'({type(e).__name__}: {e})')

    async def _purge_stale_connections(self) -> None:
        """Drop BlueZ-level links to our cubes left over from a prior run.

        If a previous process died before it could disconnect (SIGKILL, crash,
        or a wedged teardown), BlueZ keeps the cube marked connected. A
        connected cube stops advertising, so the next scan can't find it and
        connect() fails -- the "won't reconnect until I power-cycle the cube"
        symptom. Force a disconnect on any cube BlueZ still reports as connected
        before we scan.
        """
        if await self._force_disconnect_matching():
            # Give BlueZ a moment to start seeing the cube advertise again.
            await asyncio.sleep(1.0)

    async def _force_disconnect_matching(self) -> int:
        """bluetoothctl-disconnect every connected device that is one of ours.

        Matches BlueZ's connected list against the configured cube_ids by name
        and force-drops each link. Reliable regardless of the bleak/toio
        disconnect bug; scoped to the project's Ubuntu 24.04 + BlueZ setup.
        Returns the number of cubes disconnected.
        """
        wanted_ids = [r.cube_id for r in self.robots]
        targets = [
            (mac, name)
            for mac, name in await self._bluetoothctl_connected_devices()
            if any(cid in name for cid in wanted_ids)
        ]
        for mac, name in targets:
            print(f'[{self.fleet_name}] force-disconnecting {name} ({mac})')
        # Disconnect in parallel: a connected disconnect blocks ~2 s each, so
        # serial calls would blow the shutdown budget for more than one cube.
        # _bluetoothctl_disconnect swallows its own errors, so gather won't raise.
        await asyncio.gather(
            *(self._bluetoothctl_disconnect(mac) for mac, _ in targets)
        )
        return len(targets)

    @staticmethod
    async def _run_bluetoothctl(*args: str) -> str:
        """Run ``bluetoothctl <args>`` one-shot and return its stdout."""
        proc = await asyncio.create_subprocess_exec(
            'bluetoothctl', *args,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        try:
            stdout, _ = await asyncio.wait_for(
                proc.communicate(), timeout=_BLUETOOTHCTL_TIMEOUT_S
            )
        except asyncio.TimeoutError:
            proc.kill()
            await proc.wait()
            raise
        return stdout.decode(errors='replace')

    async def _bluetoothctl_connected_devices(self) -> list[tuple[str, str]]:
        """Return ``(mac, name)`` for every device BlueZ reports as connected."""
        try:
            out = await self._run_bluetoothctl('devices', 'Connected')
        except Exception as e:
            print(f'[{self.fleet_name}] could not list BlueZ connections '
                  f'({type(e).__name__}: {e}); skipping stale-connection purge')
            return []
        devices: list[tuple[str, str]] = []
        for line in out.splitlines():
            # "Device AA:BB:CC:DD:EE:FF toio Core Cube-N7D"
            parts = line.strip().split(maxsplit=2)
            if len(parts) >= 3 and parts[0] == 'Device':
                devices.append((parts[1], parts[2]))
        return devices

    async def _bluetoothctl_disconnect(self, mac: str) -> None:
        """Force BlueZ to drop the link to ``mac`` (best-effort)."""
        try:
            await self._run_bluetoothctl('disconnect', mac)
        except Exception as e:
            print(f'[{self.fleet_name}] disconnect {mac} failed '
                  f'({type(e).__name__}: {e})')

    async def stop_all(self) -> None:
        """Best-effort motor stop for every connected cube."""
        if self._cubes is None:
            return
        for i, robot in enumerate(self.robots):
            try:
                await self._cubes[i].api.motor.motor_control(left=0, right=0)
            except Exception as e:
                print(f'[{robot.name}] stop failed: {e}')

    def _make_handler(self, name: str):
        """Build a Position ID notification handler bound to ``name``."""
        loop = asyncio.get_running_loop()

        def handler(payload: bytearray) -> None:
            info = IdInformation.is_my_data(payload)
            state = self._states[name]
            if isinstance(info, PositionId):
                state.mat_x = info.center.point.x
                state.mat_y = info.center.point.y
                state.mat_angle = info.center.angle
                state.on_mat = True
            elif isinstance(info, PositionIdMissed):
                state.on_mat = False
            state.last_update_ns = time.time_ns()
            for listener in self._listeners:
                loop.create_task(listener(name, state))
        return handler

    def _make_battery_handler(self, name: str):
        """Build a battery notification handler bound to ``name``."""
        def handler(payload: bytearray) -> None:
            info = Battery.is_my_data(payload)
            if isinstance(info, BatteryInformation):
                self._states[name].battery_level = info.battery_level
        return handler
