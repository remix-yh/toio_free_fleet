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
        wanted_ids = {r.cube_id for r in self.robots}
        scanner = UniversalBleScanner()
        infos = await scanner.scan_with_id(
            cube_id=wanted_ids, timeout=self.scan_timeout_s
        )

        by_id = {self._extract_cube_id(info.name): info for info in infos}
        missing = [r.cube_id for r in self.robots if r.cube_id not in by_id]
        if missing:
            raise RuntimeError(
                f'cubes not found during BLE scan: {missing} '
                f'(found: {sorted(by_id.keys())})'
            )

        # Reorder the CubeInfo list to match the robots declaration order so
        # cubes[i] maps to robots[i] and LED/notification handlers line up.
        ordered_infos = [by_id[r.cube_id] for r in self.robots]
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
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        """Stop all cubes and tear down BLE connections."""
        await self.stop_all()
        if self._cubes_ctx is not None:
            await self._cubes_ctx.__aexit__(exc_type, exc, tb)

    async def stop_all(self) -> None:
        """Best-effort motor stop for every connected cube."""
        if self._cubes is None:
            return
        for i, robot in enumerate(self.robots):
            try:
                await self._cubes[i].api.motor.motor_control(left=0, right=0)
            except Exception as e:
                print(f'[{robot.name}] stop failed: {e}')

    @staticmethod
    def _extract_cube_id(local_name: str | None) -> str:
        """Pull the trailing 3-char id out of a BLE local name."""
        # toio cubes advertise as "toio Core Cube-XYZ" where XYZ is on the
        # sticker. rsplit handles future name layouts that still end with "-id".
        if not local_name:
            return ''
        return local_name.rsplit('-', 1)[-1]

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
