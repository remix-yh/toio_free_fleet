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
from dataclasses import dataclass, field
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


PositionListener = Callable[[str, 'CubeState'], Awaitable[None]]


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
    robot_names: list[str]
    led_colors: list[tuple[int, int, int]] = field(default_factory=lambda: [
        (0xFF, 0x00, 0x00),
        (0x00, 0x00, 0xFF),
        (0x00, 0xFF, 0x00),
        (0xFF, 0xFF, 0x00),
    ])

    def __post_init__(self) -> None:
        """Initialize empty state slots for every robot name."""
        self._cubes_ctx: MultipleToioCoreCubes | None = None
        self._cubes = None
        self._states: dict[str, CubeState] = {
            name: CubeState(name=name) for name in self.robot_names
        }
        self._listeners: list[PositionListener] = []

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
        """Scan, connect, and prime every cube in the fleet."""
        self._cubes_ctx = MultipleToioCoreCubes(cubes=len(self.robot_names))
        self._cubes = await self._cubes_ctx.__aenter__()
        for i, name in enumerate(self.robot_names):
            r, g, b = self.led_colors[i % len(self.led_colors)]
            await self._cubes[i].api.indicator.turn_on(
                IndicatorParam(duration_ms=0, color=Color(r=r, g=g, b=b))
            )
            await self._cubes[i].api.id_information.register_notification_handler(
                self._make_handler(name)
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
        for i, name in enumerate(self.robot_names):
            try:
                await self._cubes[i].api.motor.motor_control(left=0, right=0)
            except Exception as e:
                print(f'[{name}] stop failed: {e}')

    def _make_handler(self, name: str):
        """Build a Position ID notification handler bound to ``name``."""
        loop = asyncio.get_event_loop()

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
            state.last_update_ns = loop.time_ns() if hasattr(loop, 'time_ns') else 0
            for listener in self._listeners:
                asyncio.create_task(listener(name, state))
        return handler
