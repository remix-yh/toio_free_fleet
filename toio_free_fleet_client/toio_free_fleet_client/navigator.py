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

"""
Drive a cube along a sequence of RMF waypoints.

The first cut issues ``motor_control_target`` sequentially per waypoint.
A future revision can collapse paths into a single ``motor_control_multiple_targets``
call to save BLE bandwidth.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from enum import Enum

from toio import (
    CubeLocation,
    MovementType,
    Point,
    RotationOption,
    Speed,
    SpeedChangeType,
    TargetPosition,
)

from .transform import inside_safe_rect_mat, rmf_to_mat_xy, RmfPose


class NavResult(Enum):
    """Outcome of following a single waypoint or a whole path."""

    COMPLETED = 'completed'
    TIMEOUT = 'timeout'
    OFF_MAT = 'off_mat'
    PREEMPTED = 'preempted'
    INVALID = 'invalid'


@dataclass
class NavConfig:
    """Tuning knobs for the cube motion profile."""

    speed_max_value: int = 20
    speed_change_type: SpeedChangeType = SpeedChangeType.AccelerationAndDeceleration
    per_waypoint_timeout_s: int = 6


class Navigator:
    """Follow RMF waypoint paths by driving the toio target motor command."""

    def __init__(self, config: NavConfig | None = None) -> None:
        """Build a navigator with the given motion configuration."""
        self.config = config or NavConfig()

    async def follow_path(self, cube, path: list[RmfPose]) -> NavResult:
        """Drive ``cube`` through every waypoint in ``path``."""
        for waypoint in path:
            result = await self._goto(cube, waypoint)
            if result is not NavResult.COMPLETED:
                return result
        return NavResult.COMPLETED

    async def _goto(self, cube, target: RmfPose) -> NavResult:
        """Send a single ``motor_control_target`` command."""
        mx, my = rmf_to_mat_xy(target.x, target.y)
        if not inside_safe_rect_mat(mx, my):
            return NavResult.INVALID
        try:
            await cube.api.motor.motor_control_target(
                timeout=self.config.per_waypoint_timeout_s,
                movement_type=MovementType.Linear,
                speed=Speed(
                    max=self.config.speed_max_value,
                    speed_change_type=self.config.speed_change_type,
                ),
                target=TargetPosition(
                    cube_location=CubeLocation(
                        point=Point(x=mx, y=my),
                        angle=0,
                    ),
                    rotation_option=RotationOption.WithoutRotation,
                ),
            )
        except asyncio.CancelledError:
            return NavResult.PREEMPTED
        return NavResult.COMPLETED
