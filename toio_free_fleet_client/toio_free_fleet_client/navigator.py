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

"""Drive a cube along a sequence of RMF waypoints."""

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
from toio.cube.api.motor import (
    Motor,
    MotorResponseCode,
    ResponseMotorControlTarget,
)

from .transform import inside_mat, rmf_to_mat_xy, RmfPose


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
    # toio motor_control_target's own timeout is in seconds (max 255). Must be
    # long enough for the longest single lane segment to complete; the upstream
    # adapter's "unresponsive" replan is driven by the conservative
    # vehicle_traits in toio_config.yaml, not a fixed timer, so we mainly need
    # this to not cut off a legitimately in-progress move.
    per_waypoint_timeout_s: int = 6
    # Extra slack on top of the cube-side timeout before we give up locally.
    response_grace_s: float = 1.0
    # If the cube is already within this many mat units of the target, treat
    # the goal as reached without issuing a motor command (avoids spinning in
    # place when the target is essentially the cube's current position).
    arrival_tolerance_units: int = 12


class Navigator:
    """Follow RMF waypoint paths by driving the toio target motor command."""

    def __init__(self, config: NavConfig | None = None) -> None:
        """Build a navigator with the given motion configuration."""
        self.config = config or NavConfig()

    async def follow_path(self, cube, path: list[RmfPose], state=None) -> NavResult:
        """Drive ``cube`` through every waypoint in ``path``.

        ``state`` is the optional live CubeState for this cube; when provided
        it lets _goto skip targets the cube has already reached.
        """
        for waypoint in path:
            result = await self._goto(cube, waypoint, state)
            if result is not NavResult.COMPLETED:
                return result
        return NavResult.COMPLETED

    async def _goto(self, cube, target: RmfPose, state=None) -> NavResult:
        """Send a target command and wait for the cube to report arrival."""
        # `target` is in the cube's RMF-meter frame (mat top-left = 0,0).
        # Convert back to mat units for motor_control_target.
        mx, my = rmf_to_mat_xy(target.x, target.y)
        if not inside_mat(mx, my):
            return NavResult.INVALID

        # Already at this position? Skip the command. We intentionally ignore
        # the goal's target heading: the adapter rapidly re-sends same-position
        # goals with varying yaw to orient the cube, and honoring those with
        # motor_control_target makes the cube spin/drift in place (differential
        # rotation isn't a clean pivot), creating a feedback-instability loop.
        # A patrol only needs the cube to reach positions, so position-only
        # arrival keeps it stable.
        if state is not None and state.mat_x is not None and state.mat_y is not None:
            tol = self.config.arrival_tolerance_units
            if abs(state.mat_x - mx) <= tol and abs(state.mat_y - my) <= tol:
                return NavResult.COMPLETED

        loop = asyncio.get_running_loop()
        arrival: asyncio.Future[MotorResponseCode] = loop.create_future()

        def on_motor_response(payload: bytearray) -> None:
            # The motor characteristic emits several response types; only the
            # target-control response carries the arrival code we care about.
            info = Motor.is_my_data(payload)
            if isinstance(info, ResponseMotorControlTarget) and not arrival.done():
                arrival.set_result(info.response_code)

        await cube.api.motor.register_notification_handler(on_motor_response)
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
                    # Don't chase a final heading — see the position-only
                    # arrival note above. WithoutRotation keeps the cube from
                    # spinning in place at each waypoint.
                    rotation_option=RotationOption.WithoutRotation,
                ),
            )
            try:
                code = await asyncio.wait_for(
                    arrival,
                    timeout=self.config.per_waypoint_timeout_s + self.config.response_grace_s,
                )
            except asyncio.TimeoutError:
                return NavResult.TIMEOUT
        except asyncio.CancelledError:
            return NavResult.PREEMPTED
        finally:
            await cube.api.motor.unregister_notification_handler(on_motor_response)

        return self._map_response_code(code)

    @staticmethod
    def _map_response_code(code: MotorResponseCode) -> NavResult:
        """Translate a toio motor response code to a NavResult."""
        if code == MotorResponseCode.SUCCESS:
            return NavResult.COMPLETED
        # SUCCESS_WITH_OVERWRITE means a newer motor_control_target replaced
        # this one before it finished — i.e. the target was NOT reached. We
        # must not report it as COMPLETED or RMF will mark waypoints (and the
        # whole task) done while the cube is still mid-transit.
        if code == MotorResponseCode.SUCCESS_WITH_OVERWRITE:
            return NavResult.PREEMPTED
        if code == MotorResponseCode.ERROR_TIMEOUT:
            return NavResult.TIMEOUT
        if code == MotorResponseCode.ERROR_ID_MISSED:
            return NavResult.OFF_MAT
        return NavResult.INVALID
