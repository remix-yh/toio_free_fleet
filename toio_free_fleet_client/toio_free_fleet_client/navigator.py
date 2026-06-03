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
    # long enough for the longest single lane segment to complete. Upstream's
    # MoveRobot "command handle seems to be unresponsive" timer is 10s but
    # gets reset by every TF / feedback update (10Hz), so it effectively never
    # fires and isn't the limiting factor here.
    per_waypoint_timeout_s: int = 10
    # Extra slack on top of the cube-side timeout before we give up locally.
    response_grace_s: float = 1.0
    # toio reports ERROR_ID_MISSED whenever it briefly can't read a PositionId
    # tile during motion — typically a transient glitch from a worn/dirty mat,
    # not the cube actually leaving the play surface. Retry the same target
    # this many times before treating ID_MISSED as a real OFF_MAT failure.
    id_missed_max_retries: int = 3
    # Short wait after an ID_MISSED before retrying, to let the next PositionId
    # notification arrive (and state.on_mat refresh).
    id_missed_retry_delay_s: float = 0.2


class Navigator:
    """Follow RMF waypoint paths by driving the toio target motor command."""

    def __init__(self, config: NavConfig | None = None) -> None:
        """Build a navigator with the given motion configuration."""
        self.config = config or NavConfig()

    async def follow_path(self, cube, path: list[RmfPose], state=None) -> NavResult:
        """Drive ``cube`` through every waypoint in ``path``.

        ``state`` is the optional live CubeState for this cube; it lets _goto
        check on_mat when deciding whether an ID_MISSED is transient.
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

        loop = asyncio.get_running_loop()

        def make_handler(arrival: asyncio.Future[MotorResponseCode]):
            def on_motor_response(payload: bytearray) -> None:
                # The motor characteristic emits several response types; only
                # the target-control response carries the arrival code.
                info = Motor.is_my_data(payload)
                if isinstance(info, ResponseMotorControlTarget) and not arrival.done():
                    arrival.set_result(info.response_code)
            return on_motor_response

        # Retry loop: ID_MISSED is treated as transient and retried while the
        # cube still believes it's on the mat. Any other terminal code (or
        # exhausting retries) short-circuits and bubbles up.
        code: MotorResponseCode | None = None
        retries_left = self.config.id_missed_max_retries
        while True:
            arrival: asyncio.Future[MotorResponseCode] = loop.create_future()
            handler = make_handler(arrival)
            await cube.api.motor.register_notification_handler(handler)
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
                        # arrival note above. WithoutRotation keeps the cube
                        # from spinning in place at each waypoint.
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
                # RMF is yielding this robot. Brake the cube before bubbling
                # up so the negotiation result is physically respected.
                try:
                    await asyncio.shield(
                        cube.api.motor.motor_control(left=0, right=0)
                    )
                except Exception:
                    pass
                return NavResult.PREEMPTED
            finally:
                await cube.api.motor.unregister_notification_handler(handler)

            if code != MotorResponseCode.ERROR_ID_MISSED:
                break

            # ID_MISSED: usually a transient PositionId hiccup, not a real
            # off-mat. Retry while the cube still reports on_mat, up to
            # `id_missed_max_retries` times.
            await asyncio.sleep(self.config.id_missed_retry_delay_s)
            on_mat_now = state is not None and state.on_mat
            if not on_mat_now or retries_left <= 0:
                break
            retries_left -= 1

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
