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
Skeleton bridge between this client and the free_fleet Zenoh transport.

The eventual implementation must follow free_fleet upstream's message
schemas (RobotState, NavigationRequest, ...). The current encoding is a
JSON placeholder so the dataflow can be wired end to end before the
CDR / ROS 2 IDL helpers are imported.
"""

from __future__ import annotations

import asyncio
from dataclasses import asdict, dataclass
import json

import zenoh

from .cube_manager import CubeManager, CubeState
from .navigator import Navigator
from .transform import mat_angle_to_rmf_yaw, mat_to_rmf_xy, RmfPose


PUB_STATE_PERIOD_S = 0.5


@dataclass
class RobotStateMsg:
    """Placeholder envelope for the per-robot state message."""

    fleet: str
    robot: str
    x: float
    y: float
    yaw: float
    on_map: bool


class ZenohBridge:
    """Publish robot state and accept navigation requests over Zenoh."""

    def __init__(
        self,
        fleet_name: str,
        manager: CubeManager,
        navigator: Navigator,
        zenoh_config: zenoh.Config | None = None,
    ) -> None:
        """Wire the bridge to a cube manager and a navigator."""
        self.fleet_name = fleet_name
        self.manager = manager
        self.navigator = navigator
        self._zenoh_config = zenoh_config or zenoh.Config()
        self._session: zenoh.Session | None = None
        self._subs: list[zenoh.Subscriber] = []
        self._tasks: list[asyncio.Task] = []

    async def __aenter__(self) -> 'ZenohBridge':
        """Open the Zenoh session and start the periodic state publisher."""
        self._session = zenoh.open(self._zenoh_config)
        for name in self.manager.robot_names:
            key = f'free_fleet/{self.fleet_name}/{name}/navigation_request'
            self._subs.append(
                self._session.declare_subscriber(key, self._on_nav_request(name))
            )
        self._tasks.append(asyncio.create_task(self._publish_state_loop()))
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        """Cancel background tasks and close the Zenoh session."""
        for t in self._tasks:
            t.cancel()
        for s in self._subs:
            s.undeclare()
        if self._session is not None:
            self._session.close()

    async def _publish_state_loop(self) -> None:
        """Periodically publish a state message for every robot."""
        assert self._session is not None
        while True:
            for name, state in self.manager.states.items():
                msg = self._state_to_msg(name, state)
                key = f'free_fleet/{self.fleet_name}/{name}/state'
                self._session.put(key, json.dumps(asdict(msg)).encode('utf-8'))
            await asyncio.sleep(PUB_STATE_PERIOD_S)

    def _state_to_msg(self, name: str, state: CubeState) -> RobotStateMsg:
        """Snapshot a CubeState into a serializable RobotStateMsg."""
        if state.mat_x is None or state.mat_y is None:
            return RobotStateMsg(self.fleet_name, name, 0.0, 0.0, 0.0, False)
        x, y = mat_to_rmf_xy(state.mat_x, state.mat_y)
        yaw = mat_angle_to_rmf_yaw(state.mat_angle or 0)
        return RobotStateMsg(self.fleet_name, name, x, y, yaw, state.on_mat)

    def _on_nav_request(self, name: str):
        """Build a Zenoh subscriber callback for a single robot."""
        loop = asyncio.get_event_loop()

        def callback(sample: zenoh.Sample) -> None:
            payload = json.loads(bytes(sample.payload).decode('utf-8'))
            path = [RmfPose(**wp) for wp in payload.get('path', [])]
            asyncio.run_coroutine_threadsafe(
                self._handle_nav_request(name, path), loop
            )
        return callback

    async def _handle_nav_request(self, name: str, path: list[RmfPose]) -> None:
        """Run a path on a cube and publish the result back over Zenoh."""
        cube = self.manager.cube(name)
        result = await self.navigator.follow_path(cube, path)
        key = f'free_fleet/{self.fleet_name}/{name}/navigation_result'
        assert self._session is not None
        self._session.put(key, json.dumps({'result': result.value}).encode('utf-8'))
