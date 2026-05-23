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
ROS 2 facade that exposes each toio cube the way Nav2RobotAdapter expects.

For every robot ``<name>`` configured in client.yaml the facade exposes:

* ``/<name>/tf``                              (tf2_msgs/TFMessage)
* ``/<name>/battery_state``                   (sensor_msgs/BatteryState)
* ``/<name>/navigate_to_pose``                (nav2_msgs/action/NavigateToPose)

``zenoh-bridge-ros2dds`` is expected to be running alongside this node and
will translate the above into the zenoh keys the upstream free_fleet
adapter subscribes/queries from the RMF host.

The BLE side runs on an asyncio loop; rclpy callbacks are dispatched there
via ``asyncio.run_coroutine_threadsafe``.
"""

from __future__ import annotations

import asyncio
import math
import threading

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from tf2_msgs.msg import TFMessage

from .cube_manager import CubeManager
from .navigator import NavResult, Navigator
from .transform import mat_angle_to_rmf_yaw, mat_to_rmf_xy, RmfPose


BATTERY_STATE_PERIOD_S = 1.0
TF_PUBLISH_PERIOD_S = 0.1


class _RobotFacade:
    """ROS 2 entry points for a single cube."""

    def __init__(
        self,
        node: Node,
        robot_name: str,
        manager: CubeManager,
        navigator: Navigator,
        asyncio_loop: asyncio.AbstractEventLoop,
        map_frame: str = 'map',
        robot_frame: str = 'base_footprint',
    ) -> None:
        """Wire publishers and the action server for one robot."""
        self.node = node
        self.robot_name = robot_name
        self.manager = manager
        self.navigator = navigator
        self.asyncio_loop = asyncio_loop
        self.map_frame = map_frame
        self.robot_frame = robot_frame

        # Active goal bookkeeping. We only allow one nav goal per cube at a time.
        self._active_goal_handle: ServerGoalHandle | None = None
        self._active_goal_lock = threading.Lock()

        self.tf_pub = node.create_publisher(
            TFMessage, f'/{robot_name}/tf', 10
        )
        self.battery_pub = node.create_publisher(
            BatteryState, f'/{robot_name}/battery_state', 10
        )

        self.tf_timer = node.create_timer(TF_PUBLISH_PERIOD_S, self._publish_tf)
        self.battery_timer = node.create_timer(
            BATTERY_STATE_PERIOD_S, self._publish_battery_state
        )

        self.nav_server = ActionServer(
            node,
            NavigateToPose,
            f'/{robot_name}/navigate_to_pose',
            execute_callback=self._execute_navigate,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
        )

    def _publish_tf(self) -> None:
        """Publish the latest cube pose in the cube's RMF-meter frame.

        The cube's frame anchors mat top-left at (0, 0); this is what
        ``reference_coordinates.robot`` describes. Upstream's adapter then
        applies a small nudged transform to project into the traffic_editor
        image frame (which is offset by the image legend margins).
        """
        state = self.manager.states[self.robot_name]
        if state.mat_x is None or state.mat_y is None or not state.on_mat:
            return

        rmf_x, rmf_y = mat_to_rmf_xy(state.mat_x, state.mat_y)
        yaw = mat_angle_to_rmf_yaw(state.mat_angle or 0)

        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.robot_frame
        t.transform.translation.x = rmf_x
        t.transform.translation.y = rmf_y
        t.transform.translation.z = 0.0
        # yaw-only rotation around Z.
        half = yaw * 0.5
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)

        self.tf_pub.publish(TFMessage(transforms=[t]))

    def _publish_battery_state(self) -> None:
        """Publish a synthetic battery state until cube battery is wired in."""
        msg = BatteryState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_frame
        msg.voltage = float('nan')
        msg.percentage = 1.0
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery_pub.publish(msg)

    def _on_goal_request(self, goal_request) -> GoalResponse:
        """Always accept goals.

        Nav2RobotAdapter preempts in-flight goals by sending a new one when
        it decides to replan, and treats rejection as an issue-ticket-worthy
        failure. We accept every goal and the previous _execute_navigate
        will simply observe the next motor_control_target overwriting its
        target and return SUCCESS_WITH_OVERWRITE.
        """
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle) -> CancelResponse:
        """Always accept cancellations; execute_callback observes it."""
        return CancelResponse.ACCEPT

    def _execute_navigate(self, goal_handle: ServerGoalHandle):
        """Run a NavigateToPose goal by delegating to the BLE Navigator.

        Implemented as a sync callback so we can block on the BLE work
        via concurrent.futures.Future.result(). The MultiThreadedExecutor
        ensures other ROS callbacks keep running while this thread waits.
        Async + asyncio.wrap_future doesn't work here because the ROS
        action executor threads don't have an asyncio loop installed.
        """
        with self._active_goal_lock:
            self._active_goal_handle = goal_handle

        result = NavigateToPose.Result()
        try:
            target = self._goal_to_rmf_pose(goal_handle.request)
            cube = self.manager.cube(self.robot_name)
            # Schedule BLE work on the asyncio loop and block until it finishes.
            future = asyncio.run_coroutine_threadsafe(
                self.navigator.follow_path(cube, [target]),
                self.asyncio_loop,
            )
            ble_result: NavResult = future.result()
        except Exception as e:
            self.node.get_logger().error(
                f'[{self.robot_name}] navigate failed: {type(e).__name__}: {e}'
            )
            goal_handle.abort()
            return result

        if ble_result is NavResult.COMPLETED:
            goal_handle.succeed()
        elif ble_result is NavResult.PREEMPTED:
            goal_handle.canceled()
        else:
            self.node.get_logger().warn(
                f'[{self.robot_name}] navigate ended as {ble_result.value}'
            )
            goal_handle.abort()
        return result

    @staticmethod
    def _goal_to_rmf_pose(goal) -> RmfPose:
        """Convert a NavigateToPose goal message to our RmfPose."""
        p = goal.pose.pose.position
        q = goal.pose.pose.orientation
        # yaw from quaternion (z-axis rotation only).
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return RmfPose(x=p.x, y=p.y, yaw_rad=yaw)


class ToioFleetNode(Node):
    """ROS 2 node owning one _RobotFacade per configured cube."""

    def __init__(
        self,
        fleet_name: str,
        manager: CubeManager,
        navigator: Navigator,
        asyncio_loop: asyncio.AbstractEventLoop,
        map_frame: str = 'map',
        robot_frame: str = 'base_footprint',
    ) -> None:
        """Build the node and instantiate per-robot facades."""
        super().__init__(f'{fleet_name}_free_fleet_client')
        self.fleet_name = fleet_name
        self.manager = manager
        self.facades = [
            _RobotFacade(
                self,
                robot_name=name,
                manager=manager,
                navigator=navigator,
                asyncio_loop=asyncio_loop,
                map_frame=map_frame,
                robot_frame=robot_frame,
            )
            for name in manager.robot_names
        ]
        self.get_logger().info(
            f'[{fleet_name}] facade up for {len(self.facades)} robot(s)'
        )
