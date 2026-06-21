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
Entry point for the toio_free_fleet_client ROS 2 node.

Layout:
* The main thread runs an asyncio loop that owns every BLE connection.
* A worker thread runs the rclpy multi-threaded executor so action callbacks
  can hand work to the asyncio loop without blocking ROS callbacks.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.utilities import remove_ros_args
from toio import SpeedChangeType
import yaml

from .cube_manager import CubeManager, RobotSpec
from .navigator import NavConfig, Navigator
from .ros_adapter import ToioFleetNode


def _parse_robots(robots_cfg: list[dict]) -> list[RobotSpec]:
    """Convert the ``fleet.robots`` block from YAML into RobotSpec objects."""
    specs: list[RobotSpec] = []
    for entry in robots_cfg:
        led = entry.get('led_color')
        specs.append(RobotSpec(
            name=entry['name'],
            cube_id=entry['cube_id'],
            led_color=tuple(led) if led is not None else None,
        ))
    return specs


def _parse_nav_config(toio_cfg: dict) -> NavConfig:
    """Build a NavConfig from the ``toio:`` block in YAML."""
    speed_max = int(toio_cfg.get('speed_max_value', NavConfig.speed_max_value))
    speed_change_raw = toio_cfg.get('speed_change_type')
    speed_change = (
        SpeedChangeType(int(speed_change_raw))
        if speed_change_raw is not None
        else NavConfig.speed_change_type
    )
    return NavConfig(speed_max_value=speed_max, speed_change_type=speed_change)


async def _ble_main(
    fleet_name: str,
    robots: list[RobotSpec],
    nav_cfg: NavConfig,
    map_frame: str,
    robot_frame: str,
    ready: asyncio.Event,
    shutdown: asyncio.Event,
    out: dict,
) -> None:
    """Hold every BLE connection open until ``shutdown`` is set."""
    async with CubeManager(fleet_name=fleet_name, robots=robots) as manager:
        out['manager'] = manager
        out['navigator'] = Navigator(nav_cfg)
        out['loop'] = asyncio.get_running_loop()
        out['map_frame'] = map_frame
        out['robot_frame'] = robot_frame
        ready.set()
        await shutdown.wait()


def main() -> int:
    """Parse CLI args and start both the asyncio and rclpy event loops."""
    parser = argparse.ArgumentParser(prog='toio_free_fleet_client')
    parser.add_argument(
        '-c', '--config',
        type=Path,
        required=True,
        help='path to client.yaml',
    )
    # `ros2 launch` and `ros2 run` may append --ros-args / -r remap arguments
    # before our parser sees them; strip those out so argparse doesn't fail.
    args = parser.parse_args(remove_ros_args(sys.argv[1:]))
    with args.config.open('r') as f:
        cfg = yaml.safe_load(f)

    fleet_name: str = cfg['fleet']['name']
    robots = _parse_robots(cfg['fleet']['robots'])
    nav_cfg = _parse_nav_config(cfg.get('toio', {}))
    frames = cfg.get('frames', {})
    map_frame = frames.get('map', 'map')
    robot_frame = frames.get('robot', 'base_footprint')

    rclpy.init()

    # BLE loop runs on the main thread; we own its event loop explicitly so
    # rclpy callbacks (on a worker thread) can post coroutines back to it.
    asyncio_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(asyncio_loop)

    ready = asyncio.Event()
    shutdown = asyncio.Event()
    shared: dict = {}

    ble_task = asyncio_loop.create_task(
        _ble_main(fleet_name, robots, nav_cfg, map_frame, robot_frame,
                  ready, shutdown, shared)
    )

    # Run the asyncio loop in a background thread so we can spin rclpy here.
    def _run_loop() -> None:
        try:
            asyncio_loop.run_until_complete(ble_task)
        except asyncio.CancelledError:
            pass

    # daemon=True so a wedged BLE disconnect (the toio/bleak stack has unbounded
    # is_connected wait loops) can never block interpreter exit. We still join
    # with a timeout below to give a clean disconnect its chance; only a genuine
    # hang falls through, and the next run's stale-connection purge recovers.
    loop_thread = threading.Thread(target=_run_loop, name='ble-loop', daemon=True)
    loop_thread.start()

    # Wait for the BLE side to finish connecting before we expose ROS topics.
    fut = asyncio.run_coroutine_threadsafe(ready.wait(), asyncio_loop)
    fut.result()

    node = ToioFleetNode(
        fleet_name=fleet_name,
        manager=shared['manager'],
        navigator=shared['navigator'],
        asyncio_loop=shared['loop'],
        map_frame=shared['map_frame'],
        robot_frame=shared['robot_frame'],
    )
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT/SIGTERM from `ros2 launch` teardown. rclpy installs handlers
        # for both signals by default; either shuts the context and unblocks
        # spin here (KeyboardInterrupt for SIGINT, ExternalShutdownException
        # once the context is already down).
        pass
    finally:
        # Tear down BLE FIRST and wait for the disconnect to finish: a clean
        # disconnect is what lets the cubes be found and reconnected next run.
        # Do it before ROS teardown so an error destroying the node can't skip
        # it. The join timeout exceeds CubeManager's own teardown budget
        # (motor stop + disconnect) with margin.
        asyncio_loop.call_soon_threadsafe(shutdown.set)
        loop_thread.join(timeout=6.0)
        if loop_thread.is_alive():
            node.get_logger().warn(
                'BLE teardown did not finish in time; a connection may be '
                'left stale (the next run purges it before reconnecting)'
            )
        try:
            executor.remove_node(node)
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
