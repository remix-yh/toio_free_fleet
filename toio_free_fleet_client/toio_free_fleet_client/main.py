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

"""Command-line entry point for the toio_free_fleet_client process."""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path

import yaml
import zenoh

from .cube_manager import CubeManager
from .navigator import NavConfig, Navigator
from .zenoh_bridge import ZenohBridge


async def run(config: dict) -> int:
    """Run the client loop until cancelled."""
    fleet_name: str = config['fleet']['name']
    robot_names: list[str] = config['fleet']['robots']

    nav_cfg = NavConfig(
        speed_max_value=int(config['toio']['speed_max_value']),
    )

    zenoh_cfg = zenoh.Config()
    if 'zenoh' in config and 'config_file' in config['zenoh']:
        zenoh_cfg = zenoh.Config.from_file(str(config['zenoh']['config_file']))

    async with CubeManager(fleet_name=fleet_name, robot_names=robot_names) as manager:
        navigator = Navigator(nav_cfg)
        async with ZenohBridge(fleet_name, manager, navigator, zenoh_cfg):
            print(f'[{fleet_name}] connected {len(robot_names)} cube(s). Ctrl+C to stop.')
            try:
                await asyncio.Event().wait()
            except (KeyboardInterrupt, asyncio.CancelledError):
                pass
    return 0


def main() -> int:
    """Parse CLI args and dispatch to the async loop."""
    parser = argparse.ArgumentParser(prog='toio-free-fleet-client')
    parser.add_argument(
        '-c', '--config',
        type=Path,
        required=True,
        help='path to client.yaml',
    )
    args = parser.parse_args()
    with args.config.open('r') as f:
        cfg = yaml.safe_load(f)
    return asyncio.run(run(cfg))


if __name__ == '__main__':
    raise SystemExit(main())
