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
List the BLE local names of every nearby toio cube.

Run with one cube on at a time to map the physical cube to its 3-char id.

    ros2 run toio_free_fleet_client scan_cubes
"""

from __future__ import annotations

import argparse
import asyncio

from toio.scanner.ble import UniversalBleScanner


async def _scan(max_cubes: int, timeout: float) -> int:
    """Print every cube found within ``timeout`` seconds."""
    scanner = UniversalBleScanner()
    cubes = await scanner.scan(num=max_cubes, timeout=timeout)
    if not cubes:
        print('no cubes found. Make sure the cube is on and within range.')
        return 1
    print(f'found {len(cubes)} cube(s):')
    for info in cubes:
        name = info.name or '?'
        cube_id = name.rsplit('-', 1)[-1] if '-' in name else '?'
        print(f'  cube_id={cube_id:<5} local_name={name}')
    return 0


def main() -> int:
    """Parse CLI args and run a one-shot BLE scan."""
    parser = argparse.ArgumentParser(prog='scan_cubes')
    parser.add_argument(
        '-n', '--max-cubes', type=int, default=10,
        help='max number of cubes to find before returning (default: 10)',
    )
    parser.add_argument(
        '-t', '--timeout', type=float, default=5.0,
        help='scan timeout in seconds (default: 5.0)',
    )
    args = parser.parse_args()
    return asyncio.run(_scan(args.max_cubes, args.timeout))


if __name__ == '__main__':
    raise SystemExit(main())
