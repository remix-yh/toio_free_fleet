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

from glob import glob

from setuptools import find_packages, setup

package_name = 'toio_free_fleet_rmf'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config/fleet', glob('config/fleet/*.yaml')),
        (f'share/{package_name}/config/zenoh', glob('config/zenoh/*.json5')),
        (f'share/{package_name}/maps/toio',
         glob('maps/toio/*.yaml') + glob('maps/toio/*.png')),
        (f'share/{package_name}/launch', glob('launch/*.launch.xml')),
        (f'share/{package_name}/launch/include', glob('launch/include/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='remix.re.yh',
    maintainer_email='remix.re.yh@gmail.com',
    description='RMF integration assets (maps, fleet config, launch) for toio Core Cube fleets.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
