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

"""Unit tests for the mat <-> RMF coordinate transform."""

import math

from toio_free_fleet_client.transform import (
    inside_safe_rect_mat,
    MAT_ORIGIN_UNITS,
    MAT_SAFE_RECT_UNITS,
    mat_to_rmf,
    mat_to_rmf_xy,
    MatPose,
    METERS_PER_MAT_UNIT,
    rmf_to_mat_xy,
)


def test_top_left_origin_maps_to_zero():
    """The configured mat origin maps to RMF (0, 0)."""
    x, y = mat_to_rmf_xy(*MAT_ORIGIN_UNITS)
    assert x == 0.0 and y == 0.0


def test_bottom_right_corner():
    """The TMD01SS bottom-right corner maps to (15.2, 10.8) m."""
    x, y = mat_to_rmf_xy(402, 358)
    assert math.isclose(x, 15.2, abs_tol=1e-9)
    assert math.isclose(y, 10.8, abs_tol=1e-9)


def test_safe_rect_corners_in_rmf_meters():
    """Safe rect corners map to clean meter values."""
    x0, y0, x1, y1 = MAT_SAFE_RECT_UNITS
    assert mat_to_rmf_xy(x0, y0) == (2.0, 2.0)
    rx, ry = mat_to_rmf_xy(x1, y1)
    assert math.isclose(rx, 13.2, abs_tol=1e-9)
    assert math.isclose(ry, 8.8, abs_tol=1e-9)


def test_round_trip_xy():
    """Mat -> rmf -> mat is the identity for integer mat coords."""
    for mx, my in [(98, 142), (250, 250), (402, 358), (200, 200)]:
        rx, ry = mat_to_rmf_xy(mx, my)
        assert rmf_to_mat_xy(rx, ry) == (mx, my)


def test_inside_safe_rect():
    """The configured rect excludes points outside its bounds."""
    assert inside_safe_rect_mat(250, 250)
    assert not inside_safe_rect_mat(100, 100)


def test_pose_conversion_angle():
    """Mat 90 degrees maps to RMF pi/2 radians."""
    pose = MatPose(x=250, y=250, angle_deg=90)
    rmf = mat_to_rmf(pose)
    assert math.isclose(rmf.yaw_rad, math.pi / 2)


def test_meters_per_mat_unit_is_50x_scale():
    """Confirm the documented scale factor."""
    assert METERS_PER_MAT_UNIT == 0.05
