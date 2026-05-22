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
Coordinate transforms between toio mat units and the RMF map frame.

All constants below are derived from the toio official specification and
the project's fixed design decisions; no per-environment calibration is
required.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


METERS_PER_MAT_UNIT: float = 0.05
MAT_ORIGIN_UNITS: tuple[int, int] = (98, 142)
MAT_SAFE_RECT_UNITS: tuple[int, int, int, int] = (138, 182, 362, 318)


@dataclass(frozen=True)
class MatPose:
    """Pose reported by toio Position ID (mat units and degrees)."""

    x: int
    y: int
    angle_deg: int


@dataclass(frozen=True)
class RmfPose:
    """Pose expressed in the RMF map frame (meters and radians)."""

    x: float
    y: float
    yaw_rad: float


def mat_to_rmf_xy(mat_x: int, mat_y: int) -> tuple[float, float]:
    """Convert mat (x, y) in mat units to RMF (x, y) in meters."""
    ox, oy = MAT_ORIGIN_UNITS
    return ((mat_x - ox) * METERS_PER_MAT_UNIT,
            (mat_y - oy) * METERS_PER_MAT_UNIT)


def rmf_to_mat_xy(rmf_x: float, rmf_y: float) -> tuple[int, int]:
    """Convert RMF (x, y) in meters back to mat units, rounded to int."""
    ox, oy = MAT_ORIGIN_UNITS
    return (round(rmf_x / METERS_PER_MAT_UNIT) + ox,
            round(rmf_y / METERS_PER_MAT_UNIT) + oy)


def mat_angle_to_rmf_yaw(mat_deg: int) -> float:
    """Convert a mat angle in degrees to an RMF yaw in radians."""
    # Y is not flipped, so the RMF map frame is image-style (Y-down) and the
    # toio angle convention (X axis, clockwise positive) carries over verbatim.
    return math.radians(mat_deg)


def rmf_yaw_to_mat_angle(yaw_rad: float) -> int:
    """Convert an RMF yaw in radians to a mat angle in degrees (0-359)."""
    deg = math.degrees(yaw_rad) % 360
    return int(round(deg)) % 360


def mat_to_rmf(pose: MatPose) -> RmfPose:
    """Convert a full MatPose to an RmfPose."""
    x, y = mat_to_rmf_xy(pose.x, pose.y)
    return RmfPose(x=x, y=y, yaw_rad=mat_angle_to_rmf_yaw(pose.angle_deg))


def rmf_to_mat(pose: RmfPose) -> MatPose:
    """Convert a full RmfPose to a MatPose."""
    mx, my = rmf_to_mat_xy(pose.x, pose.y)
    return MatPose(x=mx, y=my, angle_deg=rmf_yaw_to_mat_angle(pose.yaw_rad))


def inside_safe_rect_mat(mat_x: int, mat_y: int) -> bool:
    """Return True if (mat_x, mat_y) lies inside the configured safe rect."""
    x0, y0, x1, y1 = MAT_SAFE_RECT_UNITS
    return x0 <= mat_x <= x1 and y0 <= mat_y <= y1
