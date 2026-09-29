"""Coordinate frames of the cell (doc/basis-theory.md §1).

* C-frame (belt): u along the belt flow (downstream positive), v across the belt surface,
  origin at the camera origin chosen during calibration.
* R-frame (robot): standard robot Cartesian frame, Z negative-down.
* ROI-mm (vision): the camera ROI in millimetres; a pure axis swap of the C-frame.

The robot frame is rotated by theta from the belt frame: the belt's downstream axis +u
expressed in the robot frame is (-sin theta, cos theta), and the cross-belt axis +v is
(cos theta, sin theta). With u along the flow and v across it, (u, v) -> (x_R, y_R) is

    x_R = -sin(theta)*u + cos(theta)*v + T_X
    y_R =  cos(theta)*u + sin(theta)*v + T_Y

`conveyor.frame.robot_origin_uv` is the ROBOT base position in belt coordinates; the
translation column is derived so the base maps to the R-frame origin:
Rot·(u_off, v_off) + T = (0, 0)  =>  T = -Rot·(u_off, v_off). To re-calibrate, run
`test_vision_only`, read a board's R-frame position on the dashboard and nudge
`robot_origin_uv` until it matches (open-issues C6).
"""

from __future__ import annotations

import math

from modules.core.angles import wrap_rad

Matrix3 = tuple[tuple[float, float, float], ...]
UVWindow = tuple[float, float, float, float]  # (u_min, u_max, v_min, v_max)

# Vision ROI frame -> C-frame (u, v). The camera origin and the belt origin are the SAME
# point, so the map has ZERO translation, and the belt flow (+u) is the camera +y axis, so it
# is a pure axis swap: u = y_mm, v = x_mm. The +y_mm sign is confirmed on the live belt: ROI
# y_mm increases as a board moves downstream, matching BeltTracker (which adds the belt
# advance). Do not flip it.
M_VISION_TO_CONVEYOR: Matrix3 = (
    (0.0, 1.0, 0.0),   # u = y_mm
    (1.0, 0.0, 0.0),   # v = x_mm
    (0.0, 0.0, 1.0),
)


def mat_apply(matrix: Matrix3, x: float, y: float) -> tuple[float, float]:
    """Apply a 3x3 homogeneous transform to the point (x, y)."""
    rx = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2]
    ry = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2]
    rw = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2]
    if rw == 0.0:
        return rx, ry
    return rx / rw, ry / rw


def mat_inverse(matrix: Matrix3) -> Matrix3:
    """Inverse of a 3x3 matrix. Raises if singular."""
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        raise ValueError("Matrix is singular; cannot invert.")
    inv_det = 1.0 / det
    return (
        ((e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det),
        ((f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det),
        ((d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det),
    )


def conveyor_to_robot_matrix(theta_deg: float, robot_origin_uv: tuple[float, float]) -> Matrix3:
    """Belt (u, v) -> robot (x, y) homogeneous transform; see the module docstring."""
    theta = math.radians(theta_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    u_off, v_off = float(robot_origin_uv[0]), float(robot_origin_uv[1])
    t_x = -(-sin_t * u_off + cos_t * v_off)
    t_y = -(cos_t * u_off + sin_t * v_off)
    return (
        (-sin_t, cos_t, t_x),
        (cos_t, sin_t, t_y),
        (0.0, 0.0, 1.0),
    )


class ConveyorFrame:
    """The belt -> robot transform F and the conversions built on it."""

    def __init__(self, theta_deg: float, robot_origin_uv: tuple[float, float]) -> None:
        self.F: Matrix3 = conveyor_to_robot_matrix(theta_deg, robot_origin_uv)
        self.F_inv: Matrix3 = mat_inverse(self.F)
        # Unit vectors of the belt axes in the R-frame (translation column dropped).
        self.u_hat: tuple[float, float] = (self.F[0][0], self.F[1][0])
        self.v_hat: tuple[float, float] = (self.F[0][1], self.F[1][1])
        # Frame rotation recovered from u_hat = (-sin θ, cos θ). The camera->C->R chain is a
        # pure +θ rotation, so a board's R-frame heading is its vision heading + θ.
        self.theta_rad: float = math.atan2(-self.u_hat[0], self.u_hat[1])

    @classmethod
    def from_settings(cls, conveyor) -> "ConveyorFrame":
        """Build from `Settings.conveyor`."""
        return cls(conveyor.frame.theta_deg, conveyor.frame.robot_origin_uv)

    def vision_heading_to_robot_rad(self, vision_heading_deg: float) -> float:
        """Vision marker heading (image-pixel degrees) -> R-frame heading (radians).

        The ONLY place the image-angle convention is translated; everything downstream works
        in R-frame radians, 0 = robot +X axis, positive = CCW seen from above, wrapped to
        [-pi, pi). The vision heading is atan2(dx, dy) on raw pixels, i.e. measured from the
        image +y (DOWN) axis, while the ROI->C->R chain (axis swap composed with F's
        reflection block) is a pure +theta rotation of angles measured CCW from the ROI x
        axis. The two references differ by exactly -90 deg, hence
        R_heading = radians(vision_heading - 90) + theta.
        """
        return wrap_rad(math.radians(vision_heading_deg - 90.0) + self.theta_rad)

    def to_robot(self, u: float, v: float) -> tuple[float, float]:
        return mat_apply(self.F, u, v)

    def to_conveyor(self, x: float, y: float) -> tuple[float, float]:
        return mat_apply(self.F_inv, x, y)

    def velocity_to_robot(self, s_u: float) -> tuple[float, float]:
        """Scalar belt speed (mm/s along +u) -> robot-frame (vx, vy)."""
        return self.u_hat[0] * s_u, self.u_hat[1] * s_u

    @staticmethod
    def is_in_window_uv(u: float, v: float, window: UVWindow) -> bool:
        u_min, u_max, v_min, v_max = window
        return u_min <= u <= u_max and v_min <= v <= v_max


def vision_mm_to_uv(x_mm: float, y_mm: float) -> tuple[float, float]:
    return mat_apply(M_VISION_TO_CONVEYOR, x_mm, y_mm)


def uv_to_vision_mm(u: float, v: float) -> tuple[float, float]:
    return mat_apply(mat_inverse(M_VISION_TO_CONVEYOR), u, v)


def is_within_xy_limit(x: float, y: float, limit_radius_xy: float) -> bool:
    """True if the R-frame point (x, y) is inside the reach circle around the robot origin."""
    return math.hypot(x, y) <= float(limit_radius_xy)


def belt_zone(u: float, camera_window_uv: UVWindow, workspace_window_uv: UVWindow) -> str:
    """Where on the belt a position u lies, for the dashboard: upstream, ROI (under the
    camera), transit (between camera and workspace), workspace, or past."""
    if u < camera_window_uv[0]:
        return "upstream"
    if u <= camera_window_uv[1]:
        return "ROI"
    if u < workspace_window_uv[0]:
        return "transit"
    if u <= workspace_window_uv[1]:
        return "workspace"
    return "past"
