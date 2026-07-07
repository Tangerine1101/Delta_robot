"""
Conveyor coordinate frame, encoder decoding, and object tracking.

This module bundles:
- The homogeneous transform F from conveyor frame (u, v) to robot frame (X, Y).
- The homogeneous transform M_cam from camera pixels (px, py) directly to robot
  frame (X, Y). M_cam already absorbs the homography H (pixel -> conveyor) and F.
- BeltPositionTracker: stores the pre-decoded belt position (mm) reported by
  the PLC (`conveyor_position`, mm since June 2026) and derives velocity (mm/s).
- BeltTracker: maintains the live list of objects sitting on the belt and
  computes their current robot-frame position from the encoder reading.

Frame definitions:
- C-frame: u along belt flow (toward downstream), v perpendicular on belt surface,
  origin at a fixed marker on the belt body chosen during calibration.
- R-frame: standard robot Cartesian frame (Z is negative-down).

Both F and M_cam below are placeholders calibrated by hand. Replace the
matrix values once the physical setup is measured.
"""
from __future__ import annotations

import argparse
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable

from modules.EthernetCom import load_config, wrap_rad
from modules.image_processing import ObjectDetection


# ---------------------------------------------------------------------------
# Calibrated transforms (3x3 homogeneous, row-major as nested tuples).
# Override these constants after running the calibration routine.
# ---------------------------------------------------------------------------

# The robot frame is rotated by theta from the conveyor frame: the belt's
# downstream axis (+u, where the pick workspace lives) expressed in the robot
# frame is (-sin theta, cos theta), and the cross-belt axis (+v) is
# (cos theta, sin theta). With u along the belt flow and v cross-belt, the
# homogeneous map (u, v) -> (x_R, y_R) is:
#     x_R = -sin(theta)*u + cos(theta)*v + T_X
#     y_R =  cos(theta)*u + sin(theta)*v + T_Y
#
# Both theta and the belt offset now live in config.json under `conveyor.frame`:
#     "frame": { "theta_deg": 28.0, "robot_origin_uv": [360.0, 130.0] }
#
# `robot_origin_uv` (u_off, v_off) is the ROBOT base position expressed in the
# conveyor (u, v) frame — read straight off doc/frames.png (u=360 along
# X_conveyor, v=130 along Y_conveyor). The translation column is then DERIVED so
# the robot base maps to the R-frame origin:
#     Rot·(u_off, v_off) + T = (0, 0)   =>   T = -Rot·(u_off, v_off)
# This is the "multiply the offset by the rotation matrix" step that previously
# had to be done by hand to fill in T_X/T_Y. Just edit u/v in config now.
# Cross-check: (360, 130) -> T ≈ (54.2, -378.9), matching the old frames.png
# estimate (~54, -379). To re-calibrate, run `test_vision_only`, read the board's
# R-frame position on the dashboard, and nudge robot_origin_uv until it matches.


def _build_F_from_config(
    theta_deg: float, robot_origin_uv: tuple[float, float]
) -> "Matrix3":
    """Conveyor (u, v) -> robot (x, y) homogeneous transform.

    The 2x2 orientation block is fixed by `theta_deg`; the translation column is
    derived from `robot_origin_uv` (the robot base in conveyor coords) so the
    base maps to the R-frame origin: T = -Rot·(u_off, v_off).
    """
    theta = math.radians(theta_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    u_off, v_off = float(robot_origin_uv[0]), float(robot_origin_uv[1])
    # Rot·(u_off, v_off) using the (-sin, cos / cos, sin) block, then negate.
    t_x = -(-sin_t * u_off + cos_t * v_off)
    t_y = -(cos_t * u_off + sin_t * v_off)
    return (
        (-sin_t, cos_t, t_x),
        (cos_t, sin_t, t_y),
        (0.0, 0.0, 1.0),
    )


def _load_frame_params() -> tuple[float, tuple[float, float]]:
    """Read (theta_deg, robot_origin_uv) from config.json `conveyor.frame`."""
    conveyor_cfg = getattr(load_config(), "conveyor", {}) or {}
    frame_cfg = conveyor_cfg.get("frame", {}) or {}
    theta_deg = float(frame_cfg.get("theta_deg", 28.0))
    raw_uv = frame_cfg.get("robot_origin_uv", [360.0, 130.0])
    if isinstance(raw_uv, (list, tuple)) and len(raw_uv) >= 2:
        robot_origin_uv = (float(raw_uv[0]), float(raw_uv[1]))
    else:
        robot_origin_uv = (360.0, 130.0)
    return theta_deg, robot_origin_uv


_THETA_DEG, _ROBOT_ORIGIN_UV = _load_frame_params()
_THETA_RAD = math.radians(_THETA_DEG)
_COS_T = math.cos(_THETA_RAD)
_SIN_T = math.sin(_THETA_RAD)

F_CONVEYOR_TO_ROBOT: "Matrix3" = _build_F_from_config(_THETA_DEG, _ROBOT_ORIGIN_UV)
_T_X = F_CONVEYOR_TO_ROBOT[0][2]   # robot-X of the C-frame origin (derived)
_T_Y = F_CONVEYOR_TO_ROBOT[1][2]   # robot-Y of the C-frame origin (derived)

# Composite camera-pixel -> robot homogeneous transform.
# NOTE: `CameraFrame` / this matrix is NOT used at runtime — the vision pipeline
# maps camera ROI mm -> C-frame (u, v) via M_VISION_TO_CONVEYOR and then to the
# robot frame via F_CONVEYOR_TO_ROBOT. Kept as a placeholder for a future direct
# pixel->robot path; replace with H homography times F once camera is calibrated.
M_CAMERA_TO_ROBOT: "Matrix3" = (
    (-_COS_T, _SIN_T, _T_Y),
    (_SIN_T,  _COS_T, _T_X),
    (0.0,     0.0,    1.0),
)

# Vision ROI frame → C-frame (u, v) transform.
# FACTS: (1) the camera origin and the conveyor origin are the SAME point, so this
# map has ZERO translation; (2) the belt-flow direction (+u = +x_conveyor) is the
# camera +y axis, so this is a pure axis swap: u = y_mm, v = x_mm.
#
# A previous version added offsets (u+=25, v+=-67) which made camera (0,0) land at
# conveyor (25, -67) — that contradicted fact (1) and pushed every R-frame position
# off (mostly the y/flow direction). Removed.
#
# U SIGN — CONFIRMED on the live running belt: ROI y_mm INCREASES as a board moves
# downstream, matching BeltTracker.current_uv (which ADDS delta_p as the belt
# advances). The `+y_mm` term below is correct as written; do not flip it.
_U_TRIGGER_OFFSET_MM = 0.0    # origins coincide → no u offset
_V_BELT_CENTER_MM = 0.0       # origins coincide → no v offset
M_VISION_TO_CONVEYOR: tuple[tuple[float, float, float], ...] = (
    # row 0 → u = 0*x_mm + 1*y_mm + _U_TRIGGER_OFFSET_MM
    (0.0, 1.0, _U_TRIGGER_OFFSET_MM),
    # row 1 → v = 1*x_mm + 0*y_mm + _V_BELT_CENTER_MM
    (1.0, 0.0, _V_BELT_CENTER_MM),
    (0.0, 0.0, 1.0),
)


# ---------------------------------------------------------------------------
# Helpers — pure 3x3 matrix ops without numpy dependency.
# ---------------------------------------------------------------------------

Matrix3 = tuple[tuple[float, float, float], ...]


def _mat_apply(matrix: Matrix3, x: float, y: float) -> tuple[float, float]:
    """Apply 3x3 homogeneous transform to point (x, y)."""
    rx = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2]
    ry = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2]
    rw = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2]
    if rw == 0.0:
        return rx, ry
    return rx / rw, ry / rw


def _mat_inverse(matrix: Matrix3) -> Matrix3:
    """Compute 3x3 matrix inverse. Raises if singular."""
    m = matrix
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        raise ValueError("Matrix is singular; cannot invert.")
    inv_det = 1.0 / det
    return (
        ((e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det),
        ((f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det),
        ((d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det),
    )


# ---------------------------------------------------------------------------
# ConveyorFrame
# ---------------------------------------------------------------------------

UVWindow = tuple[float, float, float, float]  # (u_min, u_max, v_min, v_max)


class ConveyorFrame:
    """Wraps F (C -> R) and provides convenience transforms."""

    def __init__(self, F: Matrix3 = F_CONVEYOR_TO_ROBOT) -> None:
        self.F: Matrix3 = F
        self.F_inv: Matrix3 = _mat_inverse(F)
        # Pre-extract unit vectors u_hat and v_hat in the R-frame.
        # u_hat = R * (1, 0); v_hat = R * (0, 1). The translation column is dropped.
        self.u_hat: tuple[float, float] = (F[0][0], F[1][0])
        self.v_hat: tuple[float, float] = (F[0][1], F[1][1])
        # Frame rotation theta recovered from u_hat = (-sin θ, cos θ). The
        # camera->C->R chain is a pure +θ rotation, so a board's R-frame heading
        # is its vision heading + θ (used to normalise the suction angle).
        self.theta_rad: float = math.atan2(-self.u_hat[0], self.u_hat[1])

    def vision_heading_to_robot_rad(self, vision_heading_deg: float) -> float:
        """Vision marker heading (image-pixel degrees) -> R-frame heading (radians).

        The ONLY place the image-angle convention is translated; everything
        downstream (tracker, scheduler) works in R-frame radians, 0 = robot +X
        axis, positive = CCW seen from above, wrapped to [-pi, pi).

        Derivation: `heading_from_marker_vector` measures atan2(dx, dy) on raw
        pixels, i.e. from the image +y (DOWN) axis, while the ROI->C->R chain
        (M_VISION_TO_CONVEYOR axis swap composed with F's reflection block) is a
        pure +theta rotation of angles measured CCW from the ROI x axis. The two
        references differ by exactly -90 deg, hence:
            R_heading = radians(vision_heading - 90) + theta
        """
        return wrap_rad(math.radians(vision_heading_deg - 90.0) + self.theta_rad)

    def to_robot(self, u: float, v: float) -> tuple[float, float]:
        return _mat_apply(self.F, u, v)

    def to_conveyor(self, x: float, y: float) -> tuple[float, float]:
        return _mat_apply(self.F_inv, x, y)

    def velocity_to_robot(self, s_u: float) -> tuple[float, float]:
        """Map scalar belt speed (mm/s along +u) to robot-frame (vx, vy)."""
        return self.u_hat[0] * s_u, self.u_hat[1] * s_u

    @staticmethod
    def is_in_window_uv(u: float, v: float, window: UVWindow) -> bool:
        u_min, u_max, v_min, v_max = window
        return u_min <= u <= u_max and v_min <= v <= v_max


def is_within_xy_limit(x: float, y: float, limit_radius_xy: float) -> bool:
    """True if robot-frame point (x, y) is inside the physical reach circle of
    radius limit_radius_xy centred on the robot origin (0, 0).

    Robot R-frame, XY only (ignores Z, matching the `_xy` name). Points outside
    the circle are a physical forbidden zone the mechanism cannot reach.
    """
    return math.hypot(x, y) <= float(limit_radius_xy)


# ---------------------------------------------------------------------------
# CameraFrame
# ---------------------------------------------------------------------------


class CameraFrame:
    """Maps camera pixel coordinates directly to robot-frame XY."""

    def __init__(self, M: Matrix3 = M_CAMERA_TO_ROBOT) -> None:
        self.M: Matrix3 = M
        self.M_inv: Matrix3 = _mat_inverse(M)

    def pixel_to_robot(self, px: float, py: float) -> tuple[float, float]:
        return _mat_apply(self.M, px, py)

    def robot_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        return _mat_apply(self.M_inv, x, y)


# ---------------------------------------------------------------------------
# BeltPositionTracker
# ---------------------------------------------------------------------------


class BeltPositionTracker:
    """Track belt position (mm) and derive velocity from a pre-decoded position.

    The Siemens program now sends the belt position directly (field
    `conveyor_position`, in mm), so no quadrature decoding is needed here. Feed
    `update(position_mm, now)` with the position already converted to mm; the
    tracker stores it and computes velocity as the time derivative with a small
    EMA filter to smooth polling jitter.
    """

    def __init__(
        self,
        velocity_ema_alpha: float = 0.4,
        history_len: int = 200,
    ) -> None:
        self.velocity_ema_alpha = float(velocity_ema_alpha)
        self._last_position_mm: float | None = None
        self._last_timestamp: float | None = None
        self._position_mm: float = 0.0
        self._velocity_mm_per_s: float = 0.0
        self._initialised: bool = False
        # Ring buffer of (timestamp, position_mm) so a detection captured a few
        # tens of ms ago can be anchored to the belt position AT its capture
        # time (camera-latency compensation), not at ingest time. ~200 samples
        # at the 25 ms perception tick ≈ 5 s of history.
        self._history: deque[tuple[float, float]] = deque(maxlen=history_len)

    def update(self, position_mm: float, now: float) -> None:
        new_position = float(position_mm)

        if not self._initialised:
            self._position_mm = new_position
            self._last_position_mm = new_position
            self._last_timestamp = now
            self._initialised = True
            self._history.append((now, new_position))
            return

        last_ts = self._last_timestamp if self._last_timestamp is not None else now
        dt = max(0.0, now - last_ts)
        if dt > 0.0 and self._last_position_mm is not None:
            instantaneous = (new_position - self._last_position_mm) / dt
            self._velocity_mm_per_s = (
                self.velocity_ema_alpha * instantaneous
                + (1.0 - self.velocity_ema_alpha) * self._velocity_mm_per_s
            )
        self._position_mm = new_position
        self._last_position_mm = new_position
        self._last_timestamp = now
        self._history.append((now, new_position))

    def position_at(self, t: float, max_age_s: float = 1.0) -> float | None:
        """Belt position (mm) at past time ``t`` by linear interpolation over the
        history buffer. Returns None if the buffer is empty or ``t`` is older than
        ``max_age_s`` before the oldest sample (too stale to trust)."""
        if not self._history:
            return None
        oldest_t, oldest_p = self._history[0]
        newest_t, newest_p = self._history[-1]
        if t >= newest_t:
            return newest_p
        if t <= oldest_t:
            # Only extrapolate a little past the oldest sample; else give up.
            return oldest_p if (oldest_t - t) <= max_age_s else None
        # Binary/linear scan for the bracketing pair (history is time-ordered).
        prev_t, prev_p = oldest_t, oldest_p
        for sample_t, sample_p in self._history:
            if sample_t >= t:
                span = sample_t - prev_t
                if span <= 0.0:
                    return sample_p
                frac = (t - prev_t) / span
                return prev_p + (sample_p - prev_p) * frac
            prev_t, prev_p = sample_t, sample_p
        return newest_p

    @property
    def position_mm(self) -> float:
        return self._position_mm

    @property
    def velocity_mm_per_s(self) -> float:
        return self._velocity_mm_per_s

    @property
    def initialised(self) -> bool:
        return self._initialised


# ---------------------------------------------------------------------------
# BeltTracker
# ---------------------------------------------------------------------------


@dataclass
class TrackedObject:
    """An object currently anchored to the belt in C-frame coordinates."""

    object_id: str
    object_type: str
    conveyor_uv: tuple[float, float]   # (u_i, v_i) — anchor in C-frame
    belt_pos_anchor: float             # encoder position p when first detected
    # Board heading in the R-frame (radians, [-pi, pi), 0 = robot +X, CCW from
    # above) — converted from the raw vision angle at ingest.
    rotation_rad: float = 0.0
    # Raw vision heading (image-pixel degrees) as emitted — kept for [ROTATE]
    # calibration logs only; never used in computations.
    vision_angle_deg: float = 0.0
    w_mm: float = 0.0
    h_mm: float = 0.0
    last_seen_at: float = 0.0
    confidence: float = 1.0

    def current_uv(self, p_now: float) -> tuple[float, float]:
        """Current C-frame position given current belt encoder reading."""
        delta_p = p_now - self.belt_pos_anchor
        return (self.conveyor_uv[0] + delta_p, self.conveyor_uv[1])


class BeltTracker:
    """Live list of objects sitting on the belt.

    Detections come from `image_processing.ObjectDetection`. For phase 1 we
    treat their `(x, y)` as already-resolved C-frame `(u, v)` so the scheduler
    can be wired up end-to-end without the full image pipeline. Phase 3 will
    replace this with real pixel-to-uv conversion through CameraFrame.
    """

    def __init__(
        self,
        frame: ConveyorFrame,
        workspace_window_uv: UVWindow,
        match_radius_mm: float = 15.0,
        stale_timeout_s: float = 5.0,
        camera_window_uv: UVWindow | None = None,
    ) -> None:
        self.frame = frame
        self.workspace_window_uv = workspace_window_uv
        # Downstream edge of the camera field of view (u). Objects upstream of
        # this still ought to be re-detected every frame, so the stale timeout
        # applies to them; once an object dead-reckons past it the camera can no
        # longer see it, so we keep it on the belt until it leaves the workspace.
        self.camera_window_uv = camera_window_uv
        self.match_radius_mm = float(match_radius_mm)
        self.stale_timeout_s = float(stale_timeout_s)
        self._objects: dict[str, TrackedObject] = {}

    def ingest_detection(
        self,
        detection: ObjectDetection,
        p_now: float,
        *,
        object_dimensions: dict[str, tuple[float, float]] | None = None,
    ) -> TrackedObject:
        """Register or refresh a tracked object from a single detection.

        Detection.x and Detection.y are taken to be C-frame (u, v) coordinates.
        """
        dims = (0.0, 0.0)
        if object_dimensions is not None:
            dims = object_dimensions.get(detection.object_type, (0.0, 0.0))

        if detection.object_id in self._objects:
            # Re-anchor from the camera: while the object is visible the camera
            # position is authoritative. We move the C-frame anchor to the fresh
            # detection and reset belt_pos_anchor to p_now, so dead reckoning
            # (current_uv adds p_now - belt_pos_anchor) resumes from this latest
            # camera fix once the object leaves the camera zone and stops emitting.
            obj = self._objects[detection.object_id]
            obj.conveyor_uv = (detection.x, detection.y)
            obj.belt_pos_anchor = p_now
            obj.last_seen_at = detection.timestamp
            obj.confidence = detection.confidence
            obj.rotation_rad = self.frame.vision_heading_to_robot_rad(detection.angle_deg)
            obj.vision_angle_deg = detection.angle_deg
            return obj

        obj = TrackedObject(
            object_id=detection.object_id,
            object_type=detection.object_type,
            conveyor_uv=(detection.x, detection.y),
            belt_pos_anchor=p_now,
            rotation_rad=self.frame.vision_heading_to_robot_rad(detection.angle_deg),
            vision_angle_deg=detection.angle_deg,
            w_mm=dims[0],
            h_mm=dims[1],
            last_seen_at=detection.timestamp,
            confidence=detection.confidence,
        )
        self._objects[detection.object_id] = obj
        return obj

    def remove(self, object_id: str) -> None:
        self._objects.pop(object_id, None)

    def objects(self) -> Iterable[TrackedObject]:
        return list(self._objects.values())

    def current_position_R(self, obj: TrackedObject, p_now: float) -> tuple[float, float]:
        u_now, v_now = obj.current_uv(p_now)
        return self.frame.to_robot(u_now, v_now)

    def predict_position_R(
        self,
        obj: TrackedObject,
        p_now: float,
        belt_velocity_mm_per_s: float,
        dt_future_s: float,
    ) -> tuple[float, float]:
        """Predict where the object will be in R-frame `dt_future_s` from now.

        Uses encoder position as the anchor (no time-integrated drift) and adds
        the future belt displacement `v * dt` only for the look-ahead window.
        """
        u_now, v_now = obj.current_uv(p_now)
        u_future = u_now + belt_velocity_mm_per_s * dt_future_s
        return self.frame.to_robot(u_future, v_now)

    def should_prune(self, obj: TrackedObject, p_now: float, now: float) -> bool:
        """Whether an object has left the belt span we care about.

        Drop it once it dead-reckons past the downstream workspace edge
        (``u_max``). The stale timeout only fires while the object is still
        within the camera field of view: there the camera ought to re-detect it
        every frame, so a gap means it is gone (picked by hand / false
        positive). Downstream of the camera the camera is blind, so a tracked
        object is kept (dead-reckoned on the encoder) all the way through the
        workspace — that is what keeps it visible from the ROI to the end of
        the workspace. When ``camera_window_uv`` is unset, the stale timeout
        applies everywhere (legacy behaviour).
        """
        u_now, _ = obj.current_uv(p_now)
        if u_now > self.workspace_window_uv[1]:
            return True
        in_camera_fov = (
            self.camera_window_uv is None or u_now <= self.camera_window_uv[1]
        )
        return in_camera_fov and (now - obj.last_seen_at) > self.stale_timeout_s

    def prune(self, p_now: float, now: float) -> int:
        """Drop objects that have left the tracked belt span (see should_prune)."""
        removed = 0
        for obj_id, obj in list(self._objects.items()):
            if self.should_prune(obj, p_now, now):
                self._objects.pop(obj_id, None)
                removed += 1
        return removed


# ---------------------------------------------------------------------------
# CLI — quick coordinate-frame calculator / transform-logic verifier.
#
# Usage:
#   python3 -m modules.conveyor --robot 0 -285 -310
#   python3 -m modules.conveyor --conveyor 112.5 20.2
#   python3 -m modules.conveyor --pixel 800 600
# ---------------------------------------------------------------------------


def _build_roi_frame(vision_cfg: dict):
    import cv2
    import numpy as np

    from modules.image_processing import RoiFrame

    roi_cfg = vision_cfg.get("roi", {}) or {}
    polygon = roi_cfg.get("polygon")
    pixels_per_mm = vision_cfg.get("pixels_per_mm", 1.0)
    return RoiFrame(polygon, pixels_per_mm, np, cv2)


def _roi_mm_to_uv(x_mm: float, y_mm: float) -> tuple[float, float]:
    return _mat_apply(M_VISION_TO_CONVEYOR, x_mm, y_mm)


def _uv_to_roi_mm(u: float, v: float) -> tuple[float, float]:
    return _mat_apply(_mat_inverse(M_VISION_TO_CONVEYOR), u, v)


def _convert(roi_frame: "RoiFrame", conv_frame: ConveyorFrame, frame: str, values: tuple[float, ...]) -> dict:
    """Compute pixel / ROI-mm / conveyor(u,v) / robot(x,y) for a point given in `frame`.

    Also re-derives the source frame's own coordinates from the computed chain
    (round-trip check) to verify the forward+inverse transforms agree.
    """
    z = values[2] if frame == "robot" and len(values) == 3 else None

    if frame == "robot":
        x, y = values[0], values[1]
        u, v = conv_frame.to_conveyor(x, y)
        x_mm, y_mm = _uv_to_roi_mm(u, v)
        px, py = roi_frame.mm_to_pixel(x_mm, y_mm)
        x_rt, y_rt = conv_frame.to_robot(u, v)
        roundtrip = (x_rt - x, y_rt - y)
    elif frame == "conveyor":
        u, v = values[0], values[1]
        x, y = conv_frame.to_robot(u, v)
        x_mm, y_mm = _uv_to_roi_mm(u, v)
        px, py = roi_frame.mm_to_pixel(x_mm, y_mm)
        u_rt, v_rt = conv_frame.to_conveyor(x, y)
        roundtrip = (u_rt - u, v_rt - v)
    elif frame == "pixel":
        px, py = values[0], values[1]
        x_mm, y_mm = roi_frame.to_mm(px, py)
        u, v = _roi_mm_to_uv(x_mm, y_mm)
        x, y = conv_frame.to_robot(u, v)
        px_rt, py_rt = roi_frame.mm_to_pixel(x_mm, y_mm)
        roundtrip = (px_rt - px, py_rt - py)
    else:
        raise ValueError(f"Unknown frame: {frame}")

    return {
        "frame": frame,
        "z": z,
        "pixel": (px, py),
        "roi_mm": (x_mm, y_mm),
        "conveyor": (u, v),
        "robot": (x, y),
        "roundtrip": roundtrip,
        "roi_ok": roi_frame._ok,
        "inside_roi": roi_frame.contains(px, py),
    }


def _print_report(result: dict, limit_radius_xy: float | None = None) -> None:
    frame = result["frame"]
    z = result["z"]
    px, py = result["pixel"]
    x_mm, y_mm = result["roi_mm"]
    u, v = result["conveyor"]
    x, y = result["robot"]
    z_str = f"{z:.3f}" if z is not None else "n/a"

    input_values = {
        "robot": f"x={x:.3f}, y={y:.3f}, z={z_str}",
        "conveyor": f"u={u:.3f}, v={v:.3f}",
        "pixel": f"px={px:.1f}, py={py:.1f}",
    }[frame]
    print(f"Input: {frame} ({input_values})\n")

    if not result["roi_ok"]:
        print("  [WARN] vision.roi.polygon not configured/enabled — pixel <-> mm uses a"
              " degraded raw px/pixels_per_mm fallback (no rotation/origin correction).\n")

    inside = "yes" if result["inside_roi"] else "no"
    print(f"  Robot     (x, y, z) = ({x:.3f}, {y:.3f}, {z_str}) mm")
    print(f"  Conveyor  (u, v)    = ({u:.3f}, {v:.3f}) mm")
    print(f"  ROI-mm    (x, y)    = ({x_mm:.3f}, {y_mm:.3f}) mm")
    print(f"  Pixel     (px, py)  = ({px:.1f}, {py:.1f}) px   [inside ROI: {inside}]")

    if limit_radius_xy is not None:
        radius = math.hypot(x, y)
        within = "yes" if is_within_xy_limit(x, y, limit_radius_xy) else "no"
        print(f"  Reach     |x, y|    = {radius:.3f} mm   "
              f"[within XY limit {limit_radius_xy:.1f} mm: {within}]")

    d0, d1 = result["roundtrip"]
    print(f"\nRound-trip check (back to {frame} frame): delta = ({d0:.4f}, {d1:.4f})")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Convert a point between robot (R), conveyor (u,v), ROI-mm, and "
                     "camera pixel frames. Also reports a round-trip delta to verify "
                     "the forward+inverse transform logic agrees with itself."
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--robot", type=float, nargs="+", metavar=("X", "Y"),
                        help="Robot-frame point: X Y [Z] (mm)")
    group.add_argument("--conveyor", type=float, nargs=2, metavar=("U", "V"),
                        help="Conveyor-frame point: U V (mm)")
    group.add_argument("--pixel", type=float, nargs=2, metavar=("PX", "PY"),
                        help="Camera pixel-frame point: PX PY")
    args = parser.parse_args(argv)

    if args.robot is not None and len(args.robot) not in (2, 3):
        parser.error("--robot takes 2 (X Y) or 3 (X Y Z) values")

    cfg = load_config()
    roi_frame = _build_roi_frame(getattr(cfg, "vision", {}) or {})
    conv_frame = ConveyorFrame()
    limit_radius_xy = float(getattr(cfg, "limit_radius_xy", 180.0))

    if args.robot is not None:
        frame, values = "robot", tuple(args.robot)
    elif args.conveyor is not None:
        frame, values = "conveyor", tuple(args.conveyor)
    else:
        frame, values = "pixel", tuple(args.pixel)

    result = _convert(roi_frame, conv_frame, frame, values)
    _print_report(result, limit_radius_xy)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
