"""
Conveyor coordinate frame, encoder decoding, and object tracking.

This module bundles:
- The homogeneous transform F from conveyor frame (u, v) to robot frame (X, Y).
- The homogeneous transform M_cam from camera pixels (px, py) directly to robot
  frame (X, Y). M_cam already absorbs the homography H (pixel -> conveyor) and F.
- BeltPositionTracker: stores the pre-decoded belt position (mm) reported by
  the PLC (`conveyor_position`, cm) and derives velocity (mm/s).
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

import math
from dataclasses import dataclass, field
from typing import Iterable

from modules.image_processing import ObjectDetection


# ---------------------------------------------------------------------------
# Calibrated transforms (3x3 homogeneous, row-major as nested tuples).
# Override these constants after running the calibration routine.
# ---------------------------------------------------------------------------

# Belt placed in the (-X, +Y) quadrant of the robot at ~30 degrees off the
# robot X-axis. Origin O_C at (-50, -100) in robot frame. Replace after calib.
_THETA_RAD = math.radians(28.0)
_COS_T = math.cos(_THETA_RAD)
_SIN_T = math.sin(_THETA_RAD)
_T_X = 393.0
_T_Y = 138.0

F_CONVEYOR_TO_ROBOT: tuple[tuple[float, float, float], ...] = (
    (_SIN_T, -_COS_T, _T_X),
    (_COS_T, _SIN_T, _T_Y),
    (0.0,     0.0,    1.0),
)

# Composite camera-pixel -> robot homogeneous transform.
# Placeholder: identity in u,v with a pixel-to-mm scale of 0.5 mm/pixel.
# Replace with H homography times F once camera is calibrated.
M_CAMERA_TO_ROBOT: tuple[tuple[float, float, float], ...] = (
    (-_COS_T, _SIN_T, 133.0),
    (_SIN_T,  _COS_T, 393.0),
    (0.0,     0.0,    1.0),
)

# Vision ROI frame → C-frame (u, v) transform.
# Vision ROI origin = bottom-left of ROI polygon; X = cross-belt; Y = along belt (upstream→downstream).
# Placeholder: u = y_mm + u_trigger_offset, v = x_mm + v_offset.
# Replace matrix values after physical calibration of trigger-line position and belt alignment.
#
# INTERIM (not a real calibration): the trigger line lives in the camera's view,
# which corresponds to camera_window_uv = [50, 250] (upstream). A board crossing
# the trigger projects to y_mm ≈ 124, so an offset of ~25 puts it at u ≈ 150 —
# near the camera-window centre and well below workspace u_max (620), so it
# survives BeltTracker.prune. The old value of 500 dropped fresh detections at
# u ≈ 624 (past u_max), making prune delete every detection before it was
# reported. The ROI is ~135 mm wide with x_mm ∈ [0, 135]; shifting by ~-67
# centres the belt at v ≈ 0 to match the windows' v ∈ [-65, 65].
# NOTE on the U sign: BeltTracker.current_uv always ADDS delta_p as the belt
# advances, so along-belt travel must INCREASE u. ROI y_mm decreases as a board
# moves downstream, so on a moving-belt run the `+y_mm` term below likely needs
# to become `-y_mm`; verify and flip once the belt is running.
_U_TRIGGER_OFFSET_MM = 25.0    # u (mm) in C-frame where the trigger line sits
_V_BELT_CENTER_MM = -67.0      # v shift from ROI X-zero to belt centre
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
    `conveyor_position`, in cm), so no quadrature decoding is needed here. Feed
    `update(position_mm, now)` with the position already converted to mm; the
    tracker stores it and computes velocity as the time derivative with a small
    EMA filter to smooth polling jitter.
    """

    def __init__(
        self,
        velocity_ema_alpha: float = 0.4,
    ) -> None:
        self.velocity_ema_alpha = float(velocity_ema_alpha)
        self._last_position_mm: float | None = None
        self._last_timestamp: float | None = None
        self._position_mm: float = 0.0
        self._velocity_mm_per_s: float = 0.0
        self._initialised: bool = False

    def update(self, position_mm: float, now: float) -> None:
        new_position = float(position_mm)

        if not self._initialised:
            self._position_mm = new_position
            self._last_position_mm = new_position
            self._last_timestamp = now
            self._initialised = True
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
    rotation_rad: float = 0.0
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
    ) -> None:
        self.frame = frame
        self.workspace_window_uv = workspace_window_uv
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
            obj = self._objects[detection.object_id]
            obj.last_seen_at = detection.timestamp
            obj.confidence = detection.confidence
            obj.rotation_rad = math.radians(detection.angle_deg)
            return obj

        obj = TrackedObject(
            object_id=detection.object_id,
            object_type=detection.object_type,
            conveyor_uv=(detection.x, detection.y),
            belt_pos_anchor=p_now,
            rotation_rad=math.radians(detection.angle_deg),
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

    def prune(self, p_now: float, now: float) -> int:
        """Drop objects past downstream u_max or older than stale_timeout."""
        removed = 0
        u_max = self.workspace_window_uv[1]
        for obj_id, obj in list(self._objects.items()):
            u_now, _ = obj.current_uv(p_now)
            if u_now > u_max or (now - obj.last_seen_at) > self.stale_timeout_s:
                self._objects.pop(obj_id, None)
                removed += 1
        return removed
