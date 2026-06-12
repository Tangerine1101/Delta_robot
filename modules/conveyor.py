"""
Conveyor coordinate frame, encoder decoding, and object tracking.

This module bundles:
- The homogeneous transform F from conveyor frame (u, v) to robot frame (X, Y).
- The homogeneous transform M_cam from camera pixels (px, py) directly to robot
  frame (X, Y). M_cam already absorbs the homography H (pixel -> conveyor) and F.
- EncoderDecoder: converts raw (encoderA, encoderB) DINT counts to belt
  position (mm) and velocity (mm/s).
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
_THETA_RAD = math.radians(30.0)
_COS_T = math.cos(_THETA_RAD)
_SIN_T = math.sin(_THETA_RAD)
_T_X = -50.0
_T_Y = -100.0

F_CONVEYOR_TO_ROBOT: tuple[tuple[float, float, float], ...] = (
    (_COS_T, -_SIN_T, _T_X),
    (_SIN_T,  _COS_T, _T_Y),
    (0.0,     0.0,    1.0),
)

# Composite camera-pixel -> robot homogeneous transform.
# Placeholder: identity in u,v with a pixel-to-mm scale of 0.5 mm/pixel.
# Replace with H homography times F once camera is calibrated.
M_CAMERA_TO_ROBOT: tuple[tuple[float, float, float], ...] = (
    (0.5 * _COS_T, -0.5 * _SIN_T, _T_X),
    (0.5 * _SIN_T,  0.5 * _COS_T, _T_Y),
    (0.0,           0.0,          1.0),
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
# EncoderDecoder
# ---------------------------------------------------------------------------


class EncoderDecoder:
    """Decode raw quadrature (encoderA, encoderB) into signed position and velocity.

    The decoder is intentionally simple: it assumes the PLC HSC already
    accumulates a consistent signed count on both channels. The signed count is
    the average of the two channels to reject one-bit noise on either channel.
    Multiply by encoder_constant_mm_per_pulse (signed) to get position in mm.
    Velocity is the time derivative of position with a small EMA filter to
    smooth jitter from polling jitter.

    Final decode formula will be locked once the Siemens program is finalized;
    revisit the body of update() then.
    """

    def __init__(
        self,
        encoder_constant_mm_per_pulse: float,
        velocity_ema_alpha: float = 0.4,
    ) -> None:
        self.encoder_constant = float(encoder_constant_mm_per_pulse)
        self.velocity_ema_alpha = float(velocity_ema_alpha)
        self._last_position_mm: float | None = None
        self._last_timestamp: float | None = None
        self._position_mm: float = 0.0
        self._velocity_mm_per_s: float = 0.0
        self._initialised: bool = False

    @staticmethod
    def _decode_signed_count(encoder_a: int, encoder_b: int) -> float:
        """Combine the two channels into a single signed count."""
        return (encoder_a + encoder_b) / 2.0

    def update(self, encoder_a: int, encoder_b: int, now: float) -> None:
        signed_count = self._decode_signed_count(encoder_a, encoder_b)
        new_position = signed_count * self.encoder_constant

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
            return obj

        obj = TrackedObject(
            object_id=detection.object_id,
            object_type=detection.object_type,
            conveyor_uv=(detection.x, detection.y),
            belt_pos_anchor=p_now,
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
