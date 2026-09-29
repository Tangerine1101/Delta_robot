"""Belt position and the parts riding on it.

* `BeltPositionTracker` holds the belt position (mm) reported by the Siemens PLC
  (`conveyor_position`) and derives its velocity; it keeps a short history so a detection can
  be anchored to the belt position at its capture instant.
* `BeltTracker` holds every part currently on the belt, anchored in belt coordinates and dead
  reckoned from the belt position once the camera no longer sees it.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Iterable

from modules.core.frames import ConveyorFrame, UVWindow


@dataclass(frozen=True)
class ObjectDetection:
    """One sighting of a part, in belt coordinates: `x` = u, `y` = v (mm)."""

    object_id: str
    x: float
    y: float
    object_type: str
    timestamp: float
    confidence: float = 1.0
    angle_deg: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "object_id": self.object_id,
            "x": self.x,
            "y": self.y,
            "object_type": self.object_type,
            "timestamp": self.timestamp,
            "confidence": self.confidence,
            "angle_deg": self.angle_deg,
        }


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

    Detections carry C-frame `(u, v)` in their `(x, y)` fields: the vision pipeline has
    already mapped pixels -> ROI mm -> belt coordinates.
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
