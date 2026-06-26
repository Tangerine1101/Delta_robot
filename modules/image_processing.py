"""
Image processing module — self-contained vision pipeline.

This module was rebuilt from scratch (the previous version was a fragile
conversion of the teammate's ``YOLO_OBB/src/detect_realtime.py`` Windows/WSL
script). It no longer imports anything from ``YOLO_OBB/`` at runtime and no
longer reads ``system_config.yaml`` — every parameter comes from the ``vision``
section of ``modules/config.json``.

Two public detection sources, both exposing the same ``poll(now)`` interface:

- ``SimulatedImageProcessing`` — deterministic fake stream for the offline
  scheduler scenarios (test_accuracy / test_throughput / evaluate). No heavy
  dependencies.
- ``VisionImageProcessing`` — real-time YOLO-OBB pipeline. Frames are captured
  with **PyAV** (FFmpeg-backed) in a dedicated thread so the camera runs at its
  true rate (~30 fps at 1080p MJPG); the previous ``cv2.VideoCapture`` V4L2
  backend was the real <20 fps bottleneck, not the model or the GPU. Inference
  runs in a second thread; the OpenCV GUI is pumped from the main thread (Qt
  requirement). Detections are emitted in conveyor C-frame ``(u, v)`` via
  ``modules.conveyor.M_VISION_TO_CONVEYOR``.

Also computes a belt-speed estimate from object tracking
(``BeltVelocityEstimator``). This is **informational only** — it is logged and
drawn on the overlay but is NOT fed to the scheduler; the operational belt
position/velocity still comes from the Siemens ``conveyor_position`` field.
"""
from __future__ import annotations

import collections
import math
import os
import shutil
import statistics
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any

# OpenCV's bundled Qt has no Wayland plugin; force xcb (X11/XWayland) so the live
# window actually maps. Override unconditionally — the desktop may export wayland.
os.environ["QT_QPA_PLATFORM"] = "xcb"

# Stop ultralytics from auto-pip-installing optional deps (e.g. albumentations)
# at predict time — that pulls in `opencv-python-headless`, which has NO GUI and
# silently breaks `cv2.imshow` (the live window). Keep the GUI `opencv-python`.
os.environ.setdefault("YOLO_AUTOINSTALL", "false")


# ---------------------------------------------------------------------------
# Camera device helpers
# ---------------------------------------------------------------------------

def _find_camera_by_usb_id(vendor_product: str) -> int | None:
    """Find the lowest /dev/videoN index whose USB vendor:product matches.

    `vendor_product` must be 'vid:pid' or 'vid/pid' (hex, case-insensitive).
    Scans /sys/class/video4linux/videoN/device/uevent for PRODUCT=vid/pid/...
    Returns None if not found or if /sys is unavailable.

    Note: the kernel strips leading zeros in the PRODUCT field (e.g. 0c45 → c45),
    so we normalize both sides to bare hex integers before comparing.
    """
    parts = vendor_product.replace(":", "/").lower().split("/")
    if len(parts) < 2:
        return None
    try:
        vp = f"{int(parts[0], 16):x}/{int(parts[1], 16):x}"
    except ValueError:
        return None
    base = "/sys/class/video4linux"
    if not os.path.isdir(base):
        return None
    candidates: list[int] = []
    try:
        for name in os.listdir(base):
            if not name.startswith("video"):
                continue
            try:
                idx = int(name[5:])
            except ValueError:
                continue
            uevent = os.path.join(base, name, "device", "uevent")
            try:
                with open(uevent, "r") as f:
                    content = f.read().lower()
            except OSError:
                continue
            if f"product={vp}/" in content or f"product={vp}\n" in content:
                dev_path = f"/dev/video{idx}"
                if os.path.exists(dev_path):
                    candidates.append(idx)
    except OSError:
        return None
    return min(candidates) if candidates else None


def _apply_v4l2_controls(device: str, controls: dict) -> None:
    """Best-effort apply v4l2 controls to a /dev/videoN device via `v4l2-ctl`.

    Used to fix two camera quirks before opening the stream:
      * ``exposure_dynamic_framerate=0`` — stop UVC auto-exposure from dropping
        the frame rate in dim light.
      * a short ``exposure_time_absolute`` (with ``auto_exposure=1`` manual) —
        the exposure integration time must be < 1/fps or the sensor cannot
        sustain the rated frame rate.
    No-op off Linux or when `v4l2-ctl` is absent.
    """
    if not controls or sys.platform != "linux":
        return
    if shutil.which("v4l2-ctl") is None:
        print("[VISION] v4l2-ctl not found — skipping camera control tuning "
              "(install v4l-utils for full FPS).")
        return
    for name, value in controls.items():
        try:
            subprocess.run(
                ["v4l2-ctl", "--device", device, f"--set-ctrl={name}={value}"],
                check=False, capture_output=True, timeout=2.0,
            )
        except Exception as exc:
            print(f"[VISION] Could not set {name}={value} on {device}: {exc}")


# ---------------------------------------------------------------------------
# Detection record (shared with the scheduler)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ObjectDetection:
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


# ---------------------------------------------------------------------------
# Simulated stream (unchanged behaviour — used by offline scheduler scenarios)
# ---------------------------------------------------------------------------

class SimulatedImageProcessing:
    """Deterministic fake object stream for scheduler development.

    Emits detections at C-frame `(u, v)` coordinates.
    - `throughput_spawn_y` is reused as the upstream `u_spawn`.
    - `throughput_lanes` is reused as the per-object `v_lane` values.
    - `accuracy_spawn_uv`: list of [u, v] points inside workspace_window_uv used
      by test_accuracy/test_acceptance. Separate from `accuracy_points` (which are
      robot-frame XYZ targets used only by the evaluate scenario).
    - `accuracy_object_types`: object type cycled across each wave for
      test_accuracy/test_acceptance (default `["TQFP"]`). Independent of
      `throughput_object_types`, which only test_throughput reads.
    """

    # Scenarios that spawn a static wave of fake objects and wait for the robot to
    # finish all of them before spawning the next wave (see notify_pick_finished).
    _WAVE_GATED_SCENARIOS = ("test_accuracy", "test_acceptance")

    def __init__(self, scenario_name: str, config: dict[str, Any], start_time: float) -> None:
        self.scenario_name = scenario_name
        self.config = config
        self.start_time = start_time
        self.next_emit_at = start_time
        self.counter = 0
        self.throughput_types = list(config.get("throughput_object_types", ["pcb1", "pcb2"]))
        self.throughput_lanes = list(config.get("throughput_lanes", [-40.0, 0.0, 40.0]))
        self.u_spawn = float(config.get("throughput_spawn_y", -50.0))
        self.throughput_emit_interval_s = float(config.get("throughput_emit_interval_s", 0.35))
        self.accuracy_emit_interval_s = float(config.get("accuracy_emit_interval_s", 0.8))
        raw_spawn_uv = config.get(
            "accuracy_spawn_uv",
            [[470.0, 40.0], [520.0, 10.0], [560.0, -30.0]],
        )
        self.accuracy_spawn_uv = [(float(pt[0]), float(pt[1])) for pt in raw_spawn_uv]
        # Independent from throughput_types: test_accuracy/test_acceptance spawn a
        # fixed, controlled object mix (defaults to TQFP-only) regardless of what
        # test_throughput is configured to cycle through.
        self.accuracy_object_types = list(config.get("accuracy_object_types", ["TQFP"]))
        self._wave_gated = scenario_name in self._WAVE_GATED_SCENARIOS
        self._wave_pending: set[str] = set()

    def poll(self, now: float) -> list[ObjectDetection]:
        if self._wave_gated:
            if self._wave_pending:
                return []  # previous wave still in flight — hold until it clears
            wave = [self._build_detection(now) for _ in self.accuracy_spawn_uv]
            self._wave_pending = {detection.object_id for detection in wave}
            return wave

        detections: list[ObjectDetection] = []
        interval = self._scenario_interval()
        while now >= self.next_emit_at:
            detections.append(self._build_detection(self.next_emit_at))
            self.next_emit_at += interval
        return detections

    def notify_pick_finished(self, object_id: str) -> None:
        """Release the wave gate for an attempted (succeeded or failed) object."""
        self._wave_pending.discard(object_id)

    def _scenario_interval(self) -> float:
        if self.scenario_name == "test_accuracy":
            return self.accuracy_emit_interval_s
        return self.throughput_emit_interval_s

    def _build_detection(self, timestamp: float) -> ObjectDetection:
        self.counter += 1
        if self.scenario_name in self._WAVE_GATED_SCENARIOS:
            u, v = self.accuracy_spawn_uv[(self.counter - 1) % len(self.accuracy_spawn_uv)]
            object_type = self.accuracy_object_types[(self.counter - 1) % len(self.accuracy_object_types)]
            return ObjectDetection(
                object_id=f"accuracy-{self.counter:06d}",
                x=u,
                y=v,
                object_type=object_type,
                timestamp=timestamp,
            )

        v_lane = self.throughput_lanes[(self.counter - 1) % len(self.throughput_lanes)]
        object_type = self.throughput_types[(self.counter - 1) % len(self.throughput_types)]
        return ObjectDetection(
            object_id=f"throughput-{self.counter:06d}",
            x=self.u_spawn,
            y=float(v_lane),
            object_type=object_type,
            timestamp=timestamp,
        )


# ===========================================================================
# Core vision logic — inlined, self-contained (was YOLO_OBB/src/*)
# ===========================================================================

# An OBB detection tuple: (cx, cy, w, h, theta_rad, cls_id, conf)
Obb = tuple[float, float, float, float, float, int, float]


def normalize_angle_deg(theta_rad: float) -> float:
    """Map an OBB angle (radians) to degrees in [-90, 90).

    A rectangular board is symmetric under 180° rotation, so we fold the angle
    into a half-open 180° window centred on 0 — the smallest rotation a gripper
    must apply.
    """
    deg = math.degrees(theta_rad)
    return (deg + 90.0) % 180.0 - 90.0


def extract_obb(result, np_mod) -> list[Obb]:
    """Return (cx, cy, w, h, theta_rad, cls_id, conf) for each OBB detection."""
    obb = getattr(result, "obb", None)
    if obb is None or obb.xywhr is None:
        return []
    xywhr = obb.xywhr.cpu().numpy()       # (N, 5): cx, cy, w, h, theta
    cls = obb.cls.cpu().numpy().astype(int)
    conf = obb.conf.cpu().numpy()
    out: list[Obb] = []
    for i in range(len(xywhr)):
        cx, cy, w, h, theta = xywhr[i]
        out.append((float(cx), float(cy), float(w), float(h), float(theta), int(cls[i]), float(conf[i])))
    return out


@dataclass
class Track:
    """A tracked centroid. Carries trigger state and per-frame velocity."""
    id: int
    cx: float
    cy: float
    t: float
    missing: int = 0
    prev_side: int = 0           # set by TriggerLine: -1 upstream, +1 downstream
    triggered: bool = False
    frames: int = 1              # consecutive frames matched
    vx: float = 0.0              # px/s, last instantaneous velocity
    vy: float = 0.0


class CentroidTracker:
    """Lightweight nearest-neighbour centroid tracker.

    The belt is slow, single-lane, unidirectional and non-occluding, so heavy
    trackers (ByteTrack/BoT-SORT) are unnecessary — we just need a stable id per
    board so the trigger fires exactly once, plus a per-track velocity for the
    belt-speed estimator.
    """

    def __init__(self, max_match_dist: float = 80.0, max_missing: int = 15) -> None:
        self.max_match_dist = float(max_match_dist)
        self.max_missing = int(max_missing)
        self._next_id = 1
        self.tracks: dict[int, Track] = {}

    def update(self, detections: list[tuple[float, float]], now: float) -> dict[int, Track]:
        """Match (cx, cy) centroids to nearest unclaimed track within
        max_match_dist, else start a new track. Returns id -> Track for tracks
        seen this frame, and updates per-track instantaneous velocity (px/s).

        The match distance is measured against each track's *velocity-predicted*
        position `(cx + vx*dt, cy + vy*dt)`, not its last position. On a fast
        belt at low inference fps a board can jump far between processed frames;
        matching on the last position would exceed `max_match_dist` and spawn a
        new id every frame. Predicting forward keeps the same id while moving."""
        unmatched_ids = set(self.tracks.keys())
        matched_ids: set[int] = set()

        for cx, cy in detections:
            best_id, best_d = None, self.max_match_dist
            for tid in unmatched_ids:
                t = self.tracks[tid]
                dt = max(0.0, now - t.t)
                pred_cx = t.cx + t.vx * dt
                pred_cy = t.cy + t.vy * dt
                d = math.hypot(pred_cx - cx, pred_cy - cy)
                if d < best_d:
                    best_id, best_d = tid, d
            if best_id is None:
                tid = self._next_id
                self.tracks[tid] = Track(id=tid, cx=cx, cy=cy, t=now)
                matched_ids.add(tid)
                self._next_id += 1
            else:
                t = self.tracks[best_id]
                dt = now - t.t
                if dt > 0.0:
                    t.vx = (cx - t.cx) / dt
                    t.vy = (cy - t.cy) / dt
                t.cx, t.cy, t.t, t.missing = cx, cy, now, 0
                t.frames += 1
                unmatched_ids.discard(best_id)
                matched_ids.add(best_id)

        # Age / retire tracks not matched this frame.
        for tid in list(unmatched_ids):
            t = self.tracks[tid]
            t.missing += 1
            if t.missing > self.max_missing:
                del self.tracks[tid]

        return {tid: self.tracks[tid] for tid in matched_ids}


class TriggerLine:
    """One-shot trigger-line crossing detector.

    A board fires exactly once: when its tracked centroid transitions from the
    UPSTREAM side of the fixed line to the DOWNSTREAM side.
    """

    def __init__(self, y_px: int, direction: str = "down") -> None:
        self.y_px = int(y_px)
        # +1 means downstream is the side with LARGER py (belt moves top->bottom)
        self.sign = 1 if direction == "down" else -1

    def _side(self, cy: float) -> int:
        return 1 if (cy - self.y_px) * self.sign > 0 else -1

    def crossed(self, track: Track) -> bool:
        side = self._side(track.cy)
        crossed = (not track.triggered) and track.prev_side == -1 and side == 1
        track.prev_side = side
        if crossed:
            track.triggered = True
        return crossed


class RoiFrame:
    """ROI polygon + precomputed coordinate basis.

    Origin O = polygon[3] (bottom-left); X axis = O→polygon[2] (bottom-right);
    Y axis = O→polygon[0] (top-left). `to_mm` projects a pixel onto that frame
    and scales by pixels_per_mm. The basis is computed once (the old code
    recomputed it for every detection).
    """

    def __init__(self, polygon, pixels_per_mm: float, np_mod, cv2_mod) -> None:
        self._np = np_mod
        self._cv2 = cv2_mod
        self.pixels_per_mm = float(pixels_per_mm)
        self.poly = np_mod.array(polygon, dtype=np_mod.int32) if polygon else None
        self._ok = False
        if self.poly is not None and len(self.poly) >= 4 and self.pixels_per_mm > 0:
            O = self.poly[3].astype(float)
            Xp = self.poly[2].astype(float)
            Yp = self.poly[0].astype(float)
            xlen = float(np_mod.linalg.norm(Xp - O))
            ylen = float(np_mod.linalg.norm(Yp - O))
            if xlen > 0 and ylen > 0:
                self._O = O
                self._Xu = (Xp - O) / xlen
                self._Yu = (Yp - O) / ylen
                self._ok = True

    def contains(self, cx: float, cy: float) -> bool:
        if self.poly is None:
            return True
        return self._cv2.pointPolygonTest(self.poly, (float(cx), float(cy)), False) >= 0

    def to_mm(self, cx: float, cy: float) -> tuple[float, float]:
        """Project (cx, cy) px onto the ROI frame and return (x_mm, y_mm)."""
        if not self._ok:
            return cx / self.pixels_per_mm, cy / self.pixels_per_mm
        v = self._np.array([cx, cy], dtype=float) - self._O
        x_px = float(self._np.dot(v, self._Xu))
        y_px = float(self._np.dot(v, self._Yu))
        return x_px / self.pixels_per_mm, y_px / self.pixels_per_mm

    def mm_to_pixel(self, x_mm: float, y_mm: float) -> tuple[float, float]:
        """Inverse of to_mm: project (x_mm, y_mm) in the ROI frame back to pixels."""
        if not self._ok:
            return x_mm * self.pixels_per_mm, y_mm * self.pixels_per_mm
        px = self._O + x_mm * self.pixels_per_mm * self._Xu + y_mm * self.pixels_per_mm * self._Yu
        return float(px[0]), float(px[1])


def _obb_corners(board: Obb, cv2_mod, np_mod):
    cx, cy, w, h, theta = board[0], board[1], board[2], board[3], board[4]
    return cv2_mod.boxPoints(((cx, cy), (w, h), math.degrees(theta))).astype(np_mod.float32)


def pick_marker(board: Obb, markers: list[Obb], names: dict, marker_map: dict,
                cv2_mod, np_mod, max_dist_px: float | None = None):
    """Pick the marker belonging to `board` and the PCB type it implies.

    A marker whose centre lies INSIDE the board OBB is a definitive association,
    so the closest such marker is returned unconditionally (no distance gate) —
    this is what makes a larger board like TQFP work, whose own marker sits ~22 mm
    from the centre, beyond any fixed `max_dist_px`. Only when no marker lies
    inside the OBB do we fall back to the globally closest marker, gated by
    `max_dist_px` (prevents cross-board association when boards sit close together).
    Returns (marker_tuple_or_None, inferred_type_or_None).
    """
    if not markers:
        return None, None
    bx, by = board[0], board[1]
    corners = _obb_corners(board, cv2_mod, np_mod)
    inside = [m for m in markers
              if cv2_mod.pointPolygonTest(corners, (float(m[0]), float(m[1])), False) >= 0]
    if inside:
        m = min(inside, key=lambda mk: (mk[0] - bx) ** 2 + (mk[1] - by) ** 2)
        return m, marker_map.get(names[m[5]])

    # Fallback: no marker inside the board — take the globally closest, but only
    # if it is near enough to plausibly belong to this board.
    m = min(markers, key=lambda mk: (mk[0] - bx) ** 2 + (mk[1] - by) ** 2)
    if max_dist_px is not None and math.hypot(m[0] - bx, m[1] - by) > max_dist_px:
        return None, None
    return m, marker_map.get(names[m[5]])


def heading_from_marker_vector(board: Obb, marker, offset_deg: float = 0.0) -> float | None:
    """Heading in [0, 360) — angle of the board→marker vector vs downward (0,1),
    clockwise (0°=marker below, 90°=right, 180°=above, 270°=left). None if no marker."""
    if marker is None:
        return None
    dx = marker[0] - board[0]
    dy = marker[1] - board[1]
    return (math.degrees(math.atan2(dx, dy)) + offset_deg) % 360.0


def resolve_heading_360(board: Obb, marker, offset_deg: float, symmetry_deg: float) -> float:
    """Board heading in [0, 360) using the marker to break shape symmetry.
    Falls back to the OBB angle folded into [0, symmetry_deg) when no marker."""
    theta = math.degrees(board[4])
    if marker is None:
        return (theta % symmetry_deg + offset_deg) % 360.0
    to_marker = math.degrees(math.atan2(marker[1] - board[1], marker[0] - board[0]))
    n = max(1, round(360.0 / symmetry_deg))
    best, best_d = theta, 1e9
    for k in range(n):
        cand = theta + k * symmetry_deg
        d = abs(((cand - to_marker + 180.0) % 360.0) - 180.0)
        if d < best_d:
            best_d, best = d, cand
    return (best + offset_deg) % 360.0


# ---------------------------------------------------------------------------
# Belt-speed estimator from object tracking (informational only)
# ---------------------------------------------------------------------------

class BeltVelocityEstimator:
    """Estimate belt speed (mm/s) from tracked-object displacement.

    For every track seen for at least `min_track_frames`, the centroid velocity
    (px/s) is known. We take the median of the belt-axis component across tracks
    (robust to a single mis-tracked box) and EMA-smooth it, then convert px/s →
    mm/s with `pixels_per_mm`.

    NOTE: this is NOT used for operation. The scheduler's belt position/velocity
    still comes from the Siemens `conveyor_position` field. This estimate is for
    cross-checking / future calibration only.
    """

    def __init__(self, pixels_per_mm: float, axis: str = "y",
                 ema_alpha: float = 0.3, min_track_frames: int = 3) -> None:
        self.pixels_per_mm = float(pixels_per_mm)
        self.axis = axis  # 'y' = vertical belt motion, 'x' = horizontal, 'mag' = magnitude
        self.ema_alpha = float(ema_alpha)
        self.min_track_frames = int(min_track_frames)
        self._velocity_mm_per_s = 0.0
        self._n_tracks = 0
        self._initialised = False

    def _component(self, trk: Track) -> float:
        if self.axis == "x":
            return trk.vx
        if self.axis == "mag":
            return math.hypot(trk.vx, trk.vy)
        return trk.vy

    def update(self, active_tracks: dict[int, Track]) -> float:
        comps = [self._component(t) for t in active_tracks.values()
                 if t.frames >= self.min_track_frames]
        self._n_tracks = len(comps)
        if comps:
            inst_px = statistics.median(comps)
            inst_mm = inst_px / self.pixels_per_mm if self.pixels_per_mm > 0 else 0.0
            if not self._initialised:
                self._velocity_mm_per_s = inst_mm
                self._initialised = True
            else:
                self._velocity_mm_per_s = (
                    self.ema_alpha * inst_mm + (1.0 - self.ema_alpha) * self._velocity_mm_per_s
                )
        return self._velocity_mm_per_s

    @property
    def velocity_mm_per_s(self) -> float:
        return self._velocity_mm_per_s

    @property
    def n_tracks(self) -> int:
        return self._n_tracks


# ---------------------------------------------------------------------------
# Real vision pipeline (PyAV capture + YOLO-OBB inference)
# ---------------------------------------------------------------------------

class VisionImageProcessing:
    """Real-time YOLO-OBB detection with a PyAV capture backend.

    Drop-in replacement for SimulatedImageProcessing: same `poll(now)`,
    `stop()`, `render_window()`, `close_window()` interface. All heavy imports
    (av, cv2, numpy, ultralytics) happen in __init__ so simulated scenarios
    never load them.

    Threads:
      * capture  — PyAV decodes frames at the camera's true rate; publishes the
        latest frame. (This replaces the cv2.VideoCapture path that capped fps.)
      * inference — consumes the most recent frame, runs YOLO, tracks, triggers,
        emits ObjectDetection, updates the belt-speed estimate.
      * main (render_window) — owns cv2.imshow/waitKey (Qt requires GUI on main).
    """

    def __init__(self, vision_config: dict[str, Any], start_time: float) -> None:
        import cv2
        import numpy as np

        self._cfg = vision_config
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

        # --- Model -----------------------------------------------------------
        # The model is loaded + warmed up in the inference thread (not here): the
        # 6 s weight load and the ~7 s one-time CUDA warmup would otherwise block
        # the constructor, delaying the live window by ~13 s. Deferring lets the
        # camera window show live video within ~1-2 s while the model loads.
        weights = vision_config.get("model_weights", "models/nano@1280/weights/best.pt")
        if not os.path.isabs(weights):
            weights = os.path.join(project_root, weights)
        self._weights = weights
        self._model = None
        self._names: dict = {}
        self._model_ready = threading.Event()

        self._imgsz = int(vision_config.get("imgsz", 1280))
        self._conf_th = float(vision_config.get("conf", 0.6))
        self._conf_marker = float(vision_config.get("conf_marker", self._conf_th))
        self._iou_th = float(vision_config.get("iou", 0.7))
        self._half = bool(vision_config.get("half", True))
        device = vision_config.get("device", "0")
        if isinstance(device, str) and device.isdigit():
            device = int(device)
        self._device = device if device != "" else None

        # --- ROI / coordinates ----------------------------------------------
        roi_cfg = vision_config.get("roi", {}) or {}
        polygon = roi_cfg.get("polygon") if roi_cfg.get("enabled", False) else None
        self._pixels_per_mm = float(vision_config.get("pixels_per_mm", 4.0))
        self._roi = RoiFrame(polygon, self._pixels_per_mm, np, cv2)

        # --- Trigger line ----------------------------------------------------
        tl = vision_config.get("trigger_line", {}) or {}
        self._trigger = TriggerLine(int(tl.get("y_px", 540)), tl.get("direction", "down"))
        self._tl_min_conf = float(tl.get("min_conf", self._conf_th))

        # --- Orientation -----------------------------------------------------
        ori = vision_config.get("orientation", {}) or {}
        self._ori_enabled = bool(ori.get("enabled", False))
        self._marker_map = ori.get("marker_map", {})
        self._marker_classes = set(self._marker_map.keys())
        self._cross_check = bool(ori.get("cross_check", False))
        # Default PCB classes from the class_map keys (model names aren't loaded
        # yet — the model loads in the inference thread).
        self._pcb_classes = set(ori.get("pcb_classes") or vision_config.get("class_map", {}).keys())
        self._heading_offset = float(ori.get("offset_deg", 0.0))
        self._symmetry_default = float(ori.get("symmetry_deg", 180.0))
        self._symmetry_by_class = ori.get("symmetry_by_class", {})
        # Distance gate for the fallback (no marker inside the board OBB) case.
        self._pixels_per_mm_for_marker = float(vision_config.get("pixels_per_mm", 4.0))
        self._marker_max_dist_px = float(ori.get("marker_max_dist_mm", 30.0)) * self._pixels_per_mm_for_marker

        # --- Tracker + belt estimator ---------------------------------------
        tk = vision_config.get("tracker", {}) or {}
        self._tracker = CentroidTracker(
            max_match_dist=tk.get("max_match_dist_px", 80),
            max_missing=tk.get("max_missing_frames", 15),
        )
        be = vision_config.get("belt_estimator", {}) or {}
        self._belt_estimator_enabled = bool(be.get("enabled", True))
        self._belt_estimator = BeltVelocityEstimator(
            pixels_per_mm=self._pixels_per_mm,
            axis=be.get("axis", "y"),
            ema_alpha=float(be.get("ema_alpha", 0.3)),
            min_track_frames=int(be.get("min_track_frames", 3)),
        )

        # --- Class map (vision class name → scheduler object_type) ----------
        self._class_map = vision_config.get("class_map", {})

        # --- Conveyor transform (pure-Python, no numpy) ----------------------
        from modules.conveyor import M_VISION_TO_CONVEYOR, _mat_apply
        self._vision_to_conveyor = M_VISION_TO_CONVEYOR
        self._mat_apply = _mat_apply

        # --- Camera capture (PyAV) ------------------------------------------
        cap = vision_config.get("capture", {}) or {}
        usb_id = cap.get("camera_usb_id") or vision_config.get("camera_usb_id")
        device_path = cap.get("device")
        if device_path is None and usb_id:
            idx = _find_camera_by_usb_id(usb_id)
            if idx is not None:
                device_path = f"/dev/video{idx}"
                print(f"[VISION] Auto-detected camera USB {usb_id!r} at {device_path}")
        if device_path is None:
            device_path = "/dev/video0"
        self._device_path = device_path
        self._cap_w = int(cap.get("width", 1920))
        self._cap_h = int(cap.get("height", 1080))
        self._cap_fmt = cap.get("pixelformat", "mjpeg")
        self._cap_fps = int(cap.get("fps", 30))

        # Tune v4l2 controls (short exposure so the sensor can sustain the rated
        # fps; stop auto-exposure dynamic framerate) before opening the stream.
        controls = vision_config.get("v4l2_controls")
        if controls is None:
            controls = {"exposure_dynamic_framerate": 0,
                        "auto_exposure": 1, "exposure_time_absolute": 150}
        _apply_v4l2_controls(device_path, controls)

        self._cv2 = cv2
        self._np = np
        self._counter = 0
        self._show_window = bool(vision_config.get("show_window", True))
        # The web dashboard streams the annotated frame over MJPEG. When attached
        # it flips this on so the inference thread draws the overlay even if the
        # native cv2 window is disabled (`show_window=false`). JPEG quality for
        # the MJPEG encode is read from the vision config.
        self._web_overlay = False
        self._jpeg_quality = int(vision_config.get("mjpeg_jpeg_quality", 80))

        # FPS readouts.
        self._cam_fps = 0.0
        self._proc_fps = 0.0

        # Track ids already emitted at least once, so we log "NEW" only on first
        # sighting (emission is now continuous, every frame, not one-shot).
        self._emitted_ids: set[int] = set()

        # Thread-safe detection queue.
        self._deque: collections.deque[ObjectDetection] = collections.deque()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # Latest captured frame, published by the capture thread.
        self._latest_frame = None
        self._latest_frame_id = 0
        self._frame_lock = threading.Lock()

        # Annotated frame for the main-thread GUI.
        self._display_frame = None
        self._display_lock = threading.Lock()

        # Open the PyAV container before starting threads so open errors surface
        # in the constructor (consistent with the old behaviour).
        self._container = self._open_container()

        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True, name="VisionCapture")
        self._capture_thread.start()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="VisionThread")
        self._thread.start()
        print(f"[VISION] Pipeline started (dev={device_path}, {self._cap_w}x{self._cap_h}@{self._cap_fps} "
              f"{self._cap_fmt}, weights={os.path.basename(weights)}, imgsz={self._imgsz}); "
              "model loading in background…")

    def _open_container(self):
        import av
        options = {
            "input_format": self._cap_fmt,
            "video_size": f"{self._cap_w}x{self._cap_h}",
            "framerate": str(self._cap_fps),
        }
        try:
            container = av.open(self._device_path, format="v4l2", options=options)
        except Exception as exc:
            raise RuntimeError(
                f"VisionImageProcessing: cannot open camera {self._device_path!r} via PyAV: {exc}"
            )
        container.streams.video[0].thread_type = "AUTO"
        return container

    def _capture_loop(self) -> None:
        """Decode frames at the camera's native rate and publish the latest."""
        np = self._np
        alpha = 0.3
        last_t: float | None = None
        try:
            stream = self._container.streams.video[0]
            for frame in self._container.decode(stream):
                if self._stop_event.is_set():
                    break
                img = frame.to_ndarray(format="bgr24")
                t = time.monotonic()
                if last_t is not None:
                    dt = t - last_t
                    if dt > 0.0:
                        self._cam_fps = alpha * (1.0 / dt) + (1.0 - alpha) * self._cam_fps
                last_t = t
                with self._frame_lock:
                    self._latest_frame = img
                    self._latest_frame_id += 1
        except Exception as exc:
            print(f"[VISION] Capture error: {exc}")
        finally:
            self._stop_event.set()
            try:
                self._container.close()
            except Exception:
                pass
            print("[VISION] Capture thread stopped.")

    def _loop(self) -> None:
        """Inference loop — loads the model (off the constructor's critical path),
        warms it up, then consumes the most recent frame and runs YOLO."""
        cv2 = self._cv2
        np = self._np
        alpha = 0.3
        last_id = -1

        # Load + fuse + warm up here so the constructor returns immediately and
        # the live camera window can appear while this happens.
        try:
            from ultralytics import YOLO
            t_load = time.monotonic()
            self._model = YOLO(self._weights)
            try:
                self._model.fuse()
            except Exception:
                pass
            self._names = self._model.names
            # One-time CUDA warmup (the first predict is ~7 s at imgsz 1920);
            # doing it on a dummy frame keeps the first live frame responsive.
            dummy = np.zeros((self._cap_h, self._cap_w, 3), dtype=np.uint8)
            self._model.predict(dummy, imgsz=self._imgsz, device=self._device,
                                half=self._half, verbose=False)
            self._model_ready.set()
            print(f"[VISION] Model ready ({time.monotonic() - t_load:.1f}s load+warmup).")
        except Exception as exc:
            print(f"[VISION] Model load failed: {exc}")
            self._stop_event.set()
            return

        try:
            while not self._stop_event.is_set():
                with self._frame_lock:
                    frame = self._latest_frame
                    fid = self._latest_frame_id
                if frame is None or fid == last_id:
                    time.sleep(0.001)
                    continue
                last_id = fid
                frame = frame.copy()

                t0 = time.monotonic()
                result = self._model.predict(
                    frame, conf=self._conf_marker, iou=self._iou_th,
                    imgsz=self._imgsz, device=self._device, half=self._half,
                    verbose=False,
                )[0]

                dets = [d for d in extract_obb(result, np) if self._roi.contains(d[0], d[1])]
                pcb_dets = [d for d in dets
                            if self._names[d[5]] in self._pcb_classes and d[6] >= self._conf_th]
                marker_dets = ([d for d in dets if self._names[d[5]] in self._marker_classes]
                               if self._ori_enabled else [])

                now = time.monotonic()
                centroids = [(d[0], d[1]) for d in pcb_dets]
                active = self._tracker.update(centroids, now)

                if self._belt_estimator_enabled:
                    self._belt_estimator.update(active)

                self._emit_detections(active, pcb_dets, marker_dets)

                dt_proc = time.monotonic() - t0
                if dt_proc > 0.0:
                    self._proc_fps = alpha * (1.0 / dt_proc) + (1.0 - alpha) * self._proc_fps

                if self._show_window or self._web_overlay:
                    self._draw_overlay(frame, pcb_dets, marker_dets, active)
                    with self._display_lock:
                        self._display_frame = frame
        except Exception as exc:
            print(f"[VISION] Inference error: {exc}")
        finally:
            self._stop_event.set()
            print("[VISION] Inference thread stopped.")

    def _compute_angle(self, board, marker_dets):
        """Return (angle_deg, type_name, marker_or_None) for a board.

        Orientation logic shared by the emit path and the live overlay. With
        orientation enabled the angle is the board→marker heading in [0,360);
        otherwise the OBB angle folded into [-90,90).
        """
        cls_id = board[5]
        if self._ori_enabled:
            marker, inferred = pick_marker(
                board, marker_dets, self._names, self._marker_map,
                self._cv2, self._np, max_dist_px=self._marker_max_dist_px,
            )
            type_name = inferred if (self._cross_check and inferred) else self._names[cls_id]
            angle = heading_from_marker_vector(board, marker, self._heading_offset)
            if angle is None:
                sym = float(self._symmetry_by_class.get(type_name, self._symmetry_default))
                angle = resolve_heading_360(board, None, self._heading_offset, sym)
            return angle, type_name, marker
        return normalize_angle_deg(board[4]), self._names[cls_id], None

    def _emit_detections(self, active, pcb_dets, marker_dets) -> None:
        """Emit an ObjectDetection for every tracked board, every frame it is seen.

        Replaces the old one-shot trigger-line crossing. A board is created/updated
        as soon as it is detected — but only when it has a full OBB *and* a matched
        marker (orientation enabled): the marker is what resolves the 360° heading,
        so without it we neither create the object nor update its angle. The id
        (`yolo-{trk.id}`) is stable across frames (see CentroidTracker), so the
        scheduler re-anchors the same object from the camera while it is visible
        and dead-reckons from belt position once it leaves the camera zone.
        """
        for trk in active.values():
            if not pcb_dets:
                continue
            board = min(pcb_dets, key=lambda d: (d[0] - trk.cx) ** 2 + (d[1] - trk.cy) ** 2)
            cx, cy, w, h, theta, cls_id, conf = board
            if conf < self._tl_min_conf:
                continue

            angle, type_name, marker = self._compute_angle(board, marker_dets)
            # Full OBB + marker gate: only create/update when the marker is present.
            if self._ori_enabled and marker is None:
                continue

            mapped = self._class_map.get(type_name)
            if mapped is None:
                if trk.id not in self._emitted_ids:
                    print(f"[VISION] Unknown class '{type_name}' — skipping (not in class_map)")
                continue

            x_mm, y_mm = self._roi.to_mm(cx, cy)
            u, v = self._mat_apply(self._vision_to_conveyor, x_mm, y_mm)
            det = ObjectDetection(
                object_id=f"yolo-{trk.id:06d}",
                x=u, y=v, object_type=mapped,
                timestamp=time.monotonic(),
                confidence=float(conf), angle_deg=float(angle),
            )
            with self._lock:
                self._deque.append(det)

            # Log only on first sighting of an id to avoid per-frame spam.
            if trk.id not in self._emitted_ids:
                self._emitted_ids.add(trk.id)
                self._counter += 1
                belt = (f" belt~{self._belt_estimator.velocity_mm_per_s:.1f}mm/s"
                        if self._belt_estimator_enabled else "")
                print(f"[VISION] NEW id={trk.id} type={mapped} angle={angle:.0f} "
                      f"x_mm={x_mm:.1f} y_mm={y_mm:.1f} u={u:.1f} v={v:.1f}{belt}",
                      flush=True)

    def _draw_overlay(self, frame, pcb_dets, marker_dets, active) -> None:
        """Slim overlay: ROI axes, trigger line, boxes + id/type/angle/coords, FPS, belt est."""
        cv2 = self._cv2
        np = self._np
        h, w = frame.shape[:2]

        if self._roi.poly is not None:
            cv2.polylines(frame, [self._roi.poly], True, (0, 0, 255), 2)
            # Draw coordinate axes: O=poly[3] (BL), X+=poly[2] (BR), Y+=poly[0] (TL).
            pts = self._roi.poly
            O  = tuple(pts[3].tolist())
            Xp = tuple(pts[2].tolist())
            Yp = tuple(pts[0].tolist())
            cv2.arrowedLine(frame, O, Xp, (255, 80, 0), 2, tipLength=0.04)
            mx = ((O[0] + Xp[0]) // 2, (O[1] + Xp[1]) // 2 + 20)
            cv2.putText(frame, "X", mx, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 80, 0), 2)
            cv2.arrowedLine(frame, O, Yp, (0, 200, 0), 2, tipLength=0.04)
            my = (O[0] - 28, (O[1] + Yp[1]) // 2)
            cv2.putText(frame, "Y", my, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
            cv2.circle(frame, O, 6, (0, 255, 255), -1)
            cv2.putText(frame, "O", (O[0] + 8, O[1] + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        # Trigger line removed: detections are now created on first full OBB+marker
        # sighting (see _emit_detections), not on a line crossing.

        for m in marker_dets:
            box = cv2.boxPoints(((m[0], m[1]), (m[2], m[3]), math.degrees(m[4]))).astype(int)
            cv2.polylines(frame, [box], True, (0, 255, 255), 1)

        for board in pcb_dets:
            cx, cy, bw, bh, theta, cls_id, conf = board
            box = cv2.boxPoints(((cx, cy), (bw, bh), math.degrees(theta))).astype(int)
            cv2.polylines(frame, [box], True, (0, 255, 0), 2)
            cv2.circle(frame, (int(cx), int(cy)), 4, (255, 0, 0), -1)
            tid = min(active, key=lambda i: (active[i].cx - cx) ** 2 + (active[i].cy - cy) ** 2,
                      default=None) if active else None

            # Live angle + type (same logic as emit). If a marker is matched, draw
            # the board→marker vector that defines the 360° heading.
            angle, type_name, marker = self._compute_angle(board, marker_dets)
            label = f"{type_name} {conf:.2f} {angle:.0f}deg"
            if tid is not None:
                label = f"#{tid} " + label
            cv2.putText(frame, label, (int(cx) - 40, int(cy) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if marker is not None:
                cv2.arrowedLine(frame, (int(cx), int(cy)), (int(marker[0]), int(marker[1])),
                                (255, 0, 255), 2, tipLength=0.2)

            # Position in camera frame coordinates.
            if self._roi._ok:
                x_mm, y_mm = self._roi.to_mm(cx, cy)
                coord_label = f"X:{x_mm:.1f} Y:{y_mm:.1f} mm"
            else:
                coord_label = f"px:{int(cx)} py:{int(cy)}"
            cv2.putText(frame, coord_label, (int(cx) - 40, int(cy) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 2)

        cv2.putText(frame, f"CAM {self._cam_fps:.1f} FPS", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"PROC {self._proc_fps:.1f} FPS", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if self._belt_estimator_enabled:
            cv2.putText(frame, f"BELT~ {self._belt_estimator.velocity_mm_per_s:.0f} mm/s "
                               f"(est, n={self._belt_estimator.n_tracks})",
                        (10, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

    def poll(self, now: float) -> list[ObjectDetection]:  # noqa: ARG002
        with self._lock:
            detections = list(self._deque)
            self._deque.clear()
        return detections

    @property
    def belt_velocity_mm_per_s(self) -> float:
        """Belt-speed estimate from tracking (informational; not for operation)."""
        return self._belt_estimator.velocity_mm_per_s

    def enable_web_overlay(self) -> None:
        """Ask the inference thread to keep producing an annotated frame for the
        web MJPEG stream, independent of the native cv2 window."""
        self._web_overlay = True

    def jpeg_frame(self) -> bytes | None:
        """Return the latest annotated frame JPEG-encoded for MJPEG streaming.

        Falls back to the raw captured frame (with a 'loading model...' hint)
        while the model is still warming up, so the browser shows live video
        immediately. Returns None if no frame is available yet.
        """
        cv2 = self._cv2
        with self._display_lock:
            frame = self._display_frame
        if frame is None:
            with self._frame_lock:
                raw = self._latest_frame
            if raw is None:
                return None
            frame = raw.copy()
            if not self._model_ready.is_set():
                cv2.putText(frame, "loading model...", (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2)
        ok, buf = cv2.imencode(".jpg", frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if not ok:
            return None
        return buf.tobytes()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=5.0)
        self._capture_thread.join(timeout=5.0)

    def render_window(self) -> bool:
        """Pump the GUI from the MAIN thread (Qt requires this). Returns False
        once the window should close (user pressed 'q' or a thread stopped).

        Shows the annotated frame when available; otherwise falls back to the
        latest raw captured frame so the window appears as soon as frames flow
        (no need to wait for the model to load + the first inference)."""
        if not self._show_window:
            return not self._stop_event.is_set()
        cv2 = self._cv2
        with self._display_lock:
            frame = self._display_frame
        if frame is None and not self._model_ready.is_set():
            # Model still loading — show live video with a hint.
            with self._frame_lock:
                raw = self._latest_frame
            if raw is not None:
                frame = raw.copy()
                cv2.putText(frame, "loading model...", (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2)
        if frame is not None:
            cv2.imshow("Delta Vision", frame)
            # Raise the window to front on first frame so it's not hidden behind
            # the terminal. Only once (flag cleared after first raise attempt).
            if getattr(self, "_need_raise", True):
                self._need_raise = False
                def _raise_vision():
                    time.sleep(0.4)
                    try:
                        subprocess.run(["wmctrl", "-a", "Delta Vision"],
                                       check=False, capture_output=True, timeout=2.0)
                    except Exception:
                        pass
                threading.Thread(target=_raise_vision, daemon=True).start()
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self._stop_event.set()
        return not self._stop_event.is_set()

    def close_window(self) -> None:
        if self._show_window:
            try:
                self._cv2.destroyAllWindows()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Smoke-test entrypoint: python3 -m modules.image_processing [--duration N]
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    from modules.EthernetCom import load_config

    ap = argparse.ArgumentParser(
        description="VisionImageProcessing smoke test — runs YOLO and shows the camera window"
    )
    ap.add_argument("--duration", type=float, default=None,
                    help="Run time in seconds. Omit to run until 'q' or Ctrl-C.")
    ap.add_argument("--no-window", action="store_true", help="Run headless (no camera window)")
    args = ap.parse_args()

    cfg = load_config()
    vision_cfg = dict(getattr(cfg, "vision", {}) or {})
    if args.no_window:
        vision_cfg["show_window"] = False

    run_for = "until q/Ctrl-C" if args.duration is None else f"{args.duration:.0f}s"
    print(f"[SMOKE] Starting VisionImageProcessing ({run_for}, "
          f"window {'off' if args.no_window else 'on — press q to quit'}) ...")
    vip = VisionImageProcessing(vision_cfg, time.monotonic())
    deadline = None if args.duration is None else time.monotonic() + args.duration
    try:
        while not vip._stop_event.is_set():
            if deadline is not None and time.monotonic() >= deadline:
                break
            for d in vip.poll(time.monotonic()):
                print(f"[DETECTION] {d.to_dict()}")
            if not vip.render_window():
                break
            if args.no_window:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        vip.stop()
        vip.close_window()
    print("[SMOKE] Done.")
