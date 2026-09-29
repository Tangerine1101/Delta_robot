"""Realtime YOLO-OBB vision pipeline (doc/basis-programming.md §2).

Frames are captured with **PyAV** (FFmpeg-backed) in a dedicated thread so the camera runs at
its true rate (~30 fps at 1080p MJPG); `cv2.VideoCapture`'s V4L2 backend was the <20 fps
bottleneck, not the model or the GPU. Inference runs in a second thread; the OpenCV GUI is
pumped from the main thread (Qt requirement). Detections are emitted in belt coordinates
(u, v) through `modules.core.frames.vision_mm_to_uv`.

A belt-speed estimate from tracking (`BeltVelocityEstimator`) is informational only: the
scheduler's belt position and speed come from the Siemens `conveyor_position` field.

    python3 -m modules.vision.pipeline [--duration N] [--no-window]     # smoke test
"""
from __future__ import annotations

import collections
import math
import os
import subprocess
import threading
import time
from typing import Any

from modules.core.frames import vision_mm_to_uv
from modules.core.tracking import ObjectDetection
from modules.settings import Settings
from modules.vision.camera import apply_v4l2_controls, find_camera_by_usb_id
from modules.vision.heading import (
    extract_obb,
    heading_from_marker_vector,
    normalize_angle_deg,
    pick_marker,
    resolve_heading_360,
)
from modules.vision.roi import RoiFrame, TriggerLine
from modules.vision.tracking import BeltVelocityEstimator, CentroidTracker

# OpenCV's bundled Qt has no Wayland plugin; force xcb (X11/XWayland) so the live window
# actually maps. Override unconditionally — the desktop may export wayland.
os.environ["QT_QPA_PLATFORM"] = "xcb"

# Stop ultralytics from auto-pip-installing optional deps (e.g. albumentations) at predict
# time — that pulls in `opencv-python-headless`, which has NO GUI and silently breaks
# `cv2.imshow` (the live window). Keep the GUI `opencv-python`.
os.environ.setdefault("YOLO_AUTOINSTALL", "false")


class VisionImageProcessing:
    """Real-time YOLO-OBB detection with a PyAV capture backend.

    Exposes the image-source interface of the realtime loop: `poll(now)`, `stop()`,
    `render_window()`, `close_window()` (the PLC simulator's camera exposes the same). All
    heavy imports (av, cv2, numpy, ultralytics) happen in __init__, so runs without the real
    camera never load them.

    Threads:
      * capture  — PyAV decodes frames at the camera's true rate; publishes the
        latest frame. (This replaces the cv2.VideoCapture path that capped fps.)
      * inference — consumes the most recent frame, runs YOLO, tracks, triggers,
        emits ObjectDetection, updates the belt-speed estimate.
      * main (render_window) — owns cv2.imshow/waitKey (Qt requires GUI on main).
    """

    def __init__(self, settings: Settings, start_time: float, *, show_window: bool | None = None) -> None:
        import cv2
        import numpy as np

        vision = settings.vision
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

        # --- Model -----------------------------------------------------------
        # The model is loaded + warmed up in the inference thread (not here): the
        # 6 s weight load and the ~7 s one-time CUDA warmup would otherwise block
        # the constructor, delaying the live window by ~13 s. Deferring lets the
        # camera window show live video within ~1-2 s while the model loads.
        weights = vision.model_weights
        if not os.path.isabs(weights):
            weights = os.path.join(project_root, weights)
        self._weights = weights
        self._model = None
        self._names: dict = {}
        self._model_ready = threading.Event()

        self._imgsz = vision.imgsz
        self._conf_th = vision.conf
        self._conf_marker = vision.conf_marker if vision.conf_marker is not None else self._conf_th
        self._iou_th = vision.iou
        self._half = vision.half
        device: Any = vision.device
        if isinstance(device, str) and device.isdigit():
            device = int(device)
        self._device = device if device != "" else None

        # --- ROI / coordinates ----------------------------------------------
        polygon = [list(point) for point in vision.roi.polygon] if vision.roi.enabled else None
        self._pixels_per_mm = vision.pixels_per_mm
        self._roi = RoiFrame(polygon, self._pixels_per_mm, np, cv2)

        # --- Trigger line ----------------------------------------------------
        tl = vision.trigger_line
        self._trigger = TriggerLine(tl.y_px, tl.direction)
        self._tl_min_conf = tl.min_conf if tl.min_conf is not None else self._conf_th

        # --- Orientation and per-class data (object_types) -------------------
        ori = vision.orientation
        self._ori_enabled = ori.enabled
        self._cross_check = ori.cross_check
        # YOLO class name -> object type, and the per-class orientation data keyed by the
        # YOLO class name the model reports.
        self._class_map = {settings.model_class(name): name for name in settings.object_types}
        self._marker_map = {spec.marker_class: settings.model_class(name)
                            for name, spec in settings.object_types.items() if spec.marker_class}
        self._marker_classes = set(self._marker_map.keys())
        self._pcb_classes = set(self._class_map.keys())
        # Per-class heading offset (degrees): the board->marker vector includes the marker's
        # diagonal placement on the board (a corner marker on a square QFP sits ~45 deg off
        # the edges), and that constant differs per board type (open-issues C4).
        self._heading_offset = 0.0
        self._heading_offset_by_class = {settings.model_class(name): spec.heading_offset_deg
                                         for name, spec in settings.object_types.items()}
        self._symmetry_default = 180.0
        self._symmetry_by_class = {settings.model_class(name): spec.symmetry_deg
                                   for name, spec in settings.object_types.items()}
        # Distance gate for the fallback (no marker inside the board OBB) case.
        self._marker_max_dist_px = ori.marker_max_dist_mm * self._pixels_per_mm

        # --- Tracker + belt estimator ---------------------------------------
        self._tracker = CentroidTracker(
            max_match_dist=vision.tracker.max_match_dist_px,
            max_missing=vision.tracker.max_missing_frames,
        )
        be = vision.belt_estimator
        self._belt_estimator_enabled = be.enabled
        self._belt_estimator = BeltVelocityEstimator(
            pixels_per_mm=self._pixels_per_mm,
            axis=be.axis,
            ema_alpha=be.ema_alpha,
            min_track_frames=be.min_track_frames,
        )

        # --- Camera capture (PyAV) ------------------------------------------
        cap = vision.capture
        usb_id = cap.camera_usb_id
        device_path = cap.device
        if device_path is None and usb_id:
            idx = find_camera_by_usb_id(usb_id)
            if idx is not None:
                device_path = f"/dev/video{idx}"
                print(f"[VISION] Auto-detected camera USB {usb_id!r} at {device_path}")
        if device_path is None:
            device_path = "/dev/video0"
        self._device_path = device_path
        self._cap_w = cap.width
        self._cap_h = cap.height
        self._cap_fmt = cap.pixelformat
        self._cap_fps = cap.fps

        # Tune v4l2 controls (short exposure so the sensor can sustain the rated
        # fps; stop auto-exposure dynamic framerate) before opening the stream.
        controls = vision.v4l2_controls
        if controls is None:
            controls = {"exposure_dynamic_framerate": 0,
                        "auto_exposure": 1, "exposure_time_absolute": 150}
        apply_v4l2_controls(device_path, controls)

        # Camera-latency compensation: a detection's true capture instant is the
        # MIDDLE of the exposure window, but the frame is only handed over after
        # the exposure completes. V4L2 exposure_time_absolute is in units of
        # 100 µs, so half the exposure (seconds) backdates the emitted timestamp
        # toward the true capture time. The rest of the latency (decode + YOLO +
        # poll) is absorbed downstream by anchoring to the belt position AT this
        # backdated timestamp (see BeltPositionTracker.position_at).
        try:
            exposure_units = float(controls.get("exposure_time_absolute", 0) or 0)
        except (TypeError, ValueError):
            exposure_units = 0.0
        self._half_exposure_s = exposure_units * 100e-6 / 2.0

        self._cv2 = cv2
        self._np = np
        self._counter = 0
        self._show_window = vision.show_window if show_window is None else show_window
        # The web dashboard streams the annotated frame over MJPEG. When attached
        # it flips this on so the inference thread draws the overlay even if the
        # native cv2 window is disabled (`show_window=false`). JPEG quality for
        # the MJPEG encode is read from the vision config.
        self._web_overlay = False
        self._jpeg_quality = vision.mjpeg_jpeg_quality

        # FPS readouts.
        self._cam_fps = 0.0
        self._proc_fps = 0.0

        # Track ids already emitted at least once, so we log "NEW" only on first
        # sighting (emission is now continuous, every frame, not one-shot).
        self._emitted_ids: set[int] = set()

        # Last marker-vector heading resolved per track id. A board's marker can
        # drop out of detection near the ROI edge (crop/occlusion) even though
        # the board itself is still tracked; reusing the last good marker angle
        # avoids falling back to the OBB symmetry fold (which can jump ~90/180
        # deg and looked like random post-pick misorientation downstream).
        self._marker_angle_by_track: dict[int, float] = {}

        # Thread-safe detection queue.
        self._deque: collections.deque[ObjectDetection] = collections.deque()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # Latest captured frame, published by the capture thread.
        self._latest_frame = None
        self._latest_frame_id = 0
        self._latest_frame_ts = 0.0   # monotonic time the frame was decoded
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
                    self._latest_frame_ts = t
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
                    frame_ts = self._latest_frame_ts
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
                self._marker_angle_by_track = {
                    tid: angle for tid, angle in self._marker_angle_by_track.items()
                    if tid in active
                }

                if self._belt_estimator_enabled:
                    self._belt_estimator.update(active)

                self._emit_detections(active, pcb_dets, marker_dets, frame_ts)

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
            offset = self._heading_offset_by_class.get(type_name, self._heading_offset)
            angle = heading_from_marker_vector(board, marker, offset)
            if angle is None:
                sym = float(self._symmetry_by_class.get(type_name, self._symmetry_default))
                angle = resolve_heading_360(board, None, offset, sym)
            return angle, type_name, marker
        return normalize_angle_deg(board[4]), self._names[cls_id], None

    def _emit_detections(self, active, pcb_dets, marker_dets, capture_ts=None) -> None:
        """Emit an ObjectDetection for every tracked board, every frame it is seen.

        Replaces the old one-shot trigger-line crossing. A board is created/updated
        as soon as it has a full OBB (orientation enabled or not). With orientation
        enabled the heading comes from the marker vector when a marker is matched
        this frame; if the marker is missed for a frame (e.g. cropped/occluded near
        the ROI edge) the track's last marker-resolved heading is reused instead of
        the OBB symmetry-fold fallback (see `_marker_angle_by_track`); only a track
        that has never once resolved a marker uses the OBB fallback angle. The id
        (`yolo-{trk.id}`) is stable across frames (see CentroidTracker), so the
        scheduler re-anchors the same object from the camera while it is visible
        and dead-reckons from belt position once it leaves the camera zone.

        `capture_ts` is the monotonic time the frame was decoded; the emitted
        timestamp is backdated by half the exposure so the scheduler can anchor
        the object to the belt position at its true capture instant.
        """
        detect_ts = (
            (capture_ts - self._half_exposure_s)
            if capture_ts is not None else time.monotonic()
        )
        for trk in active.values():
            if not pcb_dets:
                continue
            board = min(pcb_dets, key=lambda d: (d[0] - trk.cx) ** 2 + (d[1] - trk.cy) ** 2)
            cx, cy, w, h, theta, cls_id, conf = board
            if conf < self._tl_min_conf:
                continue

            angle, type_name, marker = self._compute_angle(board, marker_dets)
            if marker is not None:
                self._marker_angle_by_track[trk.id] = angle
            elif trk.id in self._marker_angle_by_track:
                # Marker missed this frame only (e.g. board exiting the ROI) —
                # reuse the last marker-resolved heading instead of the OBB
                # symmetry-fold fallback _compute_angle already computed.
                angle = self._marker_angle_by_track[trk.id]
            # else: this track has never resolved a marker — keep the OBB
            # fallback angle from _compute_angle (best available, still emitted
            # so the object isn't silently dropped).

            mapped = self._class_map.get(type_name)
            if mapped is None:
                if trk.id not in self._emitted_ids:
                    print(f"[VISION] Unknown class '{type_name}' — skipping (not in class_map)")
                continue

            x_mm, y_mm = self._roi.to_mm(cx, cy)
            u, v = vision_mm_to_uv(x_mm, y_mm)
            det = ObjectDetection(
                object_id=f"yolo-{trk.id:06d}",
                x=u, y=v, object_type=mapped,
                timestamp=detect_ts,
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


if __name__ == "__main__":
    import argparse

    from modules.settings import load_settings

    ap = argparse.ArgumentParser(
        description="VisionImageProcessing smoke test — runs YOLO and shows the camera window"
    )
    ap.add_argument("--duration", type=float, default=None,
                    help="Run time in seconds. Omit to run until 'q' or Ctrl-C.")
    ap.add_argument("--no-window", action="store_true", help="Run headless (no camera window)")
    args = ap.parse_args()

    run_for = "until q/Ctrl-C" if args.duration is None else f"{args.duration:.0f}s"
    print(f"[SMOKE] Starting VisionImageProcessing ({run_for}, "
          f"window {'off' if args.no_window else 'on — press q to quit'}) ...")
    vip = VisionImageProcessing(load_settings(), time.monotonic(),
                                show_window=False if args.no_window else None)
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
