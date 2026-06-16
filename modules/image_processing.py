"""
Image processing module.

- `SimulatedImageProcessing` — deterministic fake stream for test_accuracy /
  test_throughput (no heavy dependencies).
- `VisionImageProcessing` — real pipeline backed by the YOLO_OBB repo
  (ultralytics + OpenCV). Runs the vision loop in a background daemon thread;
  thread-safe `poll()` drains detections since the last call. Detections are
  emitted in C-frame (u, v) via M_VISION_TO_CONVEYOR in conveyor.py.
  Calibrate M_VISION_TO_CONVEYOR after physical rig setup.
"""
from __future__ import annotations

import collections
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

# OpenCV's bundled Qt has no Wayland plugin; force xcb (X11/XWayland) so the live
# window actually maps. Respects an explicit override if the user already set it.
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")


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


def _apply_v4l2_controls(device_index: int, controls: dict) -> None:
    """Best-effort apply v4l2 controls to /dev/video{N} via `v4l2-ctl`.

    The key default control is `exposure_dynamic_framerate=0`: UVC webcams
    otherwise let auto-exposure drop the frame rate well below the rated FPS in
    dim light (e.g. 30 → ~18). No-op off Linux or when `v4l2-ctl` is absent.
    """
    if not controls or sys.platform != "linux":
        return
    import shutil
    import subprocess
    if shutil.which("v4l2-ctl") is None:
        print("[VISION] v4l2-ctl not found — skipping camera control tuning "
              "(install v4l-utils for full FPS).")
        return
    dev = f"/dev/video{device_index}"
    for name, value in controls.items():
        try:
            subprocess.run(
                ["v4l2-ctl", "--device", dev, f"--set-ctrl={name}={value}"],
                check=False, capture_output=True, timeout=2.0,
            )
        except Exception as exc:
            print(f"[VISION] Could not set {name}={value} on {dev}: {exc}")


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


class SimulatedImageProcessing:
    """Deterministic fake object stream for scheduler development.

    Phase 1: emits detections at C-frame `(u, v)` coordinates.
    - `throughput_spawn_y` is reused as the upstream `u_spawn`.
    - `throughput_lanes` is reused as the per-object `v_lane` values.
    - `accuracy_spawn_uv`: list of [u, v] points inside workspace_window_uv used
      by test_accuracy. Separate from `accuracy_points` (which are robot-frame XYZ
      targets used only by the evaluate scenario).
    """

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

    def poll(self, now: float) -> list[ObjectDetection]:
        detections: list[ObjectDetection] = []
        interval = self._scenario_interval()
        while now >= self.next_emit_at:
            detections.append(self._build_detection(self.next_emit_at))
            self.next_emit_at += interval
        return detections

    def _scenario_interval(self) -> float:
        if self.scenario_name == "test_accuracy":
            return self.accuracy_emit_interval_s
        return self.throughput_emit_interval_s

    def _build_detection(self, timestamp: float) -> ObjectDetection:
        self.counter += 1
        if self.scenario_name == "test_accuracy":
            u, v = self.accuracy_spawn_uv[(self.counter - 1) % len(self.accuracy_spawn_uv)]
            object_type = self.throughput_types[(self.counter - 1) % len(self.throughput_types)]
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


# ---------------------------------------------------------------------------
# Real vision pipeline backed by the YOLO_OBB repo
# ---------------------------------------------------------------------------

class VisionImageProcessing:
    """Real-time YOLO26-OBB detection running in a background thread.

    Drop-in replacement for SimulatedImageProcessing with the same `poll(now)`
    interface. Imports ultralytics/cv2/numpy lazily so simulated scenarios
    never need those packages.

    Detection coordinates are transformed from the vision ROI frame to C-frame
    (u, v) using `modules.conveyor.M_VISION_TO_CONVEYOR`. Calibrate that
    matrix after the physical rig is assembled.
    """

    def __init__(self, vision_config: dict[str, Any], start_time: float) -> None:
        # Heavy imports inside __init__ — not loaded by simulated scenarios.
        import yaml
        import cv2
        import numpy as np
        from ultralytics import YOLO

        # Locate the YOLO_OBB repo directory.
        yolo_dir = vision_config.get("yolo_dir", "YOLO_OBB")
        if not os.path.isabs(yolo_dir):
            # Resolve relative to the project root (one level above modules/).
            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            yolo_dir = os.path.join(project_root, yolo_dir)

        sys_cfg_path = vision_config.get("system_config", os.path.join(yolo_dir, "config", "system_config.yaml"))
        if not os.path.isabs(sys_cfg_path):
            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            sys_cfg_path = os.path.join(project_root, sys_cfg_path)

        with open(sys_cfg_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        # Allow config.json overrides.
        camera_source_override = vision_config.get("camera_source")
        min_conf_override = vision_config.get("min_conf")

        model_cfg = cfg["model"]
        weights = model_cfg["weights"]
        if not os.path.isabs(weights):
            weights = os.path.join(yolo_dir, weights)

        self._model = YOLO(weights)
        self._names = self._model.names
        conf_th = float(min_conf_override or model_cfg.get("conf", 0.6))
        conf_marker = float(model_cfg.get("conf_marker", conf_th))
        self._conf_th = conf_th
        self._conf_marker = conf_marker
        self._iou_th = float(model_cfg.get("iou", 0.7))
        self._imgsz = int(model_cfg.get("imgsz", 640))
        self._device = model_cfg.get("device", "") or None

        # Undistort
        u_cfg = cfg.get("undistort", {})
        if u_cfg.get("enabled", False):
            self._K = np.array(u_cfg["camera_matrix"], dtype=np.float64)
            self._D = np.array(u_cfg["dist_coeffs"], dtype=np.float64)
        else:
            self._K = self._D = None

        # ROI polygon
        roi = cfg.get("roi", {})
        if roi.get("enabled", False) and roi.get("polygon"):
            self._roi_poly = np.array(roi["polygon"], dtype=np.int32)
        else:
            self._roi_poly = None

        self._pixels_per_mm = float(cfg["coordinate"].get("pixels_per_mm", 4.0))

        # Trigger line
        tl = cfg["trigger_line"]
        self._tl_y_px = int(tl["y_px"])
        self._tl_direction = tl.get("direction", "down")
        self._tl_min_conf = float(min_conf_override or tl.get("min_conf", conf_th))

        # Orientation config
        ori = cfg.get("orientation", {})
        self._ori_enabled = bool(ori.get("enabled", False))
        self._marker_map = ori.get("marker_map", {})
        self._marker_classes = set(self._marker_map.keys())
        self._cross_check = bool(ori.get("cross_check", True))
        self._pcb_classes: set = set(ori.get("pcb_classes", list(self._names.values())))
        self._heading_offset = float(ori.get("offset_deg", 0.0))
        self._symmetry_default = float(ori.get("symmetry_deg", 180.0))
        self._symmetry_by_class: dict = ori.get("symmetry_by_class", {})

        # Tracker
        tk = cfg.get("tracker", {})

        # Camera open — prefer USB-ID auto-detect, then explicit override, then yaml default.
        cam = cfg["camera"]
        usb_id = vision_config.get("camera_usb_id")
        if camera_source_override is not None:
            src = camera_source_override
        elif usb_id:
            src = _find_camera_by_usb_id(usb_id)
            if src is None:
                print(f"[VISION] Camera USB ID {usb_id!r} not found — falling back to config source")
                src = cam["source"]
            else:
                print(f"[VISION] Auto-detected camera USB {usb_id!r} at /dev/video{src}")
        else:
            src = cam["source"]
        self._cap = cv2.VideoCapture(src)
        if isinstance(src, int):
            # MJPG first — USB UVC webcams only reach 30 fps in MJPG; the default
            # uncompressed (YUYV) mode is bandwidth-limited to ~10 fps at 720p.
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam.get("width", 1280))
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam.get("height", 720))
            self._cap.set(cv2.CAP_PROP_FPS, cam.get("fps", 30))
            # Small grab buffer keeps the latest frame fresh (low display latency).
            try:
                self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
        if not self._cap.isOpened():
            raise RuntimeError(f"VisionImageProcessing: cannot open camera source {src!r}")
        if isinstance(src, int):
            # Tune driver controls (default: stop auto-exposure from throttling FPS).
            controls = vision_config.get("camera_controls")
            if controls is None:
                controls = {"exposure_dynamic_framerate": 0}
            _apply_v4l2_controls(src, controls)
            actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
            w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"[VISION] Camera opened {w}x{h} @ {actual_fps:.0f} fps (MJPG)")

        # Class map: vision class name → scheduler object_type
        self._class_map: dict[str, str] = vision_config.get("class_map", {})

        # Thread-safe detection queue.
        self._deque: collections.deque[ObjectDetection] = collections.deque()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # Conveyor transform (no numpy required — pure-Python _mat_apply).
        from modules.conveyor import M_VISION_TO_CONVEYOR, _mat_apply
        self._vision_to_conveyor = M_VISION_TO_CONVEYOR
        self._mat_apply = _mat_apply

        # Import teammate helpers (safe — detect_realtime.main is __name__-guarded).
        if yolo_dir not in sys.path:
            sys.path.insert(0, yolo_dir)
        from src.tracker import CentroidTracker
        from src.trigger import TriggerLine
        from src.geometry import normalize_angle_deg
        from src.detect_realtime import extract_obb, in_roi, roi_coords_cm, draw as _draw_overlay

        self._tracker = CentroidTracker(
            max_match_dist=tk.get("max_match_dist_px", 80),
            max_missing=tk.get("max_missing_frames", 15),
        )
        self._trigger = TriggerLine(self._tl_y_px, self._tl_direction)
        self._normalize_angle_deg = normalize_angle_deg
        self._extract_obb = extract_obb
        self._in_roi = in_roi
        self._roi_coords_cm = roi_coords_cm
        self._draw_overlay = _draw_overlay

        if self._ori_enabled:
            from src.orientation import (
                pick_marker,
                heading_from_marker_vector,
                resolve_heading_360,
            )
            self._pick_marker = pick_marker
            self._heading_from_marker_vector = heading_from_marker_vector
            self._resolve_heading_360 = resolve_heading_360
        else:
            self._pick_marker = self._heading_from_marker_vector = self._resolve_heading_360 = None

        self._cv2 = cv2
        self._np = np
        self._counter = 0

        # Live overlay window (boxes + id/pos/angle + FPS). Default on for any
        # scenario that opens the real camera; set vision.show_window=false to run headless.
        self._show_window = bool(vision_config.get("show_window", True))
        self._ori_ctx = {
            "enabled": self._ori_enabled,
            "marker_map": self._marker_map,
            "offset": self._heading_offset,
        }
        # FPS readouts: camera capture rate and inference/processing rate (GPU max).
        self._cam_fps = 0.0
        self._proc_fps = 0.0
        self._last_frame_t: float | None = None
        # The vision thread renders the annotated frame here; the MAIN thread
        # (render_window) owns cv2.imshow/waitKey — Qt requires GUI on the main thread.
        self._display_frame = None
        self._display_lock = threading.Lock()
        # Latest captured frame, published by a dedicated capture thread so the
        # measured camera FPS reflects the camera's true rate, independent of how
        # slow inference is. The inference loop consumes the most recent frame.
        self._latest_frame = None
        self._latest_frame_id = 0
        self._frame_lock = threading.Lock()

        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True, name="VisionCapture")
        self._capture_thread.start()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="VisionThread")
        self._thread.start()
        print(f"[VISION] Pipeline started (src={src!r}, weights={os.path.basename(weights)})")

    def _capture_loop(self) -> None:
        """Read frames at the camera's native rate in a dedicated thread.

        Decoupling capture from inference means `cam_fps` measures the real
        camera throughput (e.g. 30) instead of being throttled by YOLO latency.
        """
        cv2 = self._cv2
        alpha = 0.3
        try:
            while not self._stop_event.is_set():
                ret, frame = self._cap.read()
                t = time.monotonic()
                if not ret:
                    print("[VISION] Camera read failed — stopping vision threads.")
                    self._stop_event.set()
                    break
                if self._last_frame_t is not None:
                    dt = t - self._last_frame_t
                    if dt > 0.0:
                        self._cam_fps = alpha * (1.0 / dt) + (1.0 - alpha) * self._cam_fps
                self._last_frame_t = t
                with self._frame_lock:
                    self._latest_frame = frame
                    self._latest_frame_id += 1
        finally:
            self._cap.release()

    def _loop(self) -> None:
        """Inference loop — consumes the most recent captured frame and runs YOLO.

        Runs as fast as the model allows; `proc_fps` therefore reflects the GPU's
        max sustained throughput. Skips re-processing a frame it has already seen.
        """
        cv2 = self._cv2
        try:
            alpha = 0.3  # EMA smoothing for the processing-FPS readout
            last_id = -1
            while not self._stop_event.is_set():
                with self._frame_lock:
                    frame = self._latest_frame
                    fid = self._latest_frame_id
                if frame is None or fid == last_id:
                    time.sleep(0.001)
                    continue
                last_id = fid
                frame = frame.copy()  # private copy — safe to annotate without racing capture

                t_proc0 = time.monotonic()
                if self._K is not None:
                    frame = cv2.undistort(frame, self._K, self._D)

                result = self._model.predict(
                    frame,
                    conf=self._conf_marker,
                    iou=self._iou_th,
                    imgsz=self._imgsz,
                    device=self._device,
                    verbose=False,
                )[0]

                dets = list(self._extract_obb(result))
                dets = [d for d in dets if self._in_roi(self._roi_poly, d[0], d[1])]

                pcb_dets = [d for d in dets if self._names[d[5]] in self._pcb_classes and d[6] >= self._conf_th]
                marker_dets = [d for d in dets if self._names[d[5]] in self._marker_classes] if self._ori_enabled else []

                centroids = [(d[0], d[1]) for d in pcb_dets]
                active = self._tracker.update(centroids)

                for tid, trk in active.items():
                    if not pcb_dets:
                        continue
                    di = min(
                        range(len(pcb_dets)),
                        key=lambda i: (pcb_dets[i][0] - trk.cx) ** 2 + (pcb_dets[i][1] - trk.cy) ** 2,
                    )
                    board = pcb_dets[di]
                    cx, cy, w, h, theta, cls_id, conf = board

                    if conf < self._tl_min_conf:
                        continue
                    if not self._trigger.crossed(trk):
                        continue

                    # Pixel → mm in ROI frame.
                    rc = self._roi_coords_cm(cx, cy, self._roi_poly, self._pixels_per_mm)
                    if rc is not None:
                        _, _, x_cm, y_cm = rc
                        x_mm, y_mm = x_cm * 10.0, y_cm * 10.0
                    else:
                        x_mm = cx / self._pixels_per_mm
                        y_mm = cy / self._pixels_per_mm

                    # Orientation.
                    if self._ori_enabled:
                        marker, inferred = self._pick_marker(
                            board, marker_dets, self._names, self._marker_map,
                            max_dist_px=self._pixels_per_mm * 20.0,
                        )
                        type_name = inferred if (self._cross_check and inferred) else self._names[cls_id]
                        angle = self._heading_from_marker_vector(board, marker, self._heading_offset)
                        if angle is None:
                            sym = float(self._symmetry_by_class.get(type_name, self._symmetry_default))
                            angle = self._resolve_heading_360(board, None, self._heading_offset, sym)
                    else:
                        type_name = self._names[cls_id]
                        angle = self._normalize_angle_deg(theta)

                    # Map class name → scheduler object_type.
                    mapped = self._class_map.get(type_name)
                    if mapped is None:
                        print(f"[VISION] Unknown class '{type_name}' — skipping (not in class_map)")
                        continue

                    # Vision ROI frame → C-frame (u, v).
                    u, v = self._mat_apply(self._vision_to_conveyor, x_mm, y_mm)

                    self._counter += 1
                    det = ObjectDetection(
                        object_id=f"yolo-{tid:06d}",
                        x=u,
                        y=v,
                        object_type=mapped,
                        timestamp=time.monotonic(),
                        confidence=float(conf),
                        angle_deg=float(angle),
                    )
                    with self._lock:
                        self._deque.append(det)

                # Processing FPS = inverse of one capture→postprocess cycle (GPU max throughput).
                dt_proc = time.monotonic() - t_proc0
                if dt_proc > 0.0:
                    self._proc_fps = alpha * (1.0 / dt_proc) + (1.0 - alpha) * self._proc_fps

                if self._show_window:
                    self._draw_overlay(
                        frame, self._trigger, pcb_dets, marker_dets, self._names,
                        self._roi_poly, self._ori_ctx, self._pixels_per_mm,
                        active_tracks=active,
                    )
                    cv2.putText(frame, f"CAM {self._cam_fps:.1f} FPS", (10, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"PROC {self._proc_fps:.1f} FPS (GPU max)", (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    # Hand the annotated frame to the main thread for display.
                    with self._display_lock:
                        self._display_frame = frame

        except Exception as exc:
            print(f"[VISION] Thread error: {exc}")
        finally:
            self._stop_event.set()  # signal the capture thread to stop too
            print("[VISION] Thread stopped.")

    def poll(self, now: float) -> list[ObjectDetection]:  # noqa: ARG002
        with self._lock:
            detections = list(self._deque)
            self._deque.clear()
        return detections

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=5.0)
        self._capture_thread.join(timeout=5.0)

    def render_window(self) -> bool:
        """Pump the GUI from the MAIN thread (Qt requires this). Displays the
        latest annotated frame and processes key events. Returns False once the
        window should close (user pressed 'q' or the thread stopped)."""
        if not self._show_window:
            return not self._stop_event.is_set()
        cv2 = self._cv2
        with self._display_lock:
            frame = self._display_frame
        if frame is not None:
            cv2.imshow("Delta Vision", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self._stop_event.set()
        return not self._stop_event.is_set()

    def close_window(self) -> None:
        """Destroy the OpenCV window. Call from the MAIN thread."""
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
            dets = vip.poll(time.monotonic())
            for d in dets:
                print(f"[DETECTION] {d.to_dict()}")
            # render_window() pumps the GUI on the MAIN thread (Qt requirement).
            if not vip.render_window():
                break
            if args.no_window:
                time.sleep(0.1)  # waitKey already paces the windowed path
    except KeyboardInterrupt:
        pass
    finally:
        vip.stop()
        vip.close_window()
    print("[SMOKE] Done.")
