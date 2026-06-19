"""
Camera calibration tool — ROI polygon, trigger line, pixel/mm scale.

Written from scratch for the Delta Robot project's Linux/Wayland rig. It reads
and writes the `vision` section of `modules/config.json`. The interactive GUI
mirrors the proven display pattern of `modules/image_processing.py`:

  * QT_QPA_PLATFORM is forced to "xcb" *before* cv2 is imported — the bundled
    OpenCV Qt build ships only the xcb plugin (no Wayland one), so leaving the
    desktop's "wayland;xcb" preference makes the window silently fail to map.
  * A single ASCII-named window is reused across stages and re-`imshow`-ed on
    every loop iteration (continuous GUI pumping keeps it mapped on XWayland).
  * The display buffer is downscaled to fit the screen and shown at 1:1 in a
    WINDOW_AUTOSIZE window; click coordinates are scaled back to full image
    resolution only when written to config.

Camera frames are grabbed with PyAV (FFmpeg/v4l2) — the same Linux-safe path the
production pipeline uses; cv2.VideoCapture is deliberately avoided.

Usage:
    python3 camera_calibrate.py                      # live camera, all 3 stages
    python3 camera_calibrate.py --source frame.jpg   # static image (most stable)
    python3 camera_calibrate.py --roi                # one stage only
    python3 camera_calibrate.py --trigger
    python3 camera_calibrate.py --scale
    python3 camera_calibrate.py --view --source f.jpg  # show current config only
    python3 camera_calibrate.py --no-save            # compute but do not write

Keys: ENTER = confirm/save & next   R = reset stage   Q = skip stage
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time

# OpenCV's bundled Qt has only the xcb platform plugin. Force xcb before cv2 is
# imported (override, not setdefault) — the Wayland desktop exports
# QT_QPA_PLATFORM="wayland;xcb", and Qt would try the missing wayland plugin and
# fail to show the window.
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2  # noqa: E402  (must come after the env override above)
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(ROOT, "modules", "config.json")
WIN = "Delta Calib"
MAX_DISP_W = 1280

# Colours (BGR)
C_ROI = (0, 0, 255)
C_X = (255, 80, 0)
C_Y = (0, 200, 0)
C_O = (0, 255, 255)
C_PT = (0, 255, 255)
C_TRIG = (0, 0, 255)
C_HINT = (0, 220, 255)
C_WHITE = (255, 255, 255)

CORNER_LABELS = ["1-TL", "2-TR", "3-BR", "4-BL(O)"]


# ---------------------------------------------------------------------------
# Config I/O
# ---------------------------------------------------------------------------

def load_config() -> dict:
    with open(CONFIG_PATH, encoding="utf-8") as f:
        return json.load(f)


def save_config(cfg: dict) -> None:
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=4)
    print(f"[OK] Đã ghi {CONFIG_PATH}")


# ---------------------------------------------------------------------------
# Camera capture (Linux-safe: PyAV, no cv2.VideoCapture)
# ---------------------------------------------------------------------------

def resolve_video_device(usb_id: str | None) -> str:
    """Resolve a USB 'vendor:product' id to the lowest matching /dev/videoN."""
    fallback = "/dev/video0"
    if not usb_id:
        return fallback
    parts = usb_id.replace(":", "/").lower().split("/")
    if len(parts) < 2:
        return fallback
    try:
        want = f"{int(parts[0], 16):x}/{int(parts[1], 16):x}"
    except ValueError:
        return fallback
    base = "/sys/class/video4linux"
    if not os.path.isdir(base):
        return fallback
    matches: list[int] = []
    for name in os.listdir(base):
        if not name.startswith("video"):
            continue
        try:
            idx = int(name[5:])
        except ValueError:
            continue
        try:
            with open(os.path.join(base, name, "device", "uevent")) as f:
                content = f.read().lower()
        except OSError:
            continue
        if f"product={want}/" in content or f"product={want}\n" in content:
            if os.path.exists(f"/dev/video{idx}"):
                matches.append(idx)
    if matches:
        dev = f"/dev/video{min(matches)}"
        print(f"[info] Camera USB {usb_id!r} → {dev}")
        return dev
    print(f"[warn] Không tìm thấy USB {usb_id!r} — dùng {fallback}")
    return fallback


def apply_v4l2_controls(device: str, controls: dict) -> None:
    """Best-effort v4l2 control tuning (exposure etc.) before opening the stream."""
    if not controls or sys.platform != "linux" or shutil.which("v4l2-ctl") is None:
        return
    for name, value in controls.items():
        try:
            subprocess.run(
                ["v4l2-ctl", "--device", device, f"--set-ctrl={name}={value}"],
                check=False, capture_output=True, timeout=2.0,
            )
        except Exception as exc:
            print(f"[warn] Không set được {name}={value}: {exc}")


def grab_camera_frame(vision_cfg: dict) -> np.ndarray:
    """Grab one BGR frame from the live camera via PyAV."""
    import av

    cap = vision_cfg.get("capture", {}) or {}
    device = cap.get("device") or resolve_video_device(
        cap.get("camera_usb_id") or vision_cfg.get("camera_usb_id"))
    width = int(cap.get("width", 1920))
    height = int(cap.get("height", 1080))
    fps = int(cap.get("fps", 30))
    pixfmt = cap.get("pixelformat", "mjpeg")

    apply_v4l2_controls(device, vision_cfg.get("v4l2_controls") or {})

    print(f"[info] Mở {device} qua PyAV ({width}x{height} {pixfmt} {fps}fps)…")
    options = {"input_format": pixfmt, "video_size": f"{width}x{height}", "framerate": str(fps)}
    container = av.open(device, format="v4l2", options=options)
    # Discard warmup frames so the sensor stabilises at the configured exposure
    # before taking the calibration snapshot. image_processing.py runs at 30fps
    # continuously so its displayed frames are already post-stabilisation; without
    # this the first frame is captured at whatever exposure the camera had before
    # apply_v4l2_controls ran.
    warmup = int(cap.get("warmup_frames", 8))
    try:
        count = 0
        for frame in container.decode(video=0):
            count += 1
            if count >= warmup:
                return frame.to_ndarray(format="bgr24")
    finally:
        container.close()
    raise RuntimeError("PyAV không decode được frame nào.")


def get_frame(args, vision_cfg: dict) -> np.ndarray:
    if args.source:
        print(f"[info] Đọc ảnh tĩnh: {args.source}")
        img = cv2.imread(args.source)
        if img is None:
            raise FileNotFoundError(f"Không đọc được ảnh: {args.source}")
        return img
    return grab_camera_frame(vision_cfg)


# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------

def fit_display(orig: np.ndarray) -> tuple[np.ndarray, float]:
    """Downscale to <= MAX_DISP_W wide. Returns (disp_buffer, scale=orig/disp)."""
    h, w = orig.shape[:2]
    if w <= MAX_DISP_W:
        return orig.copy(), 1.0
    scale = w / MAX_DISP_W
    disp = cv2.resize(orig, (MAX_DISP_W, int(round(h / scale))),
                      interpolation=cv2.INTER_AREA)
    return disp, scale


def open_window(first_img: np.ndarray) -> None:
    """Create the shared window and raise it to the front (best-effort)."""
    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(WIN, first_img)
    cv2.waitKey(1)
    try:
        subprocess.run(["wmctrl", "-a", WIN], check=False, capture_output=True, timeout=2.0)
    except Exception:
        pass


def put_banner(img: np.ndarray, text: str, y: int = 26) -> None:
    """Readable text with a dark outline."""
    cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
    cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_WHITE, 1)


def draw_roi_axes(img: np.ndarray, polygon: list) -> None:
    """Draw ROI outline + O/X/Y axes. polygon in DISPLAY coords. Convention:
    O = poly[3] (bottom-left), X+ = poly[2] (bottom-right), Y+ = poly[0] (top-left)."""
    pts = [tuple(int(v) for v in p) for p in polygon]
    cv2.polylines(img, [np.array(pts, dtype=np.int32)], True, C_ROI, 2)
    O, Xp, Yp = pts[3], pts[2], pts[0]
    cv2.arrowedLine(img, O, Xp, C_X, 2, tipLength=0.04)
    cv2.putText(img, "X", ((O[0] + Xp[0]) // 2, (O[1] + Xp[1]) // 2 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_X, 2)
    cv2.arrowedLine(img, O, Yp, C_Y, 2, tipLength=0.04)
    cv2.putText(img, "Y", (O[0] - 26, (O[1] + Yp[1]) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_Y, 2)
    cv2.circle(img, O, 6, C_O, -1)
    cv2.putText(img, "O", (O[0] + 8, O[1] + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_O, 2)


# ---------------------------------------------------------------------------
# Stage A — ROI polygon
# ---------------------------------------------------------------------------

def stage_roi(disp: np.ndarray, scale: float, vision_cfg: dict) -> dict | None:
    """Click 4 corners TL→TR→BR→BL. Returns updated vision_cfg or None if skipped."""
    state = {"clicks": [], "last_t": 0.0}  # display coords

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP and len(state["clicks"]) < 4:
            now = time.monotonic()
            if now - state["last_t"] > 0.3:
                state["last_t"] = now
                state["clicks"].append((x, y))

    cv2.setMouseCallback(WIN, on_mouse)
    print("\n[Stage A] ROI — click 4 góc: TL → TR → BR → BL(O)  |  ENTER=lưu  R=reset  Q=bỏ qua")

    while True:
        img = disp.copy()
        clicks = state["clicks"]
        nxt = CORNER_LABELS[len(clicks)] if len(clicks) < 4 else "ENTER de luu"
        put_banner(img, f"ROI: click TL>TR>BR>BL  |  tiep: {nxt}  |  ENTER=luu R=reset Q=bo qua")
        for i, pt in enumerate(clicks):
            cv2.circle(img, pt, 6, C_PT, -1)
            cv2.putText(img, CORNER_LABELS[i], (pt[0] + 8, pt[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_PT, 2)
        for i in range(1, len(clicks)):
            cv2.line(img, clicks[i - 1], clicks[i], C_ROI, 1)
        if len(clicks) == 4:
            draw_roi_axes(img, clicks)

        cv2.imshow(WIN, img)
        key = cv2.waitKey(20) & 0xFF
        if key == ord('r'):
            state["clicks"] = []
            print("[info] Reset.")
        elif key == ord('q'):
            print("[Stage A] Bỏ qua.")
            return None
        elif key == 13 and len(clicks) == 4:
            polygon = [[int(round(x * scale)), int(round(y * scale))] for (x, y) in clicks]
            print("[Stage A] ROI (full-res):")
            for lbl, pt in zip(CORNER_LABELS, polygon):
                print(f"   {lbl:8s}: {pt}")
            new = dict(vision_cfg)
            roi = dict(new.get("roi", {}) or {})
            roi["enabled"] = True
            roi["polygon"] = polygon
            new["roi"] = roi
            return new
        elif key == 13:
            print(f"[warn] Mới {len(clicks)}/4 điểm.")


# ---------------------------------------------------------------------------
# Stage B — Trigger line
# ---------------------------------------------------------------------------

def stage_trigger(disp: np.ndarray, scale: float, vision_cfg: dict) -> dict | None:
    """Move mouse to preview a horizontal line, click to set Y. Returns cfg or None."""
    cur_full = int((vision_cfg.get("trigger_line") or {}).get("y_px", 0))
    cur_disp = int(round(cur_full / scale)) if scale else cur_full
    h, w = disp.shape[:2]
    cur_disp = max(0, min(h - 1, cur_disp))
    state = {"y": cur_disp, "hover": None, "last_t": 0.0}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            state["hover"] = y
        elif event == cv2.EVENT_LBUTTONUP:
            now = time.monotonic()
            if now - state["last_t"] > 0.3:
                state["last_t"] = now
                state["y"] = y
                state["hover"] = None

    cv2.setMouseCallback(WIN, on_mouse)
    print(f"\n[Stage B] Trigger — di chuột + click chọn Y  |  ENTER=lưu  R=reset  Q=bỏ qua "
          f"(hiện tại y_px={cur_full})")

    while True:
        img = disp.copy()
        put_banner(img, "TRIGGER: click chon Y  |  ENTER=luu  R=reset  Q=bo qua")
        # chosen line (solid red)
        cv2.line(img, (0, state["y"]), (w, state["y"]), C_TRIG, 2)
        cv2.putText(img, f"y={int(round(state['y'] * scale))}", (10, max(20, state["y"] - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_TRIG, 2)
        # hover preview (dim)
        if state["hover"] is not None and state["hover"] != state["y"]:
            cv2.line(img, (0, state["hover"]), (w, state["hover"]), (0, 140, 255), 1)

        cv2.imshow(WIN, img)
        key = cv2.waitKey(20) & 0xFF
        if key == ord('r'):
            state["y"] = cur_disp
            print(f"[info] Reset về y_px={cur_full}.")
        elif key == ord('q'):
            print("[Stage B] Bỏ qua.")
            return None
        elif key == 13:
            y_full = int(round(state["y"] * scale))
            new = dict(vision_cfg)
            tl = dict(new.get("trigger_line") or {})
            tl["y_px"] = y_full
            new["trigger_line"] = tl
            print(f"[Stage B] trigger_line.y_px = {y_full}")
            return new


# ---------------------------------------------------------------------------
# Stage C — Pixel/mm scale
# ---------------------------------------------------------------------------

def stage_scale(disp: np.ndarray, scale: float, vision_cfg: dict) -> dict | None:
    """Click 2 points on a known-length object, type the real mm. Returns cfg or None."""
    roi_poly = vision_cfg.get("roi", {}).get("polygon", [])
    # roi_poly is full-res; convert to display for context drawing
    roi_disp = [[p[0] / scale, p[1] / scale] for p in roi_poly] if len(roi_poly) == 4 else None

    state = {"pts": [], "done": False, "last_t": 0.0}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP and len(state["pts"]) < 2:
            now = time.monotonic()
            if now - state["last_t"] > 0.3:
                state["last_t"] = now
                state["pts"].append((x, y))

    cv2.setMouseCallback(WIN, on_mouse)
    print("\n[Stage C] Scale — click 2 điểm trên vật tham chiếu  |  R=reset  Q=bỏ qua "
          f"(hiện tại pixels_per_mm={vision_cfg.get('pixels_per_mm', '?')})")

    while True:
        img = disp.copy()
        if roi_disp:
            draw_roi_axes(img, roi_disp)
        pts = state["pts"]
        if not pts:
            hint = "click diem 1"
        elif len(pts) == 1:
            hint = "click diem 2"
        else:
            d = math.dist(pts[0], pts[1]) * scale
            hint = f"{d:.1f}px full-res -> nhap mm o terminal"
        put_banner(img, f"SCALE: {hint}  |  R=reset  Q=bo qua")
        for pt in pts:
            cv2.circle(img, (int(pt[0]), int(pt[1])), 6, C_PT, -1)
        if len(pts) == 2:
            cv2.line(img, pts[0], pts[1], C_PT, 2)

        cv2.imshow(WIN, img)
        key = cv2.waitKey(20) & 0xFF
        if key == ord('r'):
            state["pts"] = []
            print("[info] Reset.")
        elif key == ord('q'):
            print("[Stage C] Bỏ qua.")
            return None
        elif len(pts) == 2:
            break

    px_disp = math.dist(state["pts"][0], state["pts"][1])
    px_full = px_disp * scale
    print(f"[OK] Khoảng cách: {px_full:.2f} px (full-res)")
    while True:
        try:
            real_mm = float(input("Nhập khoảng cách thực tế (mm): ").strip())
            if real_mm <= 0:
                raise ValueError
            break
        except ValueError:
            print("[err] Nhập số dương (vd 30.0).")

    ppm = px_full / real_mm
    print(f"\n  {px_full:.2f} px = {real_mm:.2f} mm  ➜  pixels_per_mm = {ppm:.4f} "
          f"(1 px = {1/ppm:.4f} mm)\n")
    new = dict(vision_cfg)
    new["pixels_per_mm"] = round(ppm, 4)
    print(f"[Stage C] pixels_per_mm = {round(ppm, 4)}")
    return new


# ---------------------------------------------------------------------------
# View-only
# ---------------------------------------------------------------------------

def overlay_current(disp: np.ndarray, scale: float, vision_cfg: dict) -> np.ndarray:
    """Draw current config (ROI, trigger, ppm) onto a display buffer."""
    img = disp.copy()
    h, w = img.shape[:2]
    roi_poly = vision_cfg.get("roi", {}).get("polygon", [])
    if len(roi_poly) == 4:
        draw_roi_axes(img, [[p[0] / scale, p[1] / scale] for p in roi_poly])
    else:
        put_banner(img, "roi.polygon: chua set", 50)
    tl_y = int((vision_cfg.get("trigger_line") or {}).get("y_px", -1))
    if tl_y >= 0:
        yd = int(round(tl_y / scale))
        cv2.line(img, (0, yd), (w, yd), C_TRIG, 2)
        cv2.putText(img, f"trigger y={tl_y}", (10, max(20, yd - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_TRIG, 2)
    cv2.putText(img, f"pixels_per_mm={vision_cfg.get('pixels_per_mm', '?')}",
                (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_HINT, 2)
    return img


def view_only(disp: np.ndarray, scale: float, vision_cfg: dict) -> None:
    print("[view] Hiện config hiện tại — bấm Q để thoát.")
    while True:
        img = overlay_current(disp, scale, vision_cfg)
        put_banner(img, "VIEW config hien tai  |  Q=thoat")
        cv2.imshow(WIN, img)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break


# ---------------------------------------------------------------------------
# Result image
# ---------------------------------------------------------------------------

def save_result(disp: np.ndarray, scale: float, vision_cfg: dict) -> None:
    img = overlay_current(disp, scale, vision_cfg)
    out = os.path.join(ROOT, "calib_result.jpg")
    cv2.imwrite(out, img)
    print(f"[OK] Ảnh kết quả: {out}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="Camera calibration: ROI + trigger + pixel/mm")
    ap.add_argument("--source", default=None, help="Ảnh tĩnh (jpg/png). Mặc định: chụp camera.")
    ap.add_argument("--roi", action="store_true", help="Chỉ stage ROI")
    ap.add_argument("--trigger", action="store_true", help="Chỉ stage trigger line")
    ap.add_argument("--scale", action="store_true", help="Chỉ stage pixel/mm")
    ap.add_argument("--view", action="store_true", help="Chỉ xem config hiện tại")
    ap.add_argument("--no-save", action="store_true", help="Tính nhưng không ghi config")
    args = ap.parse_args()

    cfg = load_config()
    vision_cfg = dict(cfg.get("vision", {}))

    try:
        frame = get_frame(args, vision_cfg)
    except Exception as exc:
        print(f"[err] Không lấy được frame: {exc}")
        sys.exit(1)

    h, w = frame.shape[:2]
    disp, scale = fit_display(frame)
    print(f"[OK] Frame {w}x{h}px → hiển thị {disp.shape[1]}x{disp.shape[0]} (scale {scale:.3f})")

    open_window(disp)

    if args.view:
        view_only(disp, scale, vision_cfg)
        cv2.destroyAllWindows()
        return

    run_all = not (args.roi or args.trigger or args.scale)
    changed = False

    for flag, stage in ((args.roi, stage_roi),
                        (args.trigger, stage_trigger),
                        (args.scale, stage_scale)):
        if run_all or flag:
            result = stage(disp, scale, vision_cfg)
            if result is not None:
                vision_cfg = result
                changed = True

    if changed:
        save_result(disp, scale, vision_cfg)
        if args.no_save:
            print("[info] --no-save: config không được ghi.")
        else:
            cfg["vision"] = vision_cfg
            save_config(cfg)
            print("[OK] Hoàn thành. Khởi động lại pipeline để áp dụng.")
    else:
        print("[info] Không stage nào thay đổi — config giữ nguyên.")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
