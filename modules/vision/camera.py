"""Camera device helpers: find the camera by USB id and tune its v4l2 controls."""
from __future__ import annotations

import os
import shutil
import subprocess
import sys


def find_camera_by_usb_id(vendor_product: str) -> int | None:
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


def apply_v4l2_controls(device: str, controls: dict) -> None:
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
