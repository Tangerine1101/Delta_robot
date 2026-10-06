"""Convert a point between the robot (R), belt (u, v), ROI-mm and camera-pixel frames, and
report a round-trip delta that checks the forward and inverse transforms agree.

    python3 -m modules.tools.frame_convert --robot 0 -285 -310
    python3 -m modules.tools.frame_convert --conveyor 112.5 20.2
    python3 -m modules.tools.frame_convert --pixel 800 600
"""
from __future__ import annotations

import argparse
import math

from modules.core.frames import ConveyorFrame, is_within_xy_limit, uv_to_vision_mm, vision_mm_to_uv
from modules.settings import VisionSettings, load_settings


def _build_roi_frame(vision: VisionSettings):
    import cv2
    import numpy as np

    from modules.vision.roi import RoiFrame

    polygon = [list(point) for point in vision.roi.polygon] or None
    return RoiFrame(polygon, vision.pixels_per_mm, np, cv2)


def _convert(roi_frame, conv_frame: ConveyorFrame, frame: str, values: tuple[float, ...]) -> dict:
    """Compute pixel / ROI-mm / conveyor(u,v) / robot(x,y) for a point given in `frame`.

    Also re-derives the source frame's own coordinates from the computed chain
    (round-trip check) to verify the forward+inverse transforms agree.
    """
    z = values[2] if frame == "robot" and len(values) == 3 else None

    if frame == "robot":
        x, y = values[0], values[1]
        u, v = conv_frame.to_conveyor(x, y)
        x_mm, y_mm = uv_to_vision_mm(u, v)
        px, py = roi_frame.mm_to_pixel(x_mm, y_mm)
        x_rt, y_rt = conv_frame.to_robot(u, v)
        roundtrip = (x_rt - x, y_rt - y)
    elif frame == "conveyor":
        u, v = values[0], values[1]
        x, y = conv_frame.to_robot(u, v)
        x_mm, y_mm = uv_to_vision_mm(u, v)
        px, py = roi_frame.mm_to_pixel(x_mm, y_mm)
        u_rt, v_rt = conv_frame.to_conveyor(x, y)
        roundtrip = (u_rt - u, v_rt - v)
    elif frame == "pixel":
        px, py = values[0], values[1]
        x_mm, y_mm = roi_frame.to_mm(px, py)
        u, v = vision_mm_to_uv(x_mm, y_mm)
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

    settings = load_settings()
    roi_frame = _build_roi_frame(settings.vision)
    conv_frame = ConveyorFrame.from_settings(settings.conveyor)
    limit_radius_xy = settings.robot.limits.radius_xy_mm

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
