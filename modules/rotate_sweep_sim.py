"""Pure-software sweep of the T4 (suction-cup) rotation-angle pipeline.

No hardware required. `modules/test_rotate.py` probes the physical Siemens
axis with the cup free; this script instead exercises the exact PC-side math
chain that turns a vision marker heading into the post-grip rotate command,
across the full 0-360 deg input range, and dumps every stage so the mapping
can be inspected for discontinuities/inconsistencies independent of any
hardware timing question.

Chain reproduced (each function imported from its real module, not
reimplemented):
    raw_marker_angle_deg               (synthetic sweep input, 0-360)
        -> + vision.orientation.offset_by_class[type]   (image_processing convention)
        = vision_angle_deg                               (what ObjectDetection.angle_deg carries)
        -> ConveyorFrame.vision_heading_to_robot_rad      (conveyor.py)
        = board_heading_rad / board_heading_deg
        -> wrap_rad(rotate_sign * (rotate_offset_rad - board_heading_rad))   (scheduler.py formula)
        = rotate_cmd_rad / rotate_cmd_deg
        -> robot_rad_to_wire_deg                          (EthernetCom.py)
        = wire_deg

Usage:
    python3 -m modules.rotate_sweep_sim                     # 1 deg step, both classes
    python3 -m modules.rotate_sweep_sim --step-deg 0.5 --csv /tmp/rotate_sweep.csv
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from modules.EthernetCom import robot_rad_to_wire_deg, wrap_rad
from modules.conveyor import ConveyorFrame

_ROW = tuple[str, float, float, float, float, float, float]


def _offset_by_class(config: dict, object_type: str) -> float:
    ori = config.get("vision", {}).get("orientation", {})
    default = float(ori.get("offset_deg", 0.0))
    return float(ori.get("offset_by_class", {}).get(object_type, default))


def sweep(step_deg: float) -> list[_ROW]:
    """Sweep raw marker angle 0-360 (exclusive) for every PCB class.

    Returns rows of (object_type, raw_marker_deg, vision_angle_deg,
    board_heading_deg, rotate_cmd_deg, wire_deg, step_delta_deg) where
    step_delta_deg is the signed change in rotate_cmd_deg from the previous
    sample of the *same* class (None-safe: first sample of each class is 0.0).
    """
    config = json.loads((Path(__file__).parent / "config.json").read_text())

    pcb_classes = config.get("vision", {}).get("orientation", {}).get(
        "pcb_classes", ["QFP", "TQFP"]
    )
    rotate_offset_rad = math.radians(
        float(config.get("scheduler", {}).get("rotate_offset_deg", 0.0))
    )
    rotate_sign = 1.0 if float(config.get("scheduler", {}).get("rotate_sign", 1.0)) >= 0.0 else -1.0

    frame = ConveyorFrame()

    rows: list[_ROW] = []
    n_steps = max(1, round(360.0 / step_deg))
    for object_type in pcb_classes:
        offset = _offset_by_class(config, object_type)
        prev_cmd_deg: float | None = None
        for i in range(n_steps):
            raw_marker_deg = i * step_deg
            vision_angle_deg = (raw_marker_deg + offset) % 360.0

            board_heading_rad = frame.vision_heading_to_robot_rad(vision_angle_deg)
            board_heading_deg = math.degrees(board_heading_rad)

            rotate_cmd_rad = wrap_rad(rotate_sign * (rotate_offset_rad - board_heading_rad))
            rotate_cmd_deg = math.degrees(rotate_cmd_rad)

            wire_deg = robot_rad_to_wire_deg(rotate_cmd_rad)

            step_delta = 0.0 if prev_cmd_deg is None else rotate_cmd_deg - prev_cmd_deg
            prev_cmd_deg = rotate_cmd_deg

            rows.append((
                object_type,
                round(raw_marker_deg, 3),
                round(vision_angle_deg, 3),
                round(board_heading_deg, 3),
                round(rotate_cmd_deg, 3),
                round(wire_deg, 3),
                round(step_delta, 3),
            ))
    return rows


def summarize(rows: list[_ROW], step_deg: float) -> str:
    lines = ["[ROTATE-SWEEP] summary (per class)"]
    by_class: dict[str, list[_ROW]] = {}
    for row in rows:
        by_class.setdefault(row[0], []).append(row)

    for object_type, class_rows in by_class.items():
        cmd_degs = [r[4] for r in class_rows]
        deltas = [r[6] for r in class_rows[1:]]  # skip the synthetic first-sample 0.0
        expected_step = -step_deg  # rotate_cmd is a -1 slope of board_heading (sign=+1, offset=0)
        # A "clean" step is close to the expected per-degree slope; anything far
        # off (i.e. the wrap discontinuity) is flagged, not explained.
        jumps = [d for d in deltas if abs(d - expected_step) > step_deg * 1.5 and abs(d + expected_step) > step_deg * 1.5]
        lines.append(
            f"  {object_type:6s} n={len(class_rows):4d} "
            f"cmd_deg[min={min(cmd_degs):+7.2f} max={max(cmd_degs):+7.2f} "
            f"mean_abs={sum(abs(c) for c in cmd_degs) / len(cmd_degs):6.2f}] "
            f"near_180_boundary(|cmd|>170deg)={sum(1 for c in cmd_degs if abs(c) > 170.0):4d}/{len(cmd_degs)} "
            f"wrap_discontinuities={len(jumps)}"
        )
        for d in jumps[:3]:
            lines.append(f"    (discontinuity delta example: {d:+.2f} deg between consecutive {step_deg}-deg samples)")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step-deg", type=float, default=1.0,
                        help="Marker-angle sweep resolution in degrees")
    parser.add_argument("--csv", type=str, default=None,
                        help="Optional path to dump the full per-sample table as CSV")
    args = parser.parse_args(argv)

    rows = sweep(args.step_deg)
    print(summarize(rows, args.step_deg))

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "object_type", "raw_marker_deg", "vision_angle_deg",
                "board_heading_deg", "rotate_cmd_deg", "wire_deg", "step_delta_deg",
            ])
            writer.writerows(rows)
        print(f"[ROTATE-SWEEP] wrote {len(rows)} rows -> {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
