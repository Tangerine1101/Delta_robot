"""
calibrate_everything.py — whole-config calibrator + validator for the Delta Robot.

Companion to `camera_calibrate.py`. Where that tool calibrates the `vision`
section interactively, this one validates the *entire* parameter set for mutual
consistency and physical safety, orchestrates the camera tool, and leaves
clearly-marked hooks for future physical calibration.

It answers, in one place: does the config load (every key known, heights in
order), is the vision section sane, and — most importantly — does any point the
robot is actually commanded to during operation push it OUTSIDE the physical
motion envelope (`robot.limits`: XY circle and z band)?

Design notes:
  * Read-only by default. The script itself only writes config under --fix, and
    only for SAFE DERIVED values (e.g. robot.heights.slope_transition = midpoint). It
    never rewrites hand-measured physical values, and never clamps an
    out-of-circle workspace (CLAUDE.md §4.4: discard, not clamp — we *suggest* a
    fitted window instead).
  * The camera stage delegates to `camera_calibrate.py` as a subprocess: that
    tool must set QT_QPA_PLATFORM=xcb before importing cv2, so a separate process
    keeps the env isolated and keeps GUI deps out of the headless validators.
  * Importing modules.settings / modules.core does not pull cv2/ultralytics, so
    --check runs headless / in CI.

Usage:
    python3 calibrate_everything.py            # all non-interactive validators + report
    python3 calibrate_everything.py --check    # validators only (read-only, CI-friendly)
    python3 calibrate_everything.py --workspace# forbidden-circle / workspace stage only
    python3 calibrate_everything.py --camera    # delegate to camera_calibrate.py (GUI)
    python3 calibrate_everything.py --fix       # apply safe derived corrections, re-validate
    python3 calibrate_everything.py --no-save   # never write (overrides --fix)

Exit code is non-zero if any validator fails.
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
from typing import Any

from modules.config_io import CONFIG_PATH as _CONFIG_PATH, read_config, write_config
from modules.core.frames import ConveyorFrame, is_within_xy_limit
from modules.core.trajectory import goto_waypoints, pick_waypoints
from modules.scheduling.registry import validate_plugin_config
from modules.settings import Settings, settings_from_dict

ROOT = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = str(_CONFIG_PATH)
CAMERA_SCRIPT = os.path.join(ROOT, "camera_calibrate.py")

# A check result: (name, ok, detail).
CheckResult = tuple[str, bool, str]


# ---------------------------------------------------------------------------
# Config IO (raw json, mirroring camera_calibrate.load_config/save_config)
# ---------------------------------------------------------------------------


def _load_raw() -> dict[str, Any]:
    return read_config(CONFIG_PATH)


def _save_raw(cfg: dict[str, Any]) -> None:
    write_config(cfg, CONFIG_PATH)
    print(f"[OK] wrote {CONFIG_PATH}")


def _build_settings(raw: dict[str, Any]) -> tuple[Settings | None, str | None]:
    """Build Settings from the raw config. Returns (settings, error).

    The loader rejects unknown or moved keys and validates the height hierarchy and the
    plugin names, so any of those surfaces here as an error string instead of a crash.
    """
    try:
        settings = settings_from_dict(raw)
        validate_plugin_config(settings)
        return settings, None
    except Exception as exc:  # noqa: BLE001 — surface any config error as a FAIL
        return None, str(exc)


# ---------------------------------------------------------------------------
# Stage A — structural / consistency validation (read-only)
# ---------------------------------------------------------------------------


def validate_structure(
    raw: dict[str, Any],
    settings: Settings | None,
    settings_error: str | None,
) -> list[CheckResult]:
    results: list[CheckResult] = []

    if settings is None:
        results.append(("config loads (keys, types, height hierarchy, plugins)", False,
                        settings_error or "unknown error"))
        # Without settings we cannot run the structural checks that depend on it.
        return results
    h = settings.robot.heights
    results.append((
        "config loads (keys, types, height hierarchy, plugins)",
        True,
        f"clearance {h.clearance} > slope {h.slope_transition} > pre_pick {h.pre_pick} "
        f"> pickup {h.pickup}; place {h.place}",
    ))

    # The simulator feeds boards on these lanes; they must fall within the workspace v-range.
    ws = settings.conveyor.workspace_window_uv
    lane_bad = [v for v in settings.plc_sim.feed_lanes if not (ws[2] <= v <= ws[3])]
    results.append(("plc_sim.feed_lanes within workspace v-range", not lane_bad,
                    "all inside" if not lane_bad else f"outside [{ws[2]}, {ws[3]}]: {lane_bad}"))
    unknown_feed = [t for t in settings.plc_sim.feed_types if t not in settings.object_types]
    results.append(("plc_sim.feed_types are object_types", not unknown_feed,
                    "all known" if not unknown_feed else f"unknown: {unknown_feed}"))

    # Vision sanity (from raw config).
    results.extend(_validate_vision(raw))
    return results


def _validate_vision(raw: dict[str, Any]) -> list[CheckResult]:
    results: list[CheckResult] = []
    vision = raw.get("vision", {}) or {}

    polygon = (vision.get("roi", {}) or {}).get("polygon") or []
    results.append(("vision.roi.polygon has >=3 points", len(polygon) >= 3,
                    f"{len(polygon)} points"))

    ppm = vision.get("pixels_per_mm", 0)
    results.append(("vision.pixels_per_mm > 0", isinstance(ppm, (int, float)) and ppm > 0,
                    f"pixels_per_mm={ppm}"))

    cap_h = int((vision.get("capture", {}) or {}).get("height", 1080))
    y_px = (vision.get("trigger_line", {}) or {}).get("y_px")
    y_ok = isinstance(y_px, (int, float)) and 0 <= y_px <= cap_h
    results.append(("vision.trigger_line.y_px within frame height", bool(y_ok),
                    f"y_px={y_px}, frame height={cap_h}"))

    weights = vision.get("model_weights", "")
    weights_path = weights if os.path.isabs(weights) else os.path.join(ROOT, weights)
    exists = bool(weights) and os.path.exists(weights_path)
    results.append(("vision.model_weights file exists", exists, weights or "(unset)"))
    return results


# ---------------------------------------------------------------------------
# Stage B — forbidden-circle ("during operation") check
# ---------------------------------------------------------------------------


def _sample_workspace_picks(
    ws: tuple[float, float, float, float],
    frame: ConveyorFrame,
    pickup_z: float,
    grid: int = 4,
) -> list[tuple[float, float, float]]:
    """Sample pick positions across the workspace window, returned in R-frame.

    Corners suffice by convexity (the trajectory waypoints stay inside the convex
    hull of {home, pick, sort}), but a coarse grid is cheap and reassuring.
    """
    u_min, u_max, v_min, v_max = ws
    us = [u_min + (u_max - u_min) * i / (grid - 1) for i in range(grid)] if grid > 1 else [(u_min + u_max) / 2]
    vs = [v_min + (v_max - v_min) * i / (grid - 1) for i in range(grid)] if grid > 1 else [(v_min + v_max) / 2]
    picks: list[tuple[float, float, float]] = []
    for u in us:
        for v in vs:
            x, y = frame.to_robot(u, v)
            picks.append((x, y, pickup_z))
    return picks


def validate_forbidden_circle(
    raw: dict[str, Any],
    settings: Settings | None,
) -> list[CheckResult]:
    results: list[CheckResult] = []
    if settings is None:
        results.append(("forbidden-circle check", False, "config did not load"))
        return results

    envelope = settings.robot.limits
    limit = envelope.radius_xy_mm
    frame = ConveyorFrame.from_settings(settings.conveyor)
    ws = settings.conveyor.workspace_window_uv
    home = settings.robot.home_position

    # 1) Static commanded points: home and the bins.
    static_points: list[tuple[str, float, float]] = [("robot.home_position", home[0], home[1])]
    for name, spec in settings.object_types.items():
        static_points.append((f"object_types.{name}.bin", spec.bin[0], spec.bin[1]))

    static_bad = [(name, math.hypot(x, y)) for name, x, y in static_points
                  if not is_within_xy_limit(x, y, limit)]
    results.append((
        f"static commanded points within {limit:.1f} mm circle",
        not static_bad,
        f"checked {len(static_points)} points, all inside"
        if not static_bad
        else "; ".join(f"{n} r={r:.1f}" for n, r in static_bad),
    ))

    # 2) Full trajectory waypoints across the workspace, during operation.
    picks = _sample_workspace_picks(ws, frame, settings.robot.heights.pickup)
    sorts = [spec.bin for spec in settings.object_types.values()] or [home]
    worst: tuple[float, str] | None = None
    n_waypoints = 0
    z_bad: list[str] = []
    for pick in picks:
        for sort in sorts:
            goto = goto_waypoints(home, pick, settings.robot)
            pick_traj = pick_waypoints(pick, sort, settings.robot)
            for phase, traj in (("goto", goto), ("pick", pick_traj)):
                for j, (x, y, z) in enumerate(traj):
                    n_waypoints += 1
                    if not envelope.z_min_mm <= z <= envelope.z_max_mm and len(z_bad) < 3:
                        z_bad.append(f"{phase} P{j + 1} z={z:.1f}")
                    r = math.hypot(x, y)
                    if not is_within_xy_limit(x, y, limit):
                        if worst is None or r > worst[0]:
                            worst = (r, f"{phase} P{j + 1} at pick=({pick[0]:.1f},{pick[1]:.1f}) "
                                        f"sort=({sort[0]:.1f},{sort[1]:.1f}) -> r={r:.1f}")
    results.append((
        f"all operating waypoints within z [{envelope.z_min_mm:.1f}, {envelope.z_max_mm:.1f}]",
        not z_bad,
        f"checked {n_waypoints} waypoints" if not z_bad else "; ".join(z_bad),
    ))
    if worst is None:
        results.append((
            f"all operating waypoints within {limit:.1f} mm circle",
            True,
            f"checked {n_waypoints} waypoints over {len(picks)} pick samples x {len(sorts)} bins",
        ))
    else:
        results.append((
            f"all operating waypoints within {limit:.1f} mm circle",
            False,
            f"worst violation: {worst[1]} (limit {limit:.1f})",
        ))
        suggestion = _suggest_fitted_window(ws, frame, limit)
        if suggestion is None:
            results.append(("suggested workspace_window_uv", False,
                            "window centre is outside the circle — shrinking cannot fix it; "
                            "re-centre the workspace or re-check the conveyor->robot transform"))
        else:
            results.append(("suggested workspace_window_uv", True,
                            f"largest centred window inside circle: {suggestion}"))
    return results


def _suggest_fitted_window(
    ws: tuple[float, float, float, float],
    frame: ConveyorFrame,
    limit: float,
) -> list[float] | None:
    """Largest window concentric with `ws` whose 4 R-frame corners fit the circle.

    The C->R map is affine, so scaling the C-window about its centre scales the
    R-quad about its R-centre; a single scalar s in [0, 1] parametrises it.
    Returns None if the centre itself is already outside the circle.
    """
    u_min, u_max, v_min, v_max = ws
    cu, cv = (u_min + u_max) / 2.0, (v_min + v_max) / 2.0
    hu, hv = (u_max - u_min) / 2.0, (v_max - v_min) / 2.0

    cx, cy = frame.to_robot(cu, cv)
    if not is_within_xy_limit(cx, cy, limit):
        return None

    def corners_fit(s: float) -> bool:
        for su in (-1, 1):
            for sv in (-1, 1):
                x, y = frame.to_robot(cu + su * hu * s, cv + sv * hv * s)
                if not is_within_xy_limit(x, y, limit):
                    return False
        return True

    lo, hi = 0.0, 1.0
    for _ in range(40):  # binary search the max feasible scale
        mid = (lo + hi) / 2.0
        if corners_fit(mid):
            lo = mid
        else:
            hi = mid
    s = lo
    return [round(cu - hu * s, 1), round(cu + hu * s, 1),
            round(cv - hv * s, 1), round(cv + hv * s, 1)]


# ---------------------------------------------------------------------------
# --fix — safe derived writes only
# ---------------------------------------------------------------------------


def apply_safe_fixes(raw: dict[str, Any], no_save: bool) -> list[CheckResult]:
    """Recompute SAFE DERIVED config values. Currently: robot.heights.slope_transition
    as the (clearance + pre_pick)/2 midpoint when missing or out of hierarchy.
    Nothing physical, nothing clamped.
    """
    results: list[CheckResult] = []
    heights = (raw.get("robot", {}) or {}).get("heights", {}) or {}
    clearance = float(heights.get("clearance", -270.0))
    pre_pick = float(heights.get("pre_pick", -290.0))
    midpoint = round((clearance + pre_pick) / 2.0, 3)
    current = heights.get("slope_transition")

    needs = current is None or not (pre_pick < float(current) < clearance)
    if not needs:
        results.append(("fix robot.heights.slope_transition", True, f"already valid ({current}); no change"))
        return results

    if no_save:
        results.append(("fix robot.heights.slope_transition", True,
                        f"WOULD set {current} -> {midpoint} (--no-save: not written)"))
        return results

    raw.setdefault("robot", {}).setdefault("heights", {})["slope_transition"] = midpoint
    _save_raw(raw)
    results.append(("fix robot.heights.slope_transition", True, f"set {current} -> {midpoint}"))
    return results


# ---------------------------------------------------------------------------
# Stage C — camera delegation
# ---------------------------------------------------------------------------


def run_camera_stage(passthrough: list[str]) -> int:
    cmd = [sys.executable, CAMERA_SCRIPT, *passthrough]
    print(f"[INFO] delegating to camera_calibrate.py: {' '.join(cmd)}")
    return subprocess.run(cmd, cwd=ROOT, check=False).returncode


# ---------------------------------------------------------------------------
# Stage D — future physical calibration (documented hooks, not yet implemented)
# ---------------------------------------------------------------------------


def _todo_physical_calibration() -> None:
    """Placeholder for physical-rig calibration, enabled once every scenario runs
    perfectly. Each item should be driven by an existing test scenario:

      * conveyor.frame (theta, robot origin) — from `test_vision_only` board
        readings vs hand-measured robot position.
      * pick_gate.robot_movement_delay_s — from the stationary-belt [GATE]
        measurement (pick-accuracy-findings §3.1).
      * conveyor.position_scale_mm — from measured belt travel over a known move.

    Not yet enabled — see doc/dev-note.md for the calibration roadmap.
    """
    print("[INFO] physical calibration stage is not yet enabled "
          "(needs a fully-working rig; see _todo_physical_calibration docstring).")


# ---------------------------------------------------------------------------
# Report + main
# ---------------------------------------------------------------------------


def _print_results(title: str, results: list[CheckResult]) -> int:
    print(f"\n=== {title} ===")
    failures = 0
    for name, ok, detail in results:
        tag = "[PASS]" if ok else "[FAIL]"
        if not ok:
            failures += 1
        print(f"  {tag} {name} — {detail}")
    return failures


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate and calibrate the whole Delta Robot config.")
    parser.add_argument("--check", action="store_true",
                        help="Run validators only (read-only, no camera, no writes).")
    parser.add_argument("--workspace", action="store_true",
                        help="Run only the workspace / forbidden-circle stage.")
    parser.add_argument("--camera", action="store_true",
                        help="Delegate to camera_calibrate.py (interactive GUI).")
    parser.add_argument("--fix", action="store_true",
                        help="Apply safe derived corrections (e.g. slope_transition midpoint).")
    parser.add_argument("--no-save", action="store_true",
                        help="Never write config (overrides --fix writes).")
    # Passthrough for the camera stage.
    parser.add_argument("--source", default=None, help="Static image for the camera stage.")
    args, camera_extra = parser.parse_known_args(argv)

    raw = _load_raw()
    settings, settings_error = _build_settings(raw)
    failures = 0

    # --camera is a standalone interactive stage (only when no validator-only flag).
    if args.camera and not (args.check or args.workspace):
        passthrough = list(camera_extra)
        if args.source:
            passthrough += ["--source", args.source]
        if args.no_save:
            passthrough += ["--no-save"]
        rc = run_camera_stage(passthrough)
        failures += _print_results("Vision re-validation", _validate_vision(_load_raw()))
        return 1 if (rc != 0 or failures) else 0

    # --workspace runs only the forbidden-circle stage; otherwise run structure too.
    only_workspace = args.workspace
    if not only_workspace:
        failures += _print_results("Stage A — structure & consistency",
                                   validate_structure(raw, settings, settings_error))

    failures += _print_results("Stage B — physical forbidden-circle (during operation)",
                               validate_forbidden_circle(raw, settings))

    if args.fix:
        failures += _print_results("Safe derived fixes", apply_safe_fixes(raw, args.no_save))

    # Full default run (no validator-only flag) ends with the future-work notice.
    if not args.check and not only_workspace and not args.fix:
        _todo_physical_calibration()

    print(f"\n{'[OK] all checks passed' if failures == 0 else f'[FAIL] {failures} check(s) failed'}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
