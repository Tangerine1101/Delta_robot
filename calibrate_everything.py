"""
calibrate_everything.py — whole-config calibrator + validator for the Delta Robot.

Companion to `camera_calibrate.py`. Where that tool calibrates the `vision`
section interactively, this one validates the *entire* parameter set for mutual
consistency and physical safety, orchestrates the camera tool, and leaves
clearly-marked hooks for future physical calibration.

It answers, in one place: are the scheduler heights sane, do the test-scenario
configs sit inside the workspace, and — most importantly — does any config the
robot is actually commanded to during operation push it OUTSIDE the physical
forbidden circle (`limit_radius_xy`)?

Design notes:
  * Read-only by default. The script itself only writes config under --fix, and
    only for SAFE DERIVED values (e.g. slope_transition_height = midpoint). It
    never rewrites hand-measured physical values, and never clamps an
    out-of-circle workspace (CLAUDE.md §4.4: discard, not clamp — we *suggest* a
    fitted window instead).
  * The camera stage delegates to `camera_calibrate.py` as a subprocess: that
    tool must set QT_QPA_PLATFORM=xcb before importing cv2, so a separate process
    keeps the env isolated and keeps GUI deps out of the headless validators.
  * Importing modules.scheduler / modules.conveyor does not pull cv2/ultralytics
    (lazy inside image_processing), so --check runs headless / in CI.

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
import json
import math
import os
import subprocess
import sys
from typing import Any

from modules.EthernetCom import load_config
from modules.conveyor import ConveyorFrame, is_within_xy_limit
from modules.scheduler import (
    SchedulerSettings,
    _build_goto_geometry,
    _build_pick_geometry,
)

ROOT = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(ROOT, "modules", "config.json")
CAMERA_SCRIPT = os.path.join(ROOT, "camera_calibrate.py")
DEFAULT_LIMIT_RADIUS_XY = 180.0

# A check result: (name, ok, detail).
CheckResult = tuple[str, bool, str]


# ---------------------------------------------------------------------------
# Config IO (raw json, mirroring camera_calibrate.load_config/save_config)
# ---------------------------------------------------------------------------


def _load_raw() -> dict[str, Any]:
    with open(CONFIG_PATH, encoding="utf-8") as handle:
        return json.load(handle)


def _save_raw(cfg: dict[str, Any]) -> None:
    with open(CONFIG_PATH, "w", encoding="utf-8") as handle:
        json.dump(cfg, handle, indent=4)
    print(f"[OK] wrote {CONFIG_PATH}")


def _build_settings() -> tuple[SchedulerSettings | None, str | None]:
    """Build SchedulerSettings from the live config. Returns (settings, error).

    SchedulerSettings.from_config() calls .validate() at the end, so a bad height
    hierarchy raises here — we capture it as an error string instead of crashing.
    """
    try:
        return SchedulerSettings.from_config(load_config()), None
    except Exception as exc:  # noqa: BLE001 — surface any config error as a FAIL
        return None, str(exc)


# ---------------------------------------------------------------------------
# Stage A — structural / consistency validation (read-only)
# ---------------------------------------------------------------------------


def validate_structure(
    raw: dict[str, Any],
    settings: SchedulerSettings | None,
    settings_error: str | None,
) -> list[CheckResult]:
    results: list[CheckResult] = []

    if settings is None:
        results.append(("scheduler settings build + height hierarchy", False, settings_error or "unknown error"))
        # Without settings we cannot run the structural checks that depend on it.
        return results
    results.append((
        "scheduler settings build + height hierarchy",
        True,
        f"clearance {settings.clearance_height} > slope {settings.slope_transition_height} "
        f"> pre_pick {settings.pre_pick_height} > pickup {settings.pickup_height}; "
        f"place {settings.place_height}",
    ))

    # Window ordering.
    for label, window in (("workspace_window_uv", settings.workspace_window_uv),
                          ("camera_window_uv", settings.camera_window_uv)):
        u_min, u_max, v_min, v_max = window
        ok = u_min < u_max and v_min < v_max
        results.append((f"{label} ordering", ok,
                        f"u[{u_min}, {u_max}] v[{v_min}, {v_max}]"))

    ws = settings.workspace_window_uv

    # Simulated spawn / evaluate points must sit inside the workspace window.
    spawn_bad = [pt for pt in settings.accuracy_spawn_uv
                 if not ConveyorFrame.is_in_window_uv(pt[0], pt[1], ws)]
    results.append(("accuracy_spawn_uv inside workspace", not spawn_bad,
                    "all inside" if not spawn_bad else f"outside: {spawn_bad}"))

    apuv_bad = [pt for pt in settings.accuracy_points_uv
                if not ConveyorFrame.is_in_window_uv(pt[0], pt[1], ws)]
    results.append(("accuracy_points_uv inside workspace", not apuv_bad,
                    "all inside (or none configured)" if not apuv_bad else f"outside: {apuv_bad}"))

    # Throughput lanes (v positions) must fall within the workspace v-range.
    lane_bad = [v for v in settings.throughput_lanes if not (ws[2] <= v <= ws[3])]
    results.append(("throughput_lanes within workspace v-range", not lane_bad,
                    "all inside" if not lane_bad else f"outside [{ws[2]}, {ws[3]}]: {lane_bad}"))

    # Each object type's destination must have a sorting position.
    missing_dest = [dest for dest in settings.object_type_map.values()
                    if dest not in settings.sorting_positions]
    results.append(("object_types destinations resolve to sorting positions", not missing_dest,
                    "all resolved" if not missing_dest else f"missing top-level position(s): {missing_dest}"))

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
    settings: SchedulerSettings | None,
) -> list[CheckResult]:
    results: list[CheckResult] = []
    if settings is None:
        results.append(("forbidden-circle check", False, "scheduler settings did not build"))
        return results

    limit = float(raw.get("limit_radius_xy", DEFAULT_LIMIT_RADIUS_XY))
    frame = ConveyorFrame()
    ws = settings.workspace_window_uv

    # 1) Static commanded points: home, sorting positions, accuracy points.
    static_points: list[tuple[str, float, float]] = [
        ("home_position", settings.home_position[0], settings.home_position[1]),
    ]
    for dest, pos in settings.sorting_positions.items():
        static_points.append((f"sorting[{dest}]", pos[0], pos[1]))
    for i, pos in enumerate(settings.accuracy_points):
        static_points.append((f"accuracy_points[{i}]", pos[0], pos[1]))
    for i, pos in enumerate(settings.accuracy_points_uv):
        x, y = frame.to_robot(pos[0], pos[1])
        static_points.append((f"accuracy_points_uv[{i}]->R", x, y))

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
    picks = _sample_workspace_picks(ws, frame, settings.pickup_height)
    sorts = list(settings.sorting_positions.values()) or [settings.home_position]
    worst: tuple[float, str] | None = None
    n_waypoints = 0
    for pick in picks:
        for sort in sorts:
            goto = _build_goto_geometry(settings.home_position, pick, settings)
            pick_traj = _build_pick_geometry(pick, sort, settings, goto)
            for phase, traj in (("goto", goto), ("pick", pick_traj)):
                for j, (x, y, _z) in enumerate(traj):
                    n_waypoints += 1
                    r = math.hypot(x, y)
                    if not is_within_xy_limit(x, y, limit):
                        if worst is None or r > worst[0]:
                            worst = (r, f"{phase} P{j + 1} at pick=({pick[0]:.1f},{pick[1]:.1f}) "
                                        f"sort=({sort[0]:.1f},{sort[1]:.1f}) -> r={r:.1f}")
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
    """Recompute SAFE DERIVED config values. Currently: slope_transition_height
    as the (clearance + pre_pick)/2 midpoint when missing or out of hierarchy.
    Nothing physical, nothing clamped.
    """
    results: list[CheckResult] = []
    sched = raw.get("scheduler", {}) or {}
    clearance = float(sched.get("clearance_height", -270.0))
    pre_pick = float(sched.get("pre_pick_height", -290.0))
    midpoint = round((clearance + pre_pick) / 2.0, 3)
    current = sched.get("slope_transition_height")

    needs = current is None or not (pre_pick < float(current) < clearance)
    if not needs:
        results.append(("fix slope_transition_height", True, f"already valid ({current}); no change"))
        return results

    if no_save:
        results.append(("fix slope_transition_height", True,
                        f"WOULD set {current} -> {midpoint} (--no-save: not written)"))
        return results

    raw.setdefault("scheduler", {})["slope_transition_height"] = midpoint
    _save_raw(raw)
    results.append(("fix slope_transition_height", True, f"set {current} -> {midpoint}"))
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

      * F_CONVEYOR_TO_ROBOT (_T_X, _T_Y, _THETA_RAD) — from `test_vision_only`
        board readings vs hand-measured robot position.
      * robot_movement_delay_s / nominal_xy_speed / nominal_z_speed — from the
        `evaluate` scenario (gate on pos_EE convergence, not argument_time).
      * conveyor_position_scale_mm — from measured belt travel over a known move.

    Not yet enabled — see doc/ai_context.md for the calibration roadmap.
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
    settings, settings_error = _build_settings()
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
