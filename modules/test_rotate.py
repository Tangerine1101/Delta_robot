"""Siemens suction-rotation axis probe.

Boards land in the bin at wrong, per-object, random-looking orientations. The
software side (vision heading -> R-frame radians -> wire degrees) is now a
single audited chain, so what remains unknowable from code is the HARDWARE
side. This probe answers those questions with the cup free (no board, the
Omron arm is never moved):

1. **Remap + reachability** — command a sweep of R-frame angles and check the
   ``rotate_current`` feedback settles on each target (the +180 wire shift and
   the PLC's hardcoded [0, 360) limit agree with the PC's convention).
2. **Axis speed** — settle time per move => implied deg/s. If the axis is slow,
   the home-to-0 may not finish before the grip and the post-grip rotate may
   not finish before the release (both produce random orientation errors).
3. **Physical direction** — the feedback alone cannot reveal whether the axis
   turns CCW (seen from above) for a positive R-frame command; mark the cup and
   WATCH it during the sweep. If +90 turns the cup clockwise from above, set
   ``scheduler.rotate_sign: -1``.
4. **Retrigger semantics** — DB1 has no handshake and both production rotate
   commands are CommandID=7. If the ST program edge-triggers on a CommandID
   *change*, a second consecutive cmd-7 is silently dropped (production's
   post-grip rotate would only work when an adaptive change_speed happened to
   interleave). The probe sends back-to-back cmd-7 pairs with and without an
   interleaved no-op change_speed and compares.

Usage:
    python3 -m modules.test_rotate                     # full sequence
    python3 -m modules.test_rotate --settle-tol-deg 2 --timeout-s 8
"""
from __future__ import annotations

import argparse
import math
import time

from modules.EthernetCom import (
    COMMAND_ID,
    SiemensGateway,
    load_config,
    robot_rad_to_wire_deg,
    wire_deg_to_robot_rad,
)

_POLL_S = 0.05  # ~20 Hz feedback sampling


def _read_angle_deg(siemens: SiemensGateway) -> float | None:
    """Current cup angle in R-frame degrees [-180, 180), or None on error."""
    status = siemens.get_status()
    if status is None:
        return None
    raw = status.get("rotate_current")
    if raw is None:
        return None
    return math.degrees(wire_deg_to_robot_rad(float(raw)))


def _angle_err_deg(a: float, b: float) -> float:
    """Shortest signed distance a-b in degrees."""
    return (a - b + 180.0) % 360.0 - 180.0


def _send_rotate(siemens: SiemensGateway, target_deg: float) -> bool:
    """Dispatch rotate_absolute for an R-frame degree target (wire-converted)."""
    pkg = {
        "CommandID": COMMAND_ID["rotate_absolute"],
        "rotate": robot_rad_to_wire_deg(math.radians(target_deg)),
        "speed": 0.0,
    }
    return siemens.send_package(pkg) is not None


def _send_speed_noop(siemens: SiemensGateway) -> bool:
    """Interleave a change_speed that re-sends the measured belt speed (no-op)."""
    status = siemens.get_status()
    speed = 0.0
    if status is not None and status.get("speed_current") is not None:
        speed = float(status["speed_current"])
    pkg = {
        "CommandID": COMMAND_ID["change_speed"],
        "rotate": 0.0,
        "speed": speed,
    }
    return siemens.send_package(pkg) is not None


def _wait_settle(
    siemens: SiemensGateway,
    target_deg: float,
    tol_deg: float,
    timeout_s: float,
    stable_s: float = 0.3,
) -> dict[str, float | bool | None]:
    """Poll feedback until it holds within tol of target for ``stable_s``.

    Returns move stats: settled flag, settle time (first in-tolerance instant),
    final angle, peak |error| overshoot past the target, and the total angular
    path travelled (sum of |deltas| — used for the implied axis speed).
    """
    t0 = time.monotonic()
    first_in_tol: float | None = None
    last = _read_angle_deg(siemens)
    path_deg = 0.0
    final = last
    while time.monotonic() - t0 < timeout_s:
        angle = _read_angle_deg(siemens)
        if angle is not None:
            if last is not None:
                path_deg += abs(_angle_err_deg(angle, last))
            last = angle
            final = angle
            if abs(_angle_err_deg(angle, target_deg)) <= tol_deg:
                if first_in_tol is None:
                    first_in_tol = time.monotonic() - t0
                elif (time.monotonic() - t0) - first_in_tol >= stable_s:
                    return {
                        "settled": True,
                        "settle_s": first_in_tol,
                        "final_deg": final,
                        "path_deg": path_deg,
                    }
            else:
                first_in_tol = None
        time.sleep(_POLL_S)
    return {
        "settled": False,
        "settle_s": None,
        "final_deg": final,
        "path_deg": path_deg,
    }


def run(settle_tol_deg: float, timeout_s: float) -> int:
    config = load_config()
    siemens = SiemensGateway(
        getattr(config, "siemens_ip", None),
        getattr(config, "siemens_port", None),
    )
    if not siemens.connect():
        print("[ROTATE-PROBE] Siemens connect failed.")
        return 1

    speeds: list[float] = []
    failures: list[str] = []
    try:
        start = _read_angle_deg(siemens)
        print(f"[ROTATE-PROBE] start angle = {start if start is None else round(start, 2)} deg (R-frame)")
        print(
            "[ROTATE-PROBE] MARK THE CUP and watch it: every command below is an\n"
            "               R-frame target — positive should turn the cup CCW seen\n"
            "               from above. If it turns CW instead, set scheduler.rotate_sign: -1"
        )

        # --- 1+2+3: sweep — remap check, axis speed, visual direction --------
        print("\n[ROTATE-PROBE] Sweep (R-frame deg): 0, +90, -90, +170, -170, 0")
        for target in (0.0, 90.0, -90.0, 170.0, -170.0, 0.0):
            if not _send_rotate(siemens, target):
                failures.append(f"dispatch to {target:+.0f} failed")
                continue
            move = _wait_settle(siemens, target, settle_tol_deg, timeout_s)
            settled = bool(move["settled"])
            final = move["final_deg"]
            path = float(move["path_deg"] or 0.0)
            settle_s = move["settle_s"]
            speed = (path / settle_s) if (settled and settle_s and path > 1.0) else None
            if speed is not None:
                speeds.append(speed)
            print(
                f"  target {target:+7.1f}  settled={str(settled):<5} "
                f"final={final if final is None else round(final, 2)!s:>8} "
                f"settle_s={settle_s if settle_s is None else round(settle_s, 3)!s:>7} "
                f"path={path:6.1f} deg "
                f"speed={'-' if speed is None else f'{speed:6.1f} deg/s'}"
            )
            if not settled:
                failures.append(f"no settle at {target:+.0f} (final {final})")

        # --- 4: retrigger — consecutive cmd-7 vs interleaved cmd-8 -----------
        print("\n[ROTATE-PROBE] Retrigger check (edge-trigger on CommandID change?)")
        retrigger_plain = None
        retrigger_noop = None

        # Pair A: 7 -> 7, no other write in between.
        if _send_rotate(siemens, 90.0):
            _wait_settle(siemens, 90.0, settle_tol_deg, timeout_s)
            if _send_rotate(siemens, -90.0):
                move = _wait_settle(siemens, -90.0, settle_tol_deg, timeout_s)
                retrigger_plain = bool(move["settled"])
                print(f"  cmd7 -> cmd7 (no interleave):   second rotate executed = {retrigger_plain}")

        # Pair B: 7 -> (no-op 8) -> 7.
        if _send_rotate(siemens, 90.0):
            _wait_settle(siemens, 90.0, settle_tol_deg, timeout_s)
            _send_speed_noop(siemens)
            if _send_rotate(siemens, -90.0):
                move = _wait_settle(siemens, -90.0, settle_tol_deg, timeout_s)
                retrigger_noop = bool(move["settled"])
                print(f"  cmd7 -> cmd8 no-op -> cmd7:     second rotate executed = {retrigger_noop}")

        # Park back at 0 (production's grip angle).
        _send_rotate(siemens, 0.0)
        _wait_settle(siemens, 0.0, settle_tol_deg, timeout_s)

        # --- summary ----------------------------------------------------------
        print("\n[CONFIG-SUGGEST]")
        if speeds:
            axis_speed = sum(speeds) / len(speeds)
            print(f"  measured axis speed ~ {axis_speed:.1f} deg/s "
                  f"(180 deg swing ~ {180.0 / axis_speed:.2f} s)")
            print("  -> compare with the goto flight time (home-to-0 must finish before"
                  " the grip) and the pick transfer time (target rotate must finish"
                  " before the release); see [ROTATE] logs in production.")
        if retrigger_plain is False and retrigger_noop is True:
            print("  ST is EDGE-TRIGGERED on CommandID change: consecutive cmd-7s are"
                  " dropped. Production needs an interleaved no-op (or an ST patch)"
                  " between home-to-0 and the post-grip rotate.")
        elif retrigger_plain is True:
            print("  consecutive cmd-7s re-trigger correctly: no ST change needed.")
        elif retrigger_plain is None:
            print("  retrigger check inconclusive (dispatch failure).")
        else:
            print(f"  retrigger: plain={retrigger_plain} noop={retrigger_noop} —"
                  " inspect the ST program's CommandID latch.")
        print("  rotate_sign: set from the VISUAL check above"
              " (+90 turned CCW from above => 1, CW => -1).")
        if failures:
            print("\n[ROTATE-PROBE] FAILURES:")
            for item in failures:
                print(f"  - {item}")
            return 1
        return 0
    finally:
        try:
            siemens.disconnect()
        except Exception:
            pass


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--settle-tol-deg", type=float, default=2.0,
                        help="Feedback tolerance to count a move as settled")
    parser.add_argument("--timeout-s", type=float, default=8.0,
                        help="Max wait per move before declaring no-settle")
    args = parser.parse_args(argv)
    return run(args.settle_tol_deg, args.timeout_s)


if __name__ == "__main__":
    raise SystemExit(main())
