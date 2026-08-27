"""The experiment set: what actually moves pick accuracy, and by how much.

Each experiment isolates one suspected cause and holds everything else fixed.
The control side is always the shipped production loop; only the plant and the
config values under test change.

    python3 -m investigation.experiments            # everything (~12 min)
    python3 -m investigation.experiments E2 E4      # selected

Reading the numbers: `along-belt error` is signed and measured in belt
coordinates.  **Positive means the board had already travelled past the cup when
the cup came down** -- the grab fired late.  Negative means the cup got there
first and the board had not arrived yet.
"""

from __future__ import annotations

import statistics
import sys
import time
from dataclasses import dataclass

from investigation.run_production_sim import RunResult, run_once
from investigation.virtual_cell import PlantProfile

SCRATCH = "/tmp/claude-1000/-home-tangerine-Share-Global-Share-Documents-Delta-robot/8c3ca6f7-dd71-48e7-b448-13f34897f19f/scratchpad"

DURATION = 30.0


def _row(result: RunResult) -> str:
    along = result.along_errors
    if along:
        body = (f"{statistics.fmean(along):+8.2f} {statistics.median(along):+8.2f} "
                f"{min(along):+8.2f} {max(along):+8.2f}")
    else:
        body = f"{'n/a':>8} {'n/a':>8} {'n/a':>8} {'n/a':>8}"
    warn = ",".join(f"{k}:{v}" for k, v in sorted(result.warnings.items())) or "-"
    return (f"  {result.label:<34} {result.contacts:>4} {result.hits:>4} "
            f"{result.hit_rate * 100:>5.0f}% {body}   {warn}")


HEADER = (f"  {'case':<34} {'grab':>4} {'hit':>4} {'rate':>6} "
          f"{'mean':>8} {'median':>8} {'min':>8} {'max':>8}   warnings")


def _report(title: str, note: str, results: list[RunResult]) -> None:
    print(f"\n{'=' * 108}\n{title}\n{note}\n{'=' * 108}")
    print(HEADER)
    for result in results:
        print(_row(result))
    sys.stdout.flush()


def E1_belt_speed_sweep() -> list[RunResult]:
    """Is the closed loop self-consistent when the model matches the plant?

    Model == plant, and the plant's true dispatch latency is set to the value
    the config claims, so every term the gate leads by is correct.  Whatever
    error survives here is irreducible: gate poll quantisation, perception tick,
    wire staleness.
    """
    out = []
    for belt in (60.0, 90.0, 120.0, 150.0):
        out.append(run_once(
            label=f"E1 matched model, belt {belt:.0f}",
            duration_s=DURATION, belt_static_mm_s=belt, adaptive=False,
            plant=PlantProfile(command_delay_s=0.186 - 0.08, servo_lag_s=0.0),
            feed_interval_s=2.5,
            log_path=f"{SCRATCH}/E1_{belt:.0f}.log",
        ))
    return out


def E2_gate_lead_mismatch() -> list[RunResult]:
    """How hard does an error in `robot_movement_delay_s` bite?

    The config claims dispatch->motion = 0.170 s and ethernet = 0.016 s, and the
    gate leads by their sum plus a sampling term.  The plant's TRUE latency is
    swept.  If the relationship is a pure gain `v * (assumed - true)`, then the
    single uncalibrated number C2 is the whole of the systematic error.
    """
    out = []
    for true_delay in (0.02, 0.06, 0.106, 0.16):
        out.append(run_once(
            label=f"E2 belt 120, true delay {true_delay:.3f}s",
            duration_s=DURATION, belt_static_mm_s=120.0, adaptive=False,
            plant=PlantProfile(command_delay_s=true_delay),
            feed_interval_s=2.5,
            log_path=f"{SCRATCH}/E2_{true_delay:.3f}.log",
        ))
    return out


def E3_state10_descent() -> list[RunResult]:
    """The park->contact descent that nothing leads by.

    `MC_inter_curve_vel.md` State 10 bridges the arm from wherever it is to
    `Pos[0]` -- for the pick phase, the contact point -- with a 20-scan linear
    setpoint ramp.  16 mm in 80 ms is a 200 mm/s velocity step at both ends, so
    the drive lags it.  `servo_lag_s` is that lag; the gate leads by none of it.
    """
    out = []
    for lag in (0.0, 0.06, 0.12):
        out.append(run_once(
            label=f"E3 belt 120, servo lag {lag:.2f}s",
            duration_s=DURATION, belt_static_mm_s=120.0, adaptive=False,
            plant=PlantProfile(command_delay_s=0.106, servo_lag_s=lag),
            feed_interval_s=2.5,
            log_path=f"{SCRATCH}/E3_{lag:.2f}.log",
        ))
    return out


def E4_motion_model_mismatch() -> list[RunResult]:
    """C5: the interpolator limits have never been checked against the arm.

    The planner believes v_max = 300 mm/s and picks a park point the arm can
    supposedly reach in time.  Detune the plant and the arm arrives after the
    board has already crossed the gate threshold; the gate is then open on
    arrival and the grab fires immediately, wherever the board happens to be.
    """
    out = []
    for belt in (60.0, 120.0):
        for v_max in (300.0, 220.0, 160.0):
            out.append(run_once(
                label=f"E4 belt {belt:.0f}, plant v_max {v_max:.0f}",
                duration_s=DURATION, belt_static_mm_s=belt, adaptive=False,
                plant=PlantProfile(v_max=v_max, a_max=600.0, d_max=600.0,
                                   command_delay_s=0.106),
                feed_interval_s=2.5,
                log_path=f"{SCRATCH}/E4_{belt:.0f}_{v_max:.0f}.log",
            ))
    return out


def E5_arrival_tolerance() -> list[RunResult]:
    """`pick_arrival_tolerance_max_mm` was widened to 50 mm to make picks land.

    With a slow plant the arm is genuinely late.  A 50 mm tolerance declares it
    parked while it is still 50 mm away, so the gate can fire mid-flight and
    State 10 then drags the cup diagonally onto the board in 80 ms.  A tight
    tolerance instead surfaces the lateness as a goto timeout.
    """
    out = []
    slow = PlantProfile(v_max=200.0, a_max=600.0, d_max=600.0, command_delay_s=0.106)
    for floor, ceiling in ((15.0, 50.0), (5.0, 5.0)):
        out.append(run_once(
            label=f"E5 belt 120, tolerance {floor:.0f}->{ceiling:.0f} mm",
            duration_s=DURATION, belt_static_mm_s=120.0, adaptive=False,
            plant=slow, feed_interval_s=2.5,
            arrival_tolerance_mm=floor,
            arrival_tolerance_max_mm=ceiling,
            log_path=f"{SCRATCH}/E5_{ceiling:.0f}.log",
        ))
    return out


def E6_oblique_descent() -> list[RunResult]:
    """Turning the oblique descent on is the only t_d compensation there is.

    It shifts the contact point downstream by `v * t_d`, but `t_d` is computed
    by `_descent_time_s`, which models the descent as its own S-curve segment
    (0.31 s) instead of the 80 ms State 10 bridge the PLC actually performs --
    and then folds a belt slant into that already-wrong number.
    """
    out = []
    for oblique in (False, True):
        out.append(run_once(
            label=f"E6 belt 120, oblique {'ON' if oblique else 'OFF'}",
            duration_s=DURATION, belt_static_mm_s=120.0, adaptive=False,
            oblique=oblique,
            plant=PlantProfile(command_delay_s=0.106),
            feed_interval_s=2.5,
            log_path=f"{SCRATCH}/E6_{oblique}.log",
        ))
    return out


def E7_adaptive_speed() -> list[RunResult]:
    """The adaptive controller commits a `change_speed` at the grip instant.

    `execute()` clears `gate_critical` and calls `_commit_adaptive_speed`
    immediately after dispatching the pick, i.e. at the start of the descent --
    the one window where a belt ramp moves the board out from under the cup.
    A crowded feeder makes the density law commit often.
    """
    out = []
    for adaptive in (False, True):
        out.append(run_once(
            label=f"E7 crowded feed, adaptive {'ON' if adaptive else 'OFF'}",
            duration_s=40.0, belt_static_mm_s=120.0, adaptive=adaptive,
            plant=PlantProfile(command_delay_s=0.106),
            feed_interval_s=1.2,
            log_path=f"{SCRATCH}/E7_{adaptive}.log",
        ))
    return out


EXPERIMENTS = {
    "E1": (E1_belt_speed_sweep, "E1 -- baseline: the loop with a perfectly known plant",
           "Model == plant, true latency == configured latency. The residual here is the\n"
           "floor set by gate poll (50 ms), perception tick (25 ms) and wire staleness."),
    "E2": (E2_gate_lead_mismatch, "E2 -- sensitivity to robot_movement_delay_s (open issue C2)",
           "Gate lead is fixed at the configured 0.170 + 0.016 + 0.0375 s; the plant's true\n"
           "dispatch->motion latency is swept. Expect a pure gain: error = v * (assumed - true)."),
    "E3": (E3_state10_descent, "E3 -- the State 10 park->contact bridge",
           "The gate leads by zero of it. Any part of the 16 mm drop that takes longer than\n"
           "the nominal 80 ms lands the cup behind the board by exactly v * that time."),
    "E4": (E4_motion_model_mismatch, "E4 -- uncalibrated interpolator limits (open issue C5)",
           "The planner sizes the park point with v_max = 300, a = 1000. A slower arm arrives\n"
           "after the board has passed the gate threshold, and the grab fires on arrival."),
    "E5": (E5_arrival_tolerance, "E5 -- pick_arrival_tolerance_max_mm = 50 mm (open issue G6)",
           "Does the widened arrival band hide arm lateness rather than fix it?"),
    "E6": (E6_oblique_descent, "E6 -- oblique_descent_enabled (open issue C3)",
           "The only compensation for the descent that exists, driven by a t_d the PLC\n"
           "documentation contradicts."),
    "E7": (E7_adaptive_speed, "E7 -- adaptive speed commits during the descent (L9)",
           "basis-theory 6.5 promises a steady belt whenever a gate fires; execute() commits\n"
           "a new setpoint immediately after the grab is dispatched."),
}


def main() -> None:
    wanted = [a.upper() for a in sys.argv[1:]] or list(EXPERIMENTS)
    t0 = time.monotonic()
    for key in wanted:
        if key not in EXPERIMENTS:
            print(f"unknown experiment {key}; known: {', '.join(EXPERIMENTS)}")
            continue
        fn, title, note = EXPERIMENTS[key]
        results = fn()
        if key == "E8":
            E8_report(results)
        else:
            _report(title, note, results)
    print(f"\ntotal wall time {time.monotonic() - t0:.0f}s")




def E8_arm_lateness() -> list[RunResult]:
    """Was the arm actually parked when the grab fired?

    Two things the gate cannot see are sampled at the instant the grab command
    reaches the PLC:

    * **park offset** -- how far the cup still is from the contact XY.  The
      executor declares arrival inside `pick_arrival_tolerance` (up to 50 mm at
      operating speed), and State 10 then has 80 ms to drag the cup over that
      gap *and* down onto the board.
    * **board past cup** -- how far the board has already travelled beyond the
      contact point.  The gate has no upper bound, so this is unbounded too.

    Swept over belt speed and over how badly the interpolator model overstates
    the arm (C5).
    """
    out = []
    for belt in (60.0, 120.0, 150.0):
        for v_max in (300.0, 180.0):
            out.append(run_once(
                label=f"E8 belt {belt:.0f}, plant v_max {v_max:.0f}",
                duration_s=DURATION, belt_static_mm_s=belt, adaptive=False,
                plant=PlantProfile(v_max=v_max, a_max=1000.0 if v_max > 250 else 600.0,
                                   d_max=1000.0 if v_max > 250 else 600.0,
                                   command_delay_s=0.106),
                feed_interval_s=2.5,
                log_path=f"{SCRATCH}/E8_{belt:.0f}_{v_max:.0f}.log",
            ))
    return out


def _row_lateness(result: RunResult) -> str:
    def stat(values):
        if not values:
            return f"{'n/a':>8} {'n/a':>8}"
        return f"{statistics.fmean(values):+8.2f} {max(values):+8.2f}"
    return (f"  {result.label:<34} {result.contacts:>4} {result.hits:>4} "
            f"{result.hit_rate * 100:>5.0f}%  "
            f"{stat(result.park_offsets_mm)}  {stat(result.gate_overshoots_mm)}  "
            f"{stat(result.along_errors)}")


def E8_report(results: list[RunResult]) -> None:
    print(f"\n{'=' * 118}")
    print("E8 -- was the arm parked, and had the board already gone past?")
    print("Sampled at the instant the grab command lands at the PLC.")
    print(f"{'=' * 118}")
    print(f"  {'case':<34} {'grab':>4} {'hit':>4} {'rate':>6}  "
          f"{'park off mean':>8} {'max':>8}  {'past cup mean':>8} {'max':>8}  "
          f"{'err mean':>8} {'max':>8}")
    for result in results:
        print(_row_lateness(result))


EXPERIMENTS["E8"] = (E8_arm_lateness, "E8 -- arm lateness and park offset at the grab instant",
                     "See the docstring; reported by E8_report below the standard table.")


if __name__ == "__main__":
    main()
