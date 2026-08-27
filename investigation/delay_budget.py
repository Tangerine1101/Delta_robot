"""The complete delay budget between a board's true position and the cup landing.

Answers one question: **every second of dead time in the loop -- who pays for it?**

The positional pick gate is a corrector, not a delay source.  It removes the
error in *where* the arm parked and in the belt-speed estimate over the whole
flight, which is why the loop measures 2-3 mm at any belt speed once its
parameters are right (`investigation/experiments.py` E1).  What it cannot
correct splits into three budgets, and every term below belongs to exactly one:

  A  PERCEPTION   error in the `u_now` the gate compares against.
                  The gate fires when a *wrong number* crosses the threshold.
  B  ACTUATION    dead time from gate-true to the cup touching the board.
                  Must be paid for by the lead `v * T`; whatever is missing
                  from `T` lands the cup behind by `v * (missing time)`.
  C  READINESS    whether the arm was parked at all when the gate fired.
                  No lead can compensate this -- it is a feasibility failure,
                  and the one-sided gate turns it into unbounded error.
  D  GRIP         the board keeps moving between contact and being held.

Sign convention throughout: **positive = the cup lands BEHIND the board**
(the grab fired late).  Every uncompensated term below has that sign, which is
why they accumulate rather than cancel.

Run:  python3 -m investigation.delay_budget [--belt 60 120 150]
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import modules.scheduler as RS
from modules.EthernetCom import load_config

COMPENSATED = "compensated"
PARTIAL = "partial"
OPEN = "NOT compensated"


@dataclass(frozen=True)
class Term:
    """One source of delay in the chain."""

    ident: str
    budget: str
    name: str
    low_s: float
    high_s: float
    status: str
    by: str            # what pays for it, or "-" if nothing does
    where: str         # the code or document that establishes it
    measured: str      # "config" / "code" / "estimate" -- how the number was got
    how_to_measure: str = ""

    @property
    def mid_s(self) -> float:
        return 0.5 * (self.low_s + self.high_s)


def build_budget(settings) -> list[Term]:
    st = settings
    cfg = load_config()
    vision = getattr(cfg, "vision", {}) or {}
    controls = vision.get("v4l2_controls", {}) or {}
    exposure_s = float(controls.get("exposure_time_absolute", 0) or 0) * 100e-6
    fps = float((vision.get("capture", {}) or {}).get("fps", 30) or 30)
    frame_period = 1.0 / fps if fps > 0 else 0.0
    tick = 0.025                       # _realtime_perception_loop sleep
    poll = max(st.poll_interval_s, 0.02)

    return [
        # A -- perception: what the gate's `u_now` is actually describing
        Term("A1", "A", "exposure integration (half window)",
             exposure_s / 2, exposure_s / 2, COMPENSATED,
             "VisionImageProcessing._half_exposure_s",
             "image_processing.py: detect_ts = capture_ts - half_exposure", "config"),
        Term("A2", "A", "USB/UVC transport + driver buffering",
             frame_period, 2 * frame_period, OPEN, "-",
             "_capture_loop stamps t AFTER container.decode() yields the frame",
             "estimate",
             "compare frame.pts / frame.time from PyAV against time.monotonic() "
             "in _capture_loop; the gap is this term plus A3"),
        Term("A3", "A", "MJPEG decode + to_ndarray(bgr24) at 1920x1080",
             0.003, 0.010, OPEN, "-",
             "_capture_loop stamps t after to_ndarray, not before", "estimate",
             "time the to_ndarray call directly"),
        Term("A4", "A", "YOLO inference + tracker + emit",
             0.015, 0.040, COMPENSATED, "backdated anchor (position_at)",
             "_loop stamps the detection with frame_ts, not `now`", "estimate"),
        Term("A5", "A", "inference deque -> poll() at the perception tick",
             0.0, tick, COMPENSATED, "backdated anchor (position_at)",
             "PickScheduler.ingest_detections(position_at=...)", "code"),
        Term("A6", "A", "camera frame quantisation",
             0.0, frame_period, COMPENSATED, "per-frame timestamp",
             f"capture at {fps:.0f} fps", "config"),
        Term("A7", "A", "belt-position history stamped one round trip early",
             0.010, 0.025, OPEN, "-",
             "ConveyorSpeedSource.sample: `now` is read BEFORE the blocking "
             "request_status(), then stored against the returned position",
             "estimate",
             "log time.monotonic() either side of request_status() in "
             "ConveyorSpeedSource.sample for one run"),
        Term("A8", "A", "belt position stale at the gate read",
             tick, tick + 0.025, PARTIAL,
             "gate_sampling_latency_s pays 0.0125 s, i.e. half of a 25 ms tick only",
             "_realtime_perception_loop sleeps 25 ms; each tick also pays one "
             "request_status round trip, so the true period is longer",
             "code + estimate",
             "histogram the wall time between consecutive [SPEED] lines"),

        # B -- actuation: gate-true to contact
        Term("B1", "B", "gate poll quantisation",
             0.0, poll, COMPENSATED, "gate_sampling_latency_s (poll / 2)",
             "_wait_for_object_arrival sleeps status_poll_interval_s", "config"),
        Term("B2", "B", "state_lock acquisition inside the gate check",
             0.0, 0.002, OPEN, "-",
             "_object_pick_gate_status holds state_lock; the perception tick "
             "holds it across ingest + snapshot + density", "estimate"),
        Term("B3", "B", "rotate refresh + prints, gate-true -> dispatch",
             0.001, 0.020, OPEN, "-",
             "execute(): flushing prints sit between the gate and the dispatch",
             "estimate",
             "already logged: [GATE] gate_to_dispatch_s"),
        Term("B4", "B", "ipc_lock wait behind an in-flight status read",
             0.0, 0.025, OPEN, "-",
             "RealtimePickExecutor.dispatch and request_status share ipc_lock; "
             "one status read is TWO PLC reads in series (main.py _worker)",
             "code + estimate",
             "log the ipc_lock acquisition time in RealtimePickExecutor.dispatch"),
        Term("B5", "B", "IPC queue -> worker process -> pylogix tag write",
             0.003, 0.015, COMPENSATED,
             f"ethernet_delay_s = {st.ethernet_delay_s:.3f} s",
             "main.py dispatch() -> command_queue -> PLCGateway.send_package",
             "config vs estimate",
             "python3 -m modules.latency_probe --target siemens"),
        Term("B6", "B", "Omron scan picks up bit_doing (Rung 4, one scan)",
             0.0, 0.004, PARTIAL,
             f"lumped into robot_movement_delay_s = {st.robot_movement_delay_s:.3f} s",
             "main_logic.md Rung 4: processed in a single 4 ms scan", "document"),
        Term("B7", "B", "State 0 IK validation of Pos[0] -> State 10 entry",
             0.0, 0.004, PARTIAL,
             "lumped into robot_movement_delay_s",
             "MC_inter_curve_vel.md State 0", "document"),
        Term("B8", "B", "State 10: the park->contact descent itself",
             0.080, 0.080, OPEN, "-",
             "MC_inter_curve_vel.md State 10: 20 scans of linear setpoint ramp "
             "from the measured servo pose to Pos[0], which for the pick packet "
             "IS the contact point",
             "document",
             "[GATE] dispatch_to_contact_s with the belt STOPPED; subtract B5+B6"),
        Term("B9", "B", "servo following error during State 10",
             0.0, 0.150, OPEN, "-",
             "16 mm in 80 ms is a 200 mm/s velocity step at both ends of a "
             "linear joint ramp; the drive lags it by an unknown amount",
             "unknown",
             "same measurement as B8 -- it is whatever B8 comes out above 0.080 s"),

        # C -- readiness: was the arm even there
        Term("C1", "C", "arm-arrival poll quantisation",
             0.0, poll, OPEN, "-",
             "_wait_for_arm_arrival sleeps status_poll_interval_s", "config"),
        Term("C2", "C", "robot_pose staleness when arrival is declared",
             tick, tick + 0.025, OPEN, "-",
             "state.robot_pose is refreshed only by the perception tick, and is "
             "already one round trip old when written", "code + estimate"),
        Term("C3", "C", "arrival declared inside pick_arrival_tolerance",
             0.0, 0.0, OPEN, "-",
             "_arrival_tolerance_mm: 15 mm at v_min rising to 50 mm at v_max. "
             "Not a time -- a POSITION error of up to 50 mm that State 10 then "
             "has 80 ms to absorb on top of the descent (measured 20-38 mm under "
             "a 40% motion-model error, experiments.py E8)",
             "measured (E8)"),
        Term("C4", "C", "goto flight-time model error",
             -0.170, 0.0, OPEN, "-",
             "_trajectory_total_time over-estimates by ~0.17 s/cycle (no "
             "stop-distance backward pass, REPORT.md F1); interp_v_max/a_max "
             "have never been checked against the arm (open-issues C5), sign "
             "unknown and possibly larger",
             "measured (parity) + unknown",
             "CLI speed_tuning, but read REPORT.md F1 first -- the ratio is "
             "already biased before the arm is involved"),
        Term("C5", "C", "plan-build cadence (idle-only planning)",
             0.050, 3.0, OPEN, "-",
             "_run_realtime_pick_loop builds a plan only when no pick is in "
             "flight, then sleeps poll_interval_s. A board arriving mid-cycle "
             "waits up to a full pick_cycle_s to be considered at all",
             "code + config"),

        # D -- grip
        Term("D1", "D", "vacuum establishment after contact",
             0.0, 0.100, OPEN, "-",
             "Rung 19 sets Pump_Ext from ICV_Pos_E[0] at Current_Step = 0, i.e. "
             "when the trajectory starts -- so the pump is commanded ~80 ms "
             "BEFORE contact and the plumbing delay largely overlaps B8. The "
             "residual is the vacuum's physical build time against a moving "
             "board, and no suction verification exists (open-issues L6)",
             "document + unknown",
             "high-speed video of the grip, or an inline vacuum switch"),
    ]


BUDGET_TITLES = {
    "A": "A -- PERCEPTION: error in the u_now the gate compares against",
    "B": "B -- ACTUATION: gate-true to the cup touching the board",
    "C": "C -- READINESS: was the arm parked when the gate fired",
    "D": "D -- GRIP: the board keeps moving until it is held",
}


def report(belt_speeds: list[float]) -> None:
    settings = RS.SchedulerSettings.from_config(load_config())
    terms = build_budget(settings)

    gate_lead_s = (settings.robot_movement_delay_s + settings.ethernet_delay_s
                   + settings.poll_interval_s / 2.0 + 0.0125)

    print(__doc__.split("Run:")[0].rstrip())
    print()
    print(f"Gate lead actually applied: v * {gate_lead_s:.4f} s")
    print(f"  = robot_movement_delay_s {settings.robot_movement_delay_s:.3f}")
    print(f"  + ethernet_delay_s       {settings.ethernet_delay_s:.3f}")
    print(f"  + poll_interval_s / 2    {settings.poll_interval_s / 2.0:.4f}")
    print(f"  + perception tick / 2    0.0125")
    print()

    for budget in ("A", "B", "C", "D"):
        rows = [t for t in terms if t.budget == budget]
        print("=" * 100)
        print(BUDGET_TITLES[budget])
        print("=" * 100)
        print(f"  {'id':<4}{'source':<58}{'seconds':>14}  {'status':<16} how it was got")
        for t in rows:
            span = (f"{t.low_s * 1000:.0f}-{t.high_s * 1000:.0f} ms"
                    if t.low_s != t.high_s else f"{t.low_s * 1000:.0f} ms")
            print(f"  {t.ident:<4}{t.name:<58}{span:>14}  {t.status:<16} {t.measured}")
        print()
        for t in rows:
            print(f"  {t.ident}: {t.where}")
            if t.by != "-":
                print(f"      paid by: {t.by}")
            if t.how_to_measure:
                print(f"      measure: {t.how_to_measure}")
        print()

    # The number that matters: what nothing pays for.
    open_terms = [t for t in terms if t.status == OPEN and t.budget in ("A", "B")
                  and t.ident != "C3"]
    low = sum(t.low_s for t in open_terms)
    high = sum(t.high_s for t in open_terms)
    print("=" * 100)
    print("UNCOMPENSATED TOTAL (budgets A and B only -- the part that is a pure gain)")
    print("=" * 100)
    print(f"  {'':<4}{'sum of open terms':<58}{f'{low * 1000:.0f}-{high * 1000:.0f} ms':>14}")
    print()
    print("  Contact error from these alone, positive = cup lands behind the board:")
    header = "  " + "belt mm/s".ljust(14) + "".join(f"{s:>12.0f}" for s in belt_speeds)
    print(header)
    print("  " + "low  (mm)".ljust(14) + "".join(f"{low * s:>12.1f}" for s in belt_speeds))
    print("  " + "high (mm)".ljust(14) + "".join(f"{high * s:>12.1f}" for s in belt_speeds))
    print()
    print("  For scale: a TQFP board is 46 x 38 mm, a QFP 25.4 x 25.4 mm, so the")
    print("  suction cup has roughly +-12 mm of slack before the grab misses entirely.")
    print()
    print("  This total is NOT the observed error, because robot_movement_delay_s = "
          f"{settings.robot_movement_delay_s:.3f} s")
    print("  was tuned blind and is absorbing an unknown part of it. That is exactly the")
    print("  problem: one lumped scalar is standing in for nine separate terms, so it can")
    print("  only ever be right at the one belt speed it was tuned at.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Delay budget of the pick loop.")
    parser.add_argument("--belt", type=float, nargs="+", default=[60.0, 100.0, 120.0, 150.0])
    args = parser.parse_args()
    report(args.belt)


if __name__ == "__main__":
    main()
