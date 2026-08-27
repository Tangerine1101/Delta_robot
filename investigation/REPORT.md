# Pick-accuracy investigation — trajectory generator and position control

**Date**: 2026-08-24 · **Scope**: `modules/scheduler.py` trajectory generation, pick-time
prediction, and the positional pick gate, cross-checked against the sibling simulation
sandbox `../python for scheduling`.
**Deliverable**: findings only. No file in `modules/`, `main.py`, `doc/` or `config.json`
was modified. Everything new lives in this directory.

---

## 0. Summary

The gate architecture is **not** broken. With the planner's model equal to the plant and a
correct latency budget, the shipped production loop lands within 2–3 mm at every belt speed
up to 150 mm/s (§4, E1). Nothing in the geometry, the coordinate transforms or the waypoint
templates diverges from the sandbox by more than floating-point noise (§2).

What is broken is the **latency budget the gate leads by**, and the fact that nothing in the
system can currently measure it correctly:

* the gate leads by `robot_movement_delay_s + ethernet_delay_s + sampling` and by **none** of
  the park→contact descent (§3, F5);
* the repository contains **three mutually inconsistent models of that same descent**, and
  the PLC's own documentation agrees with none of the two the scheduler uses (F3);
* the `[GATE]` log that exists specifically to calibrate this subtracts the wrong one of
  them, with a speed-dependent bias of −0.10 to −0.23 s (F4) — so the one instrument
  pointed at the problem reads wrong;
* the resulting contact error is a **pure gain**: `error_mm = v_belt × Δt`, measured slope
  exactly equal to belt speed (F5). One scalar, tuned blind, explains both "it works at the
  current speed" and "it falls apart when the speed changes".

A second, independent mechanism explains the specific `>120 mm/s` threshold: at 120 mm/s the
**plannable window closes** — a board must be committed before it enters the workspace at
all (§5, F13) — and at the same time the un-parked lateral offset at the grab instant grows
to 20–40 mm under any motion-model error (F8).

Findings are ordered by how much they move pick accuracy.

---

## 1. Method

Three independent instruments, all read-only with respect to both repositories.

| File | What it does |
|---|---|
| `test_algorithm_parity.py` | Runs the same input through both repositories' implementation of each shared algorithm and reports agreement or divergence. 17 checks. |
| `test_gate_logic.py` | Drives single shipped functions (`_object_pick_gate_status`, `_predict_realtime_pick_position`, `_adaptive_belt_speed`, `_descent_time_s`) with hand-built state and asserts behaviour at reachable boundaries. 11 checks. |
| `virtual_cell.py` + `run_production_sim.py` + `experiments.py` | A plant — belt, delta arm, boards, camera, wire — that the **real** `_run_realtime_pick_loop` drives end to end, with ground-truth contact scoring. |

The third is the important one. `open-issues.md` **L7**/**L8** record that `production` has
no offline dry-run and that every experiment therefore needs the physical cell. The harness
closes that: the control side is entirely shipped code — `_run_realtime_pick_loop`, the
perception thread, `RealtimePickExecutor`, `_object_pick_gate_status`, `_build_realtime_pick_plan`,
the adaptive speed law — and only the hardware is simulated. It runs in wall-clock time with
the real thread structure and the real locks, so timing and concurrency behave as they do on
the rig.

Plant fidelity, in the order it matters:

* **State 10.** `doc/PLC_Program_description/MC_inter_curve_vel.md` specifies that on a
  stationary start the interpolator does *not* begin at the arm's pose: it linearly ramps the
  joint setpoints from the measured servo position to the IK solution of `Pos[0]` over exactly
  20 scans (80 ms), then runs `Pos[0] → Pos[1]`. For the pick packet `Pos[0]` **is** the
  contact point, so this bridge is the whole park→board descent. The plant models it that
  way, with a separate `servo_lag_s` for the drive's inability to track a 200 mm/s velocity
  step at both ends of an 80 ms ramp.
* **Two-pass segment scheduling.** The plant caps each corner at the speed from which it can
  still stop by the next waypoint, then at the speed it can reach. That is what a machine
  must do, and it is the pass the repo's timing model omits (F1).
* **Model vs plant split**, borrowed from the sandbox: the planner's beliefs
  (`config.json > scheduler.interpolator`) and the arm's behaviour are independently
  settable, which is what makes C5 measurable.
* Belt ramps at `belt_accel_mm_s2`; every dispatch and status read costs a round trip; the
  camera stamps detections with a backdated capture time so the repo's
  `BeltPositionTracker.position_at` path is genuinely exercised.

**Where the harness is optimistic — read this before trusting a number.** The virtual arm
follows its commanded setpoint exactly. It has no torque limit and no tracking error, so it
executes State 10 faithfully however far it is asked to move in 80 ms. Every finding below
that involves the arm being *not where the planner thinks it is* (F8, F11) is therefore
**under**-stated here relative to the real cell, not over-stated.

Reproduce:

```bash
python3 -m investigation.test_algorithm_parity     # ~0.1 s
python3 -m investigation.test_gate_logic           # ~0.1 s
python3 -m investigation.experiments               # ~12 min wall clock
python3 -m investigation.run_production_sim --belt 120 --duration 45 --verbose
```

Raw output is kept in `results-parity.txt`, `results-gate-logic.txt`,
`results-experiments.txt`.

---

## 2. Repository parity — what agrees

Everything geometric is identical. These were checked, not assumed:

| Algorithm | Robot repo | Sandbox | Result |
|---|---|---|---|
| C-frame → R-frame transform | `ConveyorFrame.to_robot` | `ConveyorFrame.to_robot` | identical to 1e-6 mm |
| R-frame → C-frame inverse | `to_conveyor` | `to_conveyor` | identical |
| belt-flow unit vector | `u_hat` | `belt_direction_robot()` | identical to 1e-9 |
| single-segment velocity profile | `_segment_profile_time` | `segment_time` | identical to <1e-9 s over a 128-point grid |
| corner blend, geometric term | `_corner_v_end` | `corner_velocity` | identical |
| 7-point **goto** template | `_build_goto_geometry` | `seven_point_trajectory("goto")` | identical waypoint for waypoint |
| 7-point **pick** template | `_build_pick_geometry` | `seven_point_trajectory("pick")` | identical waypoint for waypoint |

Two divergences, both structural, both in the robot repo's favour to know about:

**F1 — the robot's trajectory timing model has no backward pass.**
`kinetic_control.segment_schedule` runs two passes: backward, capping every corner at the
speed from which the arm can still stop by the following waypoint; then forward, capping at
what it can accelerate to. `_trajectory_total_time` runs only the forward pass. On the pick
phase the final segment is a 5 mm descent into the bin; the forward-only pass enters it at
**261 mm/s**, which needs **34 mm** of stopping distance, and `_segment_profile_time` then
bills a full `v/d_max` deceleration that does not fit inside the segment.

| phase | robot model | two-pass, same waypoints | difference |
|---|---|---|---|
| goto | 0.9868 s | 0.9128 s | **+0.0740 s** over-estimate |
| pick | 1.0805 s | 0.9832 s | **+0.0973 s** over-estimate |

≈ 0.17 s per cycle. The sign is *conservative* for the pick gate (the arm arrives earlier
than predicted) and *pessimistic* for feasibility and throughput. The sandbox recorded the
same bug shape in `known-issues.md` §7 and notes it is invisible to a pick-error metric,
because planner and plant commit it identically — which is exactly why it needs a parity
check rather than a run.

**F2 — the model omits the segment from the arm's pose to `Pos[0]`.**
`_trajectory_total_time`'s docstring justifies this: "the arm's pre-trajectory position is
bridged to Pos[0] by that same soft-start". For the goto phase that bridge is a 5 mm lift and
the claim holds. For the **pick** phase `Pos[0]` is the contact point 16 mm below the park,
so `modeled_pick_s` — which `[CONFIG-SUGGEST]` uses to back out fixed overhead — understates
the phase by the entire descent (0.3098 s under the S-curve model).

---

## 3. The descent: three models, none of them agreeing

**F3 — the same 16 mm park→contact move is described three different ways in the code.**

| Where | Value | Model |
|---|---|---|
| `trajectory_pick[0].time_s`, via `_build_pick_timing` → `_segment_duration` | **0.080 s** | `max(0.08, Δz / nominal_z_speed)` |
| `plan.descend_time_s`, via `_contact_position` → `_descent_time_s` | **0.390 s** at rest, **0.625 s** at belt 120 | S-curve segment + a soft start + a belt slant |
| `MC_inter_curve_vel.md` State 10, what the PLC does | **0.080 s** | 20-scan linear setpoint ramp, from and to a full stop |

The three sit inside the *same* `PickPlan`. `basis-theory.md` §4.5 documents the middle one;
`_trajectory_total_time`'s own docstring documents the third. They contradict each other, and
the middle one — the one the oblique descent and the `[GATE]` calibration are built on — is
the one the PLC documentation does not support.

For scale: State 10 asks the cup to cover 16 mm in 80 ms, i.e. a mean **200 mm/s** with a
velocity step at each end. Whatever the drive actually needs beyond 80 ms is real dead time
that nothing in the pipeline leads by.

**F4 — `_descent_time_s` folds the belt slant in even when the oblique descent is off, and
that corrupts the C2 calibration.**

`_contact_position` computes `t_d = _descent_time_s(settings, v_belt)` *before* it checks
`oblique_descent_enabled`. With the flag off — the production default — the stored
`descend_time_s` therefore describes a slanted descent the arm never performs:

| belt | reported `t_d` | vertical truth | bias |
|---|---|---|---|
| 0 mm/s | 0.3898 s | 0.3898 s | 0 |
| 60 mm/s | 0.4924 s | 0.3898 s | −0.1025 s |
| 100 mm/s | 0.5828 s | 0.3898 s | −0.1930 s |
| 120 mm/s | 0.6247 s | 0.3898 s | −0.2348 s |

That value is published per pick as `[GATE] t_d_model_s`, and the documented procedure
(`open-issues.md` C2, `basis-programming.md` §8.1) is
`robot_movement_delay_s = dispatch_to_contact_s − t_d_model_s`. Subtracting an over-stated,
speed-dependent `t_d` **under-estimates the very latency the gate lead is built from**, by an
amount that changes with belt speed. Combined with F3 this is a sufficient explanation for
"the numbers in `config.json` are not trustworthy": the instrument built to produce them
reads wrong, and reads differently at every speed.

---

## 4. The pick gate — measured behaviour

**F5 — the contact error is a pure gain in the latency mismatch, and the gate leads by none
of the descent.**

The gate fires at `u_pick − v·T`, with `T = robot_movement_delay_s + ethernet_delay_s +
poll/2 + tick/2 = 0.2235 s`. The physical chain from that instant to contact is
`wire + PLC scan + State 10 + servo lag`. Sweeping the plant's true dispatch→motion latency
at 120 mm/s, everything else fixed:

```
  case                               grab  hit   rate     mean   median      min      max
  E2 belt 120, true delay 0.020s       11   11   100%    -8.18    -8.52   -11.23    -4.33
  E2 belt 120, true delay 0.060s       11   11   100%    -1.79    -1.62    -4.39    -0.11
  E2 belt 120, true delay 0.106s       11   11   100%    +2.82    +2.84    -0.40    +5.88
  E2 belt 120, true delay 0.160s       11   10    91%    +9.36    +9.27    +4.43   +13.56
```

(positive = the board had already passed the cup; negative = the cup got there first.)
The slope between the last two rows is 6.54 mm per 0.054 s = **121 mm/s**, i.e. exactly the
belt speed. The relationship is `error = v_belt × Δt` with no other term.

And the descent specifically, holding the dispatch latency at a plausible value and varying
only how long the drive needs past State 10's nominal 80 ms:

```
  E3 belt 120, servo lag 0.00s         11   11   100%    +2.52    +2.69    -1.68    +5.74
  E3 belt 120, servo lag 0.06s         11   10    91%   +10.30   +10.69    +7.31   +14.17
  E3 belt 120, servo lag 0.12s         11    0     0%   +16.84   +17.03   +13.17   +22.28
```

120 ms of descent lag is enough to miss **every** board on a 25.4 mm part. Nothing leads by
any of it.

For contrast, the baseline with the model matched to the plant and the latency budget
correct — this is the floor imposed by the 50 ms gate poll, the 25 ms perception tick and
wire staleness:

```
  E1 matched model, belt 60            11   11   100%    +1.92    +1.80    +0.40    +3.46
  E1 matched model, belt 90            12   12   100%    +2.19    +2.27    -0.04    +5.36
  E1 matched model, belt 120           11   11   100%    +2.19    +1.74    -0.87    +5.12
  E1 matched model, belt 150           11   11   100%    +3.18    +2.45    -1.44    +7.94
```

**The architecture is sound. The parameters are not.**

**F6 — the gate is one-sided.** `_object_pick_gate_status` returns
`reached = u_now >= threshold` and nothing else. A board 20 mm, 60 mm or 200 mm past the park
point opens it identically; so does one 50 mm past `u_max`, outside the reachable window
entirely. Nothing between the gate and the pick dispatch re-checks how far past the park
point the board has gone. Every millisecond of arm lateness therefore converts one-for-one
into contact error, with no bound and no abort.

**F7 — every accepted plan is parked with exactly 80 ms of spare time.**
`_predict_pick_position` converges to `t_pick = t_arrive + interp_soft_start_s`, and
`intercept_lead_time_s = 0.8 s` is a `max()` that never binds because the modelled goto flight
from a bin is 1.17 s. Measured margin across the whole operating range:

```
   belt   u_now   park margin (s)   u_pick
     60     150         +0.0799    228.4
     60     260         +0.0797    349.4
     90     200         +0.0792    330.4
    120     150         +0.0787    321.6
```

`basis-theory.md` §4.3 describes the arm parking downstream and waiting for the board. It
does not wait; it arrives 80 ms early, every time, at every speed. The sandbox measured the
identical failure of the identical lever and wrote it up as `known-issues.md` §2 — the two
repositories have converged on the same latent bug independently.

Combined with F6, the consequence is sharp: **the system has an 80 ms tolerance for motion-model
error, and beyond that the error goes straight into the pick.**

**F8 — `pick_arrival_tolerance_max_mm = 50 mm` is a latency knob, not a precision knob.**

Sampled at the instant the grab command reaches the PLC:

```
  case                               grab  hit   rate  park off mean      max  past cup mean      max  err mean      max
  E8 belt  60, plant v_max 300         12   12   100%     +0.00    +0.00    -10.02    -7.92     +1.38    +3.51
  E8 belt  60, plant v_max 180         10    9    90%     +8.43   +14.56     -5.62    +2.02     +5.72   +13.37
  E8 belt 120, plant v_max 300         11   11   100%     +0.00    +0.00    -19.06   -14.49     +3.16    +5.83
  E8 belt 120, plant v_max 180         11   11   100%    +20.57   +38.55    -17.47   -11.47     +4.81   +11.46
  E8 belt 150, plant v_max 300         11   11   100%     +0.00    +0.00    -22.67   -14.47     +4.55    +8.54
  E8 belt 150, plant v_max 180         11   11   100%    +21.86   +34.31    -22.66   -15.16     +4.63    +9.07
```

With a well-modelled arm the cup is exactly at the contact XY when the grab fires. With a
40 %-slow arm it is **20 mm away on average and 38 mm at worst**, and State 10 then has 80 ms
to cover that laterally *and* descend 16 mm — a commanded cup speed around 660 mm/s from a
standstill. The virtual arm does it perfectly, which is why the measured error stays small;
a real drive cannot, and the residual is precisely the erratic, speed-dependent inaccuracy
being investigated. **This is the finding the harness under-states most.**

It does not follow that the band should be tightened. Doing so blind is worse:

```
  E5 belt 120, tolerance 15->50 mm     11   11   100%    +3.98    +4.30    +0.54    +6.00
  E5 belt 120, tolerance  5->5  mm      9    1    11%   +31.66   +31.83    +1.49   +49.56
```

At 5 mm the executor blocks on convergence while the board sails past, and because the gate
is one-sided (F6) the grab then fires into empty belt. `open-issues.md` **G6** calls the
widening "a symptom treatment"; the measurement says it is a symptom treatment that works,
and the real lever is somewhere else — either the model/plant gap (F1, C5) or an arm-arrival
check that does not serialise ahead of the gate.

**F9 — the belt-position history is timestamped one round trip early.**
`ConveyorSpeedSource.sample(now)` takes `now` from the caller, then performs a *blocking*
`request_status()`, then stores the returned position against that pre-request `now`. The
`(t, p)` ring buffer therefore represents `p(t + RTT)`, and `position_at(t_capture)` — the
camera-latency backdating of `basis-theory.md` §4.2 — over-reads by `v · RTT`. The detection
anchor lands that far downstream, `u_now` reads low, and the gate fires late by one round
trip: roughly 1–2.5 mm at 120 mm/s. Small, but it has the same sign as F5 and it is a
systematic bias inside the one mechanism built to remove a systematic bias.

**F10 — the adaptive controller commits a speed change during the descent.**
`execute()` sets `state.gate_critical = False` and calls `_commit_adaptive_speed` immediately
after dispatching the pick packet — i.e. at the start of the descent — and the pick-phase
wait loop calls it again every iteration. `basis-theory.md` §6.5 licenses the single-term gate
offset of §4.4 on the promise that "the belt is steady whenever a gate fires, by construction,
not by hope"; the descent is the window where a belt ramp moves the board out from under the
cup, and it is explicitly excluded from `gate_critical`.

The sandbox measured this class of effect directly (`known-issues.md` §3): mean pick error
0.44 mm on a constant belt, **13 mm** with a speed law running, and confirmed the cause is the
setpoint moving while a pick is in flight — not the travel predictor. Its own target
architecture states the rule the robot repo breaks (`doc/basis-theory.md` §3.6): *"V\* is
applied only at the instant the arm becomes free, never while a pick is committed and in
progress."* This is also `open-issues.md` **L9** arriving early: the guarantee L9 says a
redesigned controller must preserve is already not held by the current one.

**F11 — a failed pick leaves the planner's idea of the arm's pose wrong.**
On failure the loop sets `scheduler.current_position = trajectory_goto[-1]` — the park point —
regardless of where the arm actually stopped. A goto that timed out did not reach it. The
next goto's `Pos[0]` is then computed from a fictitious pose and State 10 has 80 ms to bridge
the discrepancy.

---

## 5. Why the threshold sits at ~120 mm/s

**F13 — the plannable window closes there.** Sweeping the latest `u_now` at which
`_predict_realtime_pick_position` still accepts a plan, with the arm starting from the QFP
bin:

| belt speed | latest plannable `u_now` | relative to `u_min = 188` |
|---|---|---|
| 60 mm/s | 270 mm | +82 mm |
| 90 mm/s | 226 mm | +38 mm |
| **120 mm/s** | **180 mm** | **−8 mm** |
| 150 mm/s | 134 mm | −54 mm |
| 180 mm/s | 88 mm | −100 mm |

At and above 120 mm/s the cut-off crosses `u_min`: **a board must be committed before it
enters the workspace at all**, and at 180 mm/s while it is still under the camera
(`camera_window_uv = [0, 120]`). The main loop only builds a plan when it is idle, so whether
a given board is caught in time becomes a matter of where it happens to fall in the previous
pick's cycle. Boards that miss the window are never planned; boards that just make it are
planned at the edge of feasibility with the 80 ms margin of F7 and no more.

Note also that `_predict_pick_position` returns `None` as soon as the projected intercept
passes `u_max`, one call before `_predict_realtime_pick_position` could clamp to `u_max` —
so the clamp is effectively unreachable and boards are rejected rather than pinned to the
boundary. That is the safer of the two behaviours, but it means the cliff above is a hard
edge, not a graceful degradation.

This is a *feasibility* cliff, not an accuracy bug. It compounds with §3 and §4 because both
the latency error (F5) and the un-parked offset (F8) scale with belt speed at the same time.

---

## 6. Belt speed controller

**F14 — the deployed law is saturated at the floor for any realistic feed.**
With `belt_speed_headroom = 0.4`, `pick_cycle_s = 3`, `belt_density_length_mm = 0`
(⇒ `L_meas = u_max = 363`): `λ_nom = 0.1333 obj/s`, so `v = 48.4 / N`.

| N | v target |
|---|---|
| 0 | 100.0 mm/s (`v_cap`) |
| 1 | 48.4 mm/s |
| ≥2 | 30.0 mm/s (`v_min`) |

The regulated interior of the band is reached at `N = 1` and nowhere else; from two objects
on, the law is pinned to `v_min`. Since `N` counts everything from `u = 0` — the whole 363 mm
region, not the 175 mm workspace being regulated (**G3**) — two objects is the normal case.
Measured end to end on a crowded feeder:

```
  E7 crowded feed, adaptive OFF        19   19   100%    +2.40    +2.61    -0.64    +5.45
  E7 crowded feed, adaptive ON         15   15   100%    +0.54    +0.43    -0.35    +1.52
```

Better accuracy (because everything speed-proportional shrinks at 30 mm/s) and **21 % fewer
picks**. The `N = 0 → 1` step is 51.6 mm/s, which at `belt_speed_max_step_mm_s = 20` takes
three commits to walk, so the belt spends much of its time ramping.

**G1 confirmed numerically**: the startup seed `belt_speed_static_mm_s = 120` is above the
adaptive ceiling `v_cap = 100`, so the first commit always steps the belt down.

**Oblique descent (C3), measured.** It is the only compensation for the descent that exists
anywhere in the pipeline, and it is driven by the `t_d` of F3/F4:

```
  E6 belt 120, oblique OFF             11   11   100%    +1.92    +2.14    -0.97    +4.43
  E6 belt 120, oblique ON              11    1     9%   -64.81   -70.36   -76.13    +1.38
```

Turning it on shifts contact downstream by `v · t_d = 120 × 0.6247 = 75 mm`, while the
physical descent (State 10) is 80 ms ≈ 9.6 mm — an over-correction of about 65 mm, which is
exactly what the run shows. Disabling it was correct; re-enabling it requires fixing `t_d`
first, not calibrating `a_max`.

---

## 7. Scheduling policy — noted, not a cause

**F12.** Selection in `_build_realtime_pick_plan` is `(danger tier, then −u_now within the
tier, else cycle distance)` — greedy nearest with a downstream-priority tier at
`u_min + ⅔L = 304.7 mm`. There is no deadline in it. Both repositories' documents point at
EDD / Moore–Hodgson / Kise–Ibaraki–Mine (`../python for scheduling/doc/basis-theory.md` §2);
neither the robot repo nor the sandbox implements any of them yet. This is the declared NEAR
work and is recorded here only so the redesign starts from an accurate picture of the
baseline.

Also worth carrying into that redesign, from the sandbox's theory: the distinction between
**grab time** `g_j` (start → contact, the only thing the deadline constrains) and
**occupancy** `p_j` (start → board in bin, what releases the machine). The robot repo's
feasibility test `arm_arrival + command_delay ≤ pick_time` is a `g_j` test, which is correct;
but nothing anywhere accumulates `p_j`, so no horizon reasoning is possible today.

---

## 8. A note on the coordinate transform

Both repositories' C→R block

```
x = −sinθ·u + cosθ·v + Tx
y =  cosθ·u + sinθ·v + Ty
```

has determinant **−1**: it is a rotation composed with a reflection, not "a plain 2-D
rotation" as both documents describe it. Position mapping is unaffected and the two repos
agree exactly, so this is *not* a pick-position issue. It matters only for the angle chain:
a heading measured CCW in the C-frame comes out reversed in the R-frame, and
`ConveyorFrame.vision_heading_to_robot_rad` adds `+θ` on the assumption of a pure rotation.
The composite vision→C→R chain *is* a proper rotation (two reflections), so the code may well
be right — but the reasoning recorded in `basis-theory.md` §5.2 is not, and C1/C4 remain
open. Outside the scope of this investigation; flagged so it is not re-derived from the wrong
premise.

---

## 9. Suggested order of work

Not applied — listed because several of these block each other.

1. **Fix the measurement before measuring anything.** `[GATE] t_d_model_s` must report the
   time the arm actually spends between the park and the board (State 10, plus whatever the
   drive needs), not `_descent_time_s(settings, v_belt)`. Until then every `robot_movement_delay_s`
   derived from the log carries the F4 bias.
2. **Settle F3 on hardware.** One number decides it: `dispatch_to_contact_s` from the `[GATE]`
   log at belt = 0. If it is ≈ 0.08 + wire, State 10 is the descent and `_descent_time_s` is
   the wrong model. If it is ≈ 0.39, the S-curve model is right and the PLC documentation is
   stale.
3. **Collapse the gate lead to one measured scalar.** F5 shows the error is `v × Δt` in a
   single lumped delay. Three separately-guessed terms buy nothing and hide the bias; one
   number measured at two belt speeds identifies it exactly (the slope *is* the belt speed).
4. **Then** C5 (`speed_tuning`), keeping F1 in mind: the model already over-estimates by
   ~0.17 s per cycle from the missing backward pass, so a measured/modelled ratio is biased
   before the arm is even involved.
5. **Then** revisit `pick_arrival_tolerance_*` (F8) and `oblique_descent_enabled` (F4/E6).
   Neither can be tuned meaningfully while 1–3 are open.
6. Independently of the above: F10 (speed commit during the descent) and F11 (pose after a
   failed pick) are logic defects that do not need any calibration to fix.

---

## 10. Index of findings

| ID | Finding | Where | Impact on pick accuracy |
|---|---|---|---|
| F1 | timing model has no stop-distance backward pass | `_trajectory_total_time` | indirect (feasibility, C5 calibration) |
| F2 | pose→`Pos[0]` segment omitted; for the pick phase that is the descent | `_trajectory_total_time` | indirect (`[CONFIG-SUGGEST]`) |
| F3 | three inconsistent models of the park→contact descent | `_segment_duration` / `_descent_time_s` / State 10 | **root cause** |
| F4 | `_descent_time_s` applies the belt slant with the oblique descent off, biasing the C2 calibration by up to −0.23 s | `_contact_position` | **root cause (of the bad calibration)** |
| F5 | gate leads by no part of the descent; error is `v × Δt`, pure gain | `_object_pick_gate_status` | **direct, dominant** |
| F6 | gate is one-sided — no late bound, no abort | `_object_pick_gate_status` | **direct, unbounded** |
| F7 | every plan parked with exactly 80 ms margin; `intercept_lead_time_s` never binds | `_predict_pick_position` | **direct** |
| F8 | 50 mm arrival tolerance ⇒ 20–40 mm un-parked offset under model error | `_wait_for_arm_arrival` | **direct on hardware** (under-stated here) |
| F9 | belt-position history stamped one round trip early | `ConveyorSpeedSource.sample` | direct, ~1–2.5 mm |
| F10 | speed committed at the grip instant and during the descent | `execute` / `_wait_for_arm_arrival` | direct when adaptive is on |
| F11 | planner's arm pose wrong after a failed pick | `_run_realtime_pick_loop` | intermittent |
| F12 | selection policy is greedy-nearest, no deadline reasoning | `_build_realtime_pick_plan` | none (throughput) |
| F13 | plannable window closes at 120 mm/s | `_predict_realtime_pick_position` | **explains the reported threshold** |
| F14 | adaptive law saturated at `v_min` for N ≥ 2 | `_adaptive_belt_speed` | none (throughput) |

---

## 11. Delay budget (added after §10, in answer to "list every source of delay")

Generated by [`delay_budget.py`](delay_budget.py); raw output in
`results-delay-budget.txt`. Re-run with `python3 -m investigation.delay_budget`.

The park-and-wait is **not** a delay source — it is the corrector, and it works: it removes
the error in where the arm parked and in the belt-speed estimate over the whole flight, which
is why E1 measures 2–3 mm at any belt speed once the parameters are right. What it cannot
remove splits into four budgets:

| Budget | What it is | Can a lead fix it? |
|---|---|---|
| **A — perception** | error in the `u_now` the gate compares against | no — the gate fires on a *wrong number* |
| **B — actuation** | dead time from gate-true to cup-on-board | **yes**, this is exactly what `v·T` pays for |
| **C — readiness** | whether the arm was parked at all | no — feasibility failure, and F6 makes it unbounded |
| **D — grip** | the board keeps moving until the vacuum holds it | no |

Sign convention: positive = the cup lands **behind** the board. Every uncompensated term has
that sign, so they accumulate rather than cancel.

**A — perception (error in `u_now`)**

| id | source | duration | paid for by |
|---|---|---|---|
| A1 | exposure integration, half window | 5 ms | `_half_exposure_s` ✅ |
| A2 | USB/UVC transport + driver buffering | 33–67 ms | **nothing** |
| A3 | MJPEG decode + `to_ndarray(bgr24)` at 1920×1080 | 3–10 ms | **nothing** |
| A4 | YOLO inference + tracker + emit | 15–40 ms | backdated anchor ✅ |
| A5 | inference deque → `poll()` at the perception tick | 0–25 ms | backdated anchor ✅ |
| A6 | camera frame quantisation, 30 fps | 0–33 ms | per-frame timestamp ✅ |
| A7 | belt-position history stamped one round trip early (F9) | 10–25 ms | **nothing** |
| A8 | belt position stale at the gate read | 25–50 ms | `gate_sampling_latency_s` pays 12.5 ms — partial |

A2 and A3 are the same root cause: `_capture_loop` takes `t = time.monotonic()` **after**
`container.decode()` has yielded and `to_ndarray()` has run, so the stamp is
decode-completion, not capture. The backdating then subtracts only half the exposure. The
module docstring claims "the rest of the latency (decode + YOLO + poll) is absorbed
downstream" — decode is not: it happens *before* the stamp. PyAV exposes `frame.pts` /
`frame.time`, which the code ignores.

A9, not a delay but the reason A2/A3/A7 matter: past `camera_window_uv[1] = 120 mm` the board
is dead-reckoned on the encoder with no further camera fixes, so an anchor error is frozen in
for the remaining ~180 mm to the pick point. Nothing self-corrects it.

Also unverified: `config.json > vision.v4l2_controls.auto_exposure = 0`, while
`basis-programming.md` §5 and the code's own fallback both say manual mode is **1** on V4L2
(0 is not a defined value of that menu control). `_apply_v4l2_controls` runs `v4l2-ctl` with
`check=False, capture_output=True`, so a rejected control **fails silently**. If it is being
rejected, the camera is in auto-exposure, the exposure time varies with ambient light, and
both A1's constant and A2's frame period become variable. One command settles it:
`v4l2-ctl --device /dev/videoN --get-ctrl=auto_exposure,exposure_time_absolute` while the
pipeline is running.

**B — actuation (gate-true → contact). This is the budget the lead `v·0.2235 s` is meant to cover.**

| id | source | duration | paid for by |
|---|---|---|---|
| B1 | gate poll quantisation | 0–50 ms | `poll_interval_s / 2` ✅ |
| B2 | `state_lock` acquisition in the gate check | 0–2 ms | **nothing** |
| B3 | rotate refresh + flushing prints, gate-true → dispatch | 1–20 ms | **nothing** (logged as `gate_to_dispatch_s`) |
| B4 | `ipc_lock` wait behind an in-flight status read | 0–25 ms | **nothing** |
| B5 | IPC queue → worker process → pylogix write | 3–15 ms | `ethernet_delay_s = 0.016` ✅ |
| B6 | Omron scan picks up `bit_doing` (Rung 4) | 0–4 ms | lumped into `robot_movement_delay_s` |
| B7 | State 0 IK validation → State 10 entry | 0–4 ms | lumped into `robot_movement_delay_s` |
| B8 | **State 10 — the park→contact descent itself** | **80 ms** | **nothing** |
| B9 | servo following error during State 10 | 0–150 ms, unknown | **nothing** |

B4 deserves a note: one `request_status()` is **two** PLC reads in series —
`gateway.get_package()` over EtherNet/IP *plus* `siemens_gateway.get_status()` over snap7
(`main.py` `_worker`). `RealtimePickExecutor.dispatch` and `request_status` share `ipc_lock`,
so the pick dispatch can be held behind a full status round trip. The same round trip is why
the perception loop's real period is 35–50 ms rather than the 25 ms that
`gate_sampling_latency_s`'s hard-coded `0.0125` assumes (A8).

B8+B9 is the finding of §3: `robot_movement_delay_s = 0.170` is a lumped scalar standing in
for B6+B7 (≤8 ms) plus, in practice, whatever of B8/B9 the blind tuning happened to absorb.

**C — readiness (was the arm parked)**

| id | source | duration | note |
|---|---|---|---|
| C1 | arm-arrival poll quantisation | 0–50 ms | `_wait_for_arm_arrival` sleeps `poll_interval_s` |
| C2 | `robot_pose` staleness when arrival is declared | 25–50 ms | one perception period, already one round trip old |
| C3 | arrival declared inside `pick_arrival_tolerance` | *position*, up to 50 mm | measured 20–38 mm under a 40 % model error (E8) |
| C4 | goto flight-time model error | −170 ms known bias, plus C5 unknown | F1 over-estimates; `interp_v_max/a_max` unverified |
| C5 | plan-build cadence — planning happens only when idle | 50 ms to one full pick cycle (~3 s) | the mechanism behind the F13 cliff |

C3 is not a time and cannot be converted to one: it is a straight position offset that State
10 must absorb in 80 ms *on top of* the descent. C1+C2 say the arm is declared arrived up to
100 ms after it truly was — and because the gate is one-sided (F6), that 100 ms is
uncorrected error whenever the board was already at the threshold.

**D — grip**

| id | source | duration | note |
|---|---|---|---|
| D1 | vacuum establishment after contact | 0–100 ms, unknown | largely overlapped, see below |

The design here is right: `main_logic.md` Rung 19 sets `Pump_Ext := (ICV_Pos_E[step] = 1)`
with `Current_Step := 0` latched on `ICV_Start_NewTurn`, so suction is commanded at the
*start* of the pick trajectory — roughly 80 ms before contact — and the plumbing delay
overlaps B8. The residual is the vacuum's physical build time against a board that is still
moving, and there is no suction verification to measure it (`open-issues.md` **L6**).

**What nothing pays for**

Summing the open terms in budgets A and B (excluding C3, which is a position not a time):

```
  sum of open terms                                      127-379 ms

  belt mm/s               60         100         120         150
  low  (mm)              7.6        12.7        15.3        19.1
  high (mm)             22.7        37.9        45.4        56.8
```

A TQFP is 46 × 38 mm and a QFP 25.4 × 25.4 mm, so the cup has roughly ±12 mm of slack before
the grab misses entirely.

This is **not** the observed error, because `robot_movement_delay_s = 0.170` was tuned blind
and is absorbing an unknown part of it. That is precisely the problem: **one lumped scalar is
standing in for nine separate terms**, several of which are not constant. It can only ever be
right at the single belt speed it was tuned at, which is exactly the behaviour reported.

**Ranking, for where to spend effort**

1. **B8 + B9** — the descent. Largest single uncompensated block (80 ms known, up to 230 ms),
   and settled by one measurement (`[GATE] dispatch_to_contact_s` with the belt stopped).
2. **A2 + A3** — camera capture timestamp. 36–77 ms, entirely inside the PC, fixable without
   touching hardware, and permanent once the board leaves the camera (A9).
3. **C1 + C2 + C3** — arm readiness. Bounded in time (~100 ms) but unbounded in effect,
   because of F6.
4. **A7, A8, B4** — the status round trip appearing three separate times. One structural
   change (a decoupled belt-position reader, or timestamping after the read) closes all three.
5. **B9, D1** — unknown until instrumented; neither can be reasoned about further from here.
