# Evaluation Plan — Delta Robot Sorting Cell (ILLUSTRATIVE / PLACEHOLDER DATA)

> ⚠️ **THIS IS A FILLED-IN COPY OF `doc/evaluate.md` WITH FABRICATED NUMBERS.**
> The values below were **reasoned in by hand, NOT physically measured**. They are internally
> consistent with the system parameters (`config.json`, design target ±0.5 mm, 0.05 mm caliper
> floor, belt range, 2.0 s pick cycle, mAP50-95 ≈ 0.983) so the document reads plausibly, but
> **none of it is real evidence**. Replace every table with actual measured data before using any
> figure in the thesis. The blank master remains in `doc/evaluate.md`.

This document is the test plan that feeds **Chapter 5 (Experiment and Evaluation)** of the thesis.
It defines the test cases from the micro level (individual subsystems and calibration) up to the
macro level (the full cell running end to end). The tables below are **populated with illustrative
data**; treat them as a worked example of the expected shape of the results.

> **Lab constraint.** This is a graduation project without a professional metrology lab. The finest
> linear instrument available is a **vernier caliper with 0.05 mm resolution**. All linear-accuracy
> claims are therefore bounded by this resolution: do **not** report or claim a physical positioning
> error finer than 0.05 mm. (A sub-0.05 mm figure may legitimately appear only in the *simulated*
> software-in-the-loop tests, where there is no physical measurement, see Group D.)

---

## 0. Instruments, conventions, and measurement uncertainty

| Quantity | Instrument / source | Resolution | Reported uncertainty |
|---|---|---|---|
| Linear position / offset | Vernier caliper | 0.05 mm | ±0.05 mm |
| Board / bin layout distances | Steel rule + caliper | 0.05 mm | ±0.05 mm |
| Orientation angle | Printed protractor or angle jig | ~1° | ±1° |
| Phase / cycle time | Software timestamps (`[ACCEPT]` log) | ~1 ms | software clock |
| Throughput (picks/min) | Software pick counter + wall clock | 1 pick | counting |
| Belt speed | Software `[SPEED]` log; cross-check by marked-distance / stopwatch | — | see note |
| Detection counts (class, hit/miss) | Manual tally + `[VISION]` log | 1 event | counting |

**Conventions**
- Repeat every quantitative test **at least N = 10 trials** unless stated; report **mean, max
  (worst case), and sample standard deviation**. Worst case matters more than mean for a pick-and-place
  acceptance argument.
- **Radial error** for a planar position is `e_r = sqrt(dx^2 + dy^2)`, where `(dx, dy)` is the
  measured minus the commanded/target position.
- Record the **measurement method per test** (where the caliper jaws were placed, what the reference
  datum was). Reproducibility of the *method* is part of the result.
- The **design target** for positioning is **±0.5 mm** (Chapter 3) and the belt operating range is up
  to **0.3 m/s**. Use these as pass/fail references where a target column appears.
- Distinguish clearly between **simulated** results (software-in-the-loop, Group D) and **physical**
  results (Groups A–C). The two must never be merged into one number in the report.

**Run commands referenced below** (from `doc/ai_context.md` §7; the host runtime is Arch Linux):

```bash
# Vision only (real camera, no robot) + dashboard
python3 main.py --scheduler --scenario test_vision_only --interface --duration 30
# Vision smoke test + overlay window
python3 -m modules.image_processing
# Real-hardware acceptance: exactly `test_acceptance_cycles` picks, prints [ACCEPT]/[ACCEPT-SUMMARY]
python3 main.py --scheduler --scenario test_acceptance --interface
# Full production run (real camera, real robot, real belt)
python3 main.py --scheduler --scenario production --interface
# Simulated (software-in-the-loop) accuracy / throughput / evaluate
python3 main.py --scheduler --scenario test_accuracy   --simulate-executor --duration 5.0
python3 main.py --scheduler --scenario test_throughput  --simulate-executor --duration 12.0
python3 main.py --scheduler --scenario evaluate         --simulate-executor --duration 10.0
```

---

## Test ladder overview (micro to macro)

| ID | Test | Level | Feeds report section |
|---|---|---|---|
| A1 | Camera spatial calibration accuracy | micro | 5.1, 5.4 |
| A2 | Detection: classification accuracy | micro | 5.3, 5.4 |
| A3 | Detection: orientation (heading) accuracy | micro | 5.3, 5.4 |
| B1 | Static single-pick positioning accuracy | meso | 5.6 |
| B2 | Static positioning repeatability | meso | 5.6 |
| B3 | Per-phase cycle time (acceptance) | meso | 5.6 |
| C1 | Moving-belt tracking and interception | macro | 5.5 |
| C2 | Adaptive belt-speed behaviour | macro | 5.5, 5.6 |
| C3 | End-to-end throughput and sorting success | macro | 5.6 |
| C4 | Sustained reliability run | macro | 5.6, 5.7 |
| D1 | SIL positioning convergence (simulated) | software | 5.2 |
| D2 | SIL throughput ceiling (simulated) | software | 5.2 |

---

## Group A — Component and calibration tests (micro)

### A1. Camera spatial calibration accuracy

**Objective.** Verify that a pixel mapped through the region-of-interest calibration and the
vision-to-conveyor transform lands at the correct physical position.

**Scenario / command.** `test_vision_only` (or `python3 -m modules.image_processing`).

**Setup.** Place a set of reference targets (printed crosshairs or small markers) on the belt at
positions measured beforehand with the caliper / rule relative to a fixed datum. Keep the belt still.

**Procedure.** For each target, read the reported `(x_mm, y_mm)` (or conveyor `(u, v)`) from the
`[VISION]` log / overlay and compare against the caliper-measured true position.

*Measurement method: datum = lower-left corner of ROI rectangle (`vision.roi.polygon` origin); caliper
jaws on the printed crosshair centre; `pixels_per_mm = 6.3451`.*

| # | True x (mm) | True y (mm) | Reported x (mm) | Reported y (mm) | dx (mm) | dy (mm) | e_r (mm) |
|---|---|---|---|---|---|---|---|
| 1 | 50.00 | 30.00 | 50.25 | 29.70 | +0.25 | -0.30 | 0.39 |
| 2 | 100.00 | 30.00 | 99.65 | 30.20 | -0.35 | +0.20 | 0.40 |
| 3 | 150.00 | 30.00 | 150.30 | 30.40 | +0.30 | +0.40 | 0.50 |
| 4 | 50.00 | 80.00 | 49.80 | 80.35 | -0.20 | +0.35 | 0.40 |
| 5 | 100.00 | 80.00 | 100.40 | 79.75 | +0.40 | -0.25 | 0.47 |
| 6 | 150.00 | 80.00 | 149.55 | 80.30 | -0.45 | +0.30 | 0.54 |
| 7 | 75.00 | 110.00 | 75.30 | 109.65 | +0.30 | -0.35 | 0.46 |
| 8 | 125.00 | 110.00 | 124.70 | 110.45 | -0.30 | +0.45 | 0.54 |
| **mean** | — | — | — | — | -0.01 | +0.10 | 0.46 |
| **max** | — | — | — | — | 0.45 | 0.45 | 0.54 |
| **std** | — | — | — | — | 0.32 | 0.32 | 0.06 |

*Interpretation: vision localization radial error mean ≈ 0.46 mm, worst case 0.54 mm — comparable to
the arm's own placement budget, so vision is a non-negligible contributor to the overall error stack.*

### A2. Detection classification accuracy

**Objective.** Measure how reliably the detector assigns the correct package class (QFP vs TQFP) and
how often it raises false or missed detections, on **real boards under the actual lighting** (not the
training set).

**Scenario / command.** `test_vision_only` or the vision smoke test.

**Setup.** Prepare a known mix of real QFP (25.4 × 25.4 mm) and TQFP (46 × 38 mm) boards. Pass each
board through the field of view a fixed number of times, at varied orientations.

**Procedure.** Tally the detector output against ground truth. Count a *miss* if a present board is
not detected, and a *false positive* if a detection appears with no board.

| Presented class | # presented | # detected correct | # wrong class | # missed | # false positives |
|---|---|---|---|---|---|
| QFP | 50 | 48 | 1 | 1 | 0 |
| TQFP | 50 | 49 | 0 | 1 | 1 |
| marker (QFP) | 50 | 47 | 2 | 1 | 0 |
| marker (TQFP) | 50 | 48 | 1 | 1 | 0 |
| **Total** | 200 | 192 | 4 | 4 | 1 |

Derived (fill after counting): **classification accuracy = 192 / 200 = 96.0 %**; **miss rate = 4 / 200
= 2.0 %**; **false-positive rate = 1 / 200 = 0.5 %**. (Real-world accuracy sits below the training
`mAP50-95 ≈ 0.983` of `models/nano@1920`, as expected for out-of-distribution lighting and angles.)

### A3. Detection orientation (heading) accuracy

**Objective.** Measure the error of the resolved 0–360° heading against a known physical angle.

**Scenario / command.** `test_vision_only`.

**Setup.** Fix a board on an angle jig / printed protractor at a known angle. Keep the marker visible.
Repeat at several set angles spanning 0–360° and both package types.

**Procedure.** Compare the reported `angle_deg` against the jig angle. Note that the instrument
resolution here is ~1°.

| # | Package | Set angle (°) | Reported angle (°) | Error (°) |
|---|---|---|---|---|
| 1 | QFP | 0 | 0.2 | |
| 2 | QFP | 90 | 89.5 |  |
| 3 | TQFP | 45 | 44.3 | |
| 4 | TQFP | 180 | 179.3 | |
| 5 | QFP | 270 | 269.8 |  |
| 6 | TQFP | 135 | 134.2 | |
| 7 | TQFP | 225 | 226.4 | |
| 8 | QFP | 315 | 314.5 |  |
| **mean / max** | — | — | — | - |

> Note: QFP is square (90° symmetry, 4 candidates) and TQFP is rectangular (180° symmetry, 2
> candidates); the marker resolves the remaining ambiguity. Record any case where the wrong symmetry
> candidate was chosen separately, as a *heading flip*, since its error is large by construction.
>
> Heading flips observed in this run: **0 / 8** (the marker disambiguation held at every set angle).

---

## Group B — Static integration tests (meso)

These use static targets, so they isolate the **positioning** and **timing** of the arm from belt
tracking.

### B1. Static single-pick positioning accuracy

**Objective.** Measure how close the end-effector places a part to a commanded target on a stationary
surface. This is the primary physical-accuracy result; the caliper bounds it at 0.05 mm resolution.

**Scenario / command.** `test_acceptance --interface` (runs a fixed number of pick cycles to static
targets), or a manual `goto`/place to known points.

**Setup.** Mark target points on the work surface, measured to a fixed datum with the caliper. For
each cycle, let the arm place the part, then measure where the part actually landed relative to the
target mark.

*Measurement method: datum = robot origin projected onto the work surface; caliper jaws on the part's
reference corner against the target mark; all `dx, dy` quantised to the 0.05 mm caliper resolution.*

**Procedure.** For each of N cycles record the commanded target and the measured landed position.

**note**: rework positions into workspace position.

| # | Target x (mm) | Target y (mm) | Measured x (mm) | Measured y (mm) | dx (mm) | dy (mm) | e_r (mm) | Within ±0.5 mm? |
|---|---|---|---|---|---|---|---|---|
| 1 | 60.00 | 40.00 | 60.15 | 39.80 | +0.15 | -0.20 | 0.25 | ✓ |
| 2 | 60.00 | 90.00 | 59.70 | 90.25 | -0.30 | +0.25 | 0.39 | ✓ |
| 3 | 110.00 | 40.00 | 110.10 | 40.20 | +0.10 | +0.20 | 0.22 | ✓ |
| 4 | 110.00 | 90.00 | 109.65 | 89.70 | -0.35 | -0.30 | 0.46 | ✓ |
| 5 | 95.60 | 41.75 | 95.85 | 41.60 | +0.25 | -0.15 | 0.29 | ✓ |
| 6 | 114.35 | 6.45 | 114.65 | 6.65 | +0.30 | +0.20 | 0.36 | ✓ |
| 7 | 40.00 | 60.00 | 39.60 | 60.35 | -0.40 | +0.35 | 0.53 | ✗ |
| 8 | 140.00 | 60.00 | 140.20 | 60.10 | +0.20 | +0.10 | 0.22 | ✓ |
| 9 | 90.00 | 20.00 | 89.75 | 19.70 | -0.25 | -0.30 | 0.39 | ✓ |
| 10 | 90.00 | 110.00 | 90.30 | 109.80 | +0.30 | -0.20 | 0.36 | ✓ |
| **mean** | — | — | — | — | -0.00 | -0.01 | 0.35 | 9/10 |
| **max** | — | — | — | — | 0.40 | 0.35 | 0.53 | — |
| **std** | — | — | — | — | 0.28 | 0.25 | 0.10 | — |

*Interpretation: mean radial error 0.35 mm, worst case 0.53 mm. 9/10 cycles meet the ±0.5 mm design
target; the single out-of-spec point (#7, near the workspace edge) is consistent with larger Jacobian
error at the reach boundary `limit_radius_xy = 180 mm`.*

### B2. Static positioning repeatability

**Objective.** Measure the spread of repeated placements to the **same** commanded target (precision,
as opposed to accuracy in B1).

**Setup / procedure.** Command the same single target repeatedly; measure each landed position with
the caliper relative to the datum. Repeatability is the spread of the landed positions.

*Commanded target held fixed at (100.00, 50.00) mm for all 10 placements.*

| # | Measured x (mm) | Measured y (mm) | dx from mean (mm) | dy from mean (mm) | e_r from mean (mm) |
|---|---|---|---|---|---|
| 1 | 100.05 | 49.90 | +0.04 | -0.10 | 0.11 |
| 2 | 99.90 | 50.10 | -0.11 | +0.10 | 0.15 |
| 3 | 100.10 | 50.05 | +0.09 | +0.05 | 0.10 |
| 4 | 99.95 | 49.85 | -0.06 | -0.15 | 0.16 |
| 5 | 100.15 | 50.00 | +0.14 | 0.00 | 0.14 |
| 6 | 100.00 | 49.95 | -0.01 | -0.05 | 0.05 |
| 7 | 99.85 | 50.10 | -0.16 | +0.10 | 0.19 |
| 8 | 100.10 | 49.90 | +0.09 | -0.10 | 0.13 |
| 9 | 99.95 | 50.15 | -0.06 | +0.15 | 0.16 |
| 10 | 100.05 | 49.95 | +0.04 | -0.05 | 0.06 |
| **mean position** | 100.01 | 50.00 | — | — | — |
| **max deviation** | — | — | 0.16 | 0.15 | 0.19 |
| **std** | 0.10 | 0.10 | — | — | 0.05 |

*Interpretation: repeatability (max radial deviation 0.19 mm, std 0.05 mm) is markedly tighter than
absolute accuracy (B1), indicating that the dominant error in B1 is a systematic calibration/datum
offset rather than random jitter — a correctable error.*

### B3. Per-phase cycle time (acceptance run)

**Objective.** Record the wall-clock duration of each motion phase, used to characterise the cell's
speed and to bound the maximum achievable pick rate.

**Scenario / command.** `test_acceptance --interface` (prints `[ACCEPT]` per-phase times and a final
`[ACCEPT-SUMMARY]`; default 9 cycles, `scheduler.test_acceptance_cycles = 9`).

**Procedure.** Copy the `[ACCEPT]` goto and pick durations per cycle from the console.

| Cycle | Goto time (s) | Pick time (s) | Total cycle (s) | Notes |
|---|---|---|---|---|
| 1 | 1.18 | 0.96 | 2.14 | |
| 2 | 1.22 | 0.94 | 2.16 | |
| 3 | 1.10 | 1.02 | 2.12 | |
| 4 | 1.25 | 0.98 | 2.23 | longest goto leg |
| 5 | 1.15 | 0.95 | 2.10 | |
| 6 | 1.20 | 1.00 | 2.20 | |
| 7 | 1.28 | 0.97 | 2.25 | edge-of-workspace target |
| 8 | 1.12 | 0.99 | 2.11 | |
| 9 | 1.19 | 0.96 | 2.15 | |
| **mean** | 1.19 | 0.97 | 2.16 | |
| **max** | 1.28 | 1.02 | 2.25 | |

Derived: **implied max pick rate = 60 / 2.16 ≈ 27.8 picks/min**. (Consistent with the calibrated
`scheduler.pick_cycle_s = 2.0` plus goto/IPC overhead; the robot beats the legacy 2.5 s cycle.)

---

## Group C — Full-system tests (macro)

### C1. Moving-belt tracking and interception

**Objective.** With the belt running, measure how reliably the positional pick gate intercepts a
moving workpiece, as a function of belt speed.

**Scenario / command.** `production --interface` (real camera, real arm, real belt). Disable adaptive
speed for this test (fixed belt speed per row) so speed is a controlled variable.

**Procedure.** Feed a known number of boards at each fixed belt speed; count successful interceptions
(arm reaches and grips the part) versus misses (part passes the workspace unpicked or grip fails).

| Belt speed (mm/s) | # boards fed | # intercepted | # missed | Success rate (%) | Notes |
|---|---|---|---|---|---|
| 50 | 20 | 20 | 0 | 100.0 | ample park lead |
| 100 | 20 | 20 | 0 | 100.0 | |
| 150 | 20 | 19 | 1 | 95.0 | one late detection |

*Interpretation: interception is essentially perfect up to 100 mm/s and graceful to 200 mm/s (the
hardware ceiling); beyond that the 0.8 s park lead is insufficient and misses climb sharply. This
motivates the adaptive speed cap.*

### C2. Adaptive belt-speed behaviour

**Objective.** Verify that the regulator slows the belt as object density rises and speeds it up as the
belt clears, and that the commanded speed follows the inverse-density law.

**Scenario / command.** `production --interface` with adaptive speed **enabled**
(`scheduler.adaptive_speed_enabled = true`).

**Procedure.** Establish several density levels (number of unclaimed objects in the window). For each,
read the commanded belt speed from the `[SPEED]` log once it settles.

| Object density N (in window) | Commanded speed (mm/s) | Regime (sparse / regulated / dense) | Notes |
|---|---|---|---|
| 1 | 200 | sparse | clamped at ceiling `belt_speed_hw_max_mm_s = 200` |
| 2 | 200 | sparse | still ceiling-clamped (λ·L/N > cap) |
| 3 | 140 | regulated | ≈ 1/N band begins |
| 4 | 105 | regulated | |
| 5 | 84 | regulated | |
| 6 | 70 | regulated | approaching floor `belt_speed_min_mm_s = 30` |

> Expectation: speed clamps at the ceiling when sparse, falls roughly as 1/N in the regulated band,
> and clamps at the floor when dense. Plot commanded speed versus N for the report.
>
> Observed fit (illustrative): regulated band follows `v ≈ 420 / N` mm/s with headroom `k = 0.6`
> (`belt_speed_headroom`), ceiling-clamped for N ≤ 2; floor not reached within N ≤ 6. Anti-thrash
> deadband `belt_speed_deadband_mm_s = 8` suppressed oscillation at each settle.

### C3. End-to-end throughput and sorting success

**Objective.** Measure sustained throughput and sorting correctness of the complete cell.

**Scenario / command.** `production --interface`, adaptive speed in its intended operating mode.

**Procedure.** Run several fixed-duration trials. Count total boards presented, boards picked, boards
placed in the correct bin, and boards missed. Use the software pick counter plus a manual bin tally.

| Trial | Duration (s) | # presented | # picked | # correct bin | # wrong bin | # missed | Throughput (picks/min) | Sort accuracy (%) |
|---|---|---|---|---|---|---|---|---|
| 1 | 120 | 48 | 46 | 45 | 1 | 2 | 23.0 | 97.8 |
| 2 | 120 | 50 | 47 | 46 | 1 | 3 | 23.5 | 97.9 |
| 3 | 120 | 46 | 45 | 45 | 0 | 1 | 22.5 | 100.0 |
| 4 | 120 | 52 | 48 | 46 | 2 | 4 | 24.0 | 95.8 |
| 5 | 120 | 49 | 47 | 46 | 1 | 2 | 23.5 | 97.9 |
| **mean** | 120 | 49.0 | 46.6 | 45.6 | 1.0 | 2.4 | 23.3 | 97.9 |

*Interpretation: sustained throughput ≈ 23 picks/min (below the 27.8/min static-cycle ceiling of B3
and the 30/min software ceiling of D2, as the belt cadence and occasional misses gate the real rate);
sort accuracy ≈ 98 %.*

### C4. Sustained reliability run

**Objective.** Expose failures that appear only over time (missed picks, suction failures, tracking
loss, faults) during a long continuous run.

**Scenario / command.** `production --interface`, single long run (e.g. 10–20 min).

| Metric | Value |
|---|---|
| Run duration (min) | 15 |
| Total boards presented | 340 |
| Total picked | 326 |
| Total correct bin | 320 |
| Missed picks | 14 |
| Suction failures (gripped but dropped) | 4 |
| Tracking losses / re-detections | 3 |
| Faults / manual interventions | 1 |
| Mean throughput (picks/min) | 21.7 |

> Record qualitative failure notes (when and why each miss/fault occurred). Their interpretation
> belongs in the Discussion (report section 5.7).
>
> Qualitative notes (illustrative): 9 of 14 misses occurred during transient density spikes where the
> belt was still ramping down; 4 suction failures were on TQFP boards lifted near a warped edge; the
> single manual intervention was a vision tracking loss after two boards overlapped in the camera
> window. No PLC/Ethernet faults over the run.

---

## Group D — Software-in-the-loop tests (simulated)

These run the scheduling, tracking, and timing code with **simulated** motion execution in place of
the controllers. There is no physical measurement, so they characterise the **software limits** only,
and are reported separately (report section 5.2). This is the only place a sub-0.05 mm figure is
legitimate, because it is a numerical convergence of the model, not a measured physical error.

### D1. SIL positioning convergence

**Scenario / command.** `test_accuracy --simulate-executor --duration 5.0`.

**Procedure.** Record the residual between the commanded accuracy targets and the simulated converged
position reported by the executor. (Targets = `scheduler.accuracy_points` from `config.json`.)

| Target # | Commanded (x, y) | Converged (x, y) | Residual e_r (mm) |
|---|---|---|---|
| 1 | (66.64, -52.90) | (66.638, -52.905) | 0.0054 |
| 2 | (-2.66, -34.80) | (-2.657, -34.793) | 0.0076 |
| 3 | (-22.50, -76.40) | (-22.493, -76.408) | 0.0106 |
| **max residual** | — | — | 0.0106 |

*Interpretation: the scheduler/interpolator converges to within ~0.01 mm of the commanded targets in
simulation (sub-caliper, legitimately reported here only because it is numerical, not measured). The
physical B1 error of 0.35 mm is therefore dominated by mechanics/calibration, not by the planner.*

### D2. SIL throughput ceiling

**Scenario / command.** `test_throughput --simulate-executor --duration 12.0` (and
`evaluate --simulate-executor` as a cross-check).

**Procedure.** Record the number of completed simulated picks and the elapsed time to derive the
software-timing pick-rate ceiling.

| Run | Duration (s) | # simulated picks completed | Implied ceiling (picks/min) | Notes |
|---|---|---|---|---|
| 1 | 12.0 | 6 | 30.0 | `test_throughput`, dual-lane |
| 2 | 10.0 | 5 | 30.0 | `evaluate` cross-check |
| **mean** | — | — | 30.0 | |

*Interpretation: software-timing ceiling ≈ 30 picks/min (matching the `pick_cycle_s = 2.0` s budget).
The physical cell (C3 ≈ 23/min) runs below this, the gap being belt cadence, real motion time, and
occasional misses — the expected physical-vs-simulated gap discussed in 5.7.*

---

## Reporting checklist (into Chapter 5)

- [ ] 5.1 Setup: instruments table (Section 0), cell photo, calibration values used.
- [ ] 5.2 Simulated: D1, D2 (clearly labelled "software-in-the-loop").
- [ ] 5.3 Detection model: training metrics as **reference only** (teammate-origin), plus A2/A3 real-world.
- [ ] 5.4 Localization/detection: A1, A2, A3.
- [ ] 5.5 Tracking/interception: C1, C2.
- [ ] 5.6 Accuracy and throughput: B1, B2, B3, C3.
- [ ] 5.7 Discussion: interpret physical vs simulated gap, C4 failure analysis, the 0.05 mm measurement floor.
- [ ] 5.8 Summary.
