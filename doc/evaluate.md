# Evaluation Plan — Delta Robot Sorting Cell

This document is the test plan that feeds **Chapter 5 (Experiment and Evaluation)** of the thesis.
It defines the test cases from the micro level (individual subsystems and calibration) up to the
macro level (the full cell running end to end), and provides **blank data-collection tables** to be
filled in during testing. Fill the tables here first, then transcribe the summarised numbers into the
report.

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

| # | True x (mm) | True y (mm) | Reported x (mm) | Reported y (mm) | dx (mm) | dy (mm) | e_r (mm) |
|---|---|---|---|---|---|---|---|
| 1 |  |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |  |
| 4 |  |  |  |  |  |  |  |
| 5 |  |  |  |  |  |  |  |
| 6 |  |  |  |  |  |  |  |
| 7 |  |  |  |  |  |  |  |
| 8 |  |  |  |  |  |  |  |
| **mean** |  |  |  |  |  |  |  |
| **max** |  |  |  |  |  |  |  |
| **std** |  |  |  |  |  |  |  |

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
| QFP |  |  |  |  |  |
| TQFP |  |  |  |  |  |
| marker (QFP) |  |  |  |  |  |
| marker (TQFP) |  |  |  |  |  |
| **Total** |  |  |  |  |  |

Derived (fill after counting): classification accuracy = correct / presented; miss rate; false-positive
rate.

### A3. Detection orientation (heading) accuracy

**Objective.** Measure the error of the resolved 0–360° heading against a known physical angle.

**Scenario / command.** `test_vision_only`.

**Setup.** Fix a board on an angle jig / printed protractor at a known angle. Keep the marker visible.
Repeat at several set angles spanning 0–360° and both package types.

**Procedure.** Compare the reported `angle_deg` against the jig angle. Note that the instrument
resolution here is ~1°.

| # | Package | Set angle (°) | Reported angle (°) | Error (°) |
|---|---|---|---|---|
| 1 | QFP |  |  |  |
| 2 | QFP |  |  |  |
| 3 | TQFP |  |  |  |
| 4 | TQFP |  |  |  |
| 5 |  |  |  |  |
| 6 |  |  |  |  |
| 7 |  |  |  |  |
| 8 |  |  |  |  |
| **mean / max** |  |  |  |  |

> Note: QFP is square (90° symmetry, 4 candidates) and TQFP is rectangular (180° symmetry, 2
> candidates); the marker resolves the remaining ambiguity. Record any case where the wrong symmetry
> candidate was chosen separately, as a *heading flip*, since its error is large by construction.

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

**Procedure.** For each of N cycles record the commanded target and the measured landed position.

| # | Target x (mm) | Target y (mm) | Measured x (mm) | Measured y (mm) | dx (mm) | dy (mm) | e_r (mm) | Within ±0.5 mm? |
|---|---|---|---|---|---|---|---|---|
| 1 |  |  |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |  |  |
| 4 |  |  |  |  |  |  |  |  |
| 5 |  |  |  |  |  |  |  |  |
| 6 |  |  |  |  |  |  |  |  |
| 7 |  |  |  |  |  |  |  |  |
| 8 |  |  |  |  |  |  |  |  |
| 9 |  |  |  |  |  |  |  |  |
| 10 |  |  |  |  |  |  |  |  |
| **mean** |  |  |  |  |  |  |  |  |
| **max** |  |  |  |  |  |  |  |  |
| **std** |  |  |  |  |  |  |  |  |

### B2. Static positioning repeatability

**Objective.** Measure the spread of repeated placements to the **same** commanded target (precision,
as opposed to accuracy in B1).

**Setup / procedure.** Command the same single target repeatedly; measure each landed position with
the caliper relative to the datum. Repeatability is the spread of the landed positions.

| # | Measured x (mm) | Measured y (mm) | dx from mean (mm) | dy from mean (mm) | e_r from mean (mm) |
|---|---|---|---|---|---|
| 1 |  |  |  |  |  |
| 2 |  |  |  |  |  |
| 3 |  |  |  |  |  |
| 4 |  |  |  |  |  |
| 5 |  |  |  |  |  |
| 6 |  |  |  |  |  |
| 7 |  |  |  |  |  |
| 8 |  |  |  |  |  |
| 9 |  |  |  |  |  |
| 10 |  |  |  |  |  |
| **mean position** |  |  | — | — | — |
| **max deviation** |  |  |  |  |  |
| **std** |  |  |  |  |  |

### B3. Per-phase cycle time (acceptance run)

**Objective.** Record the wall-clock duration of each motion phase, used to characterise the cell's
speed and to bound the maximum achievable pick rate.

**Scenario / command.** `test_acceptance --interface` (prints `[ACCEPT]` per-phase times and a final
`[ACCEPT-SUMMARY]`; default 9 cycles).

**Procedure.** Copy the `[ACCEPT]` goto and pick durations per cycle from the console.

| Cycle | Goto time (s) | Pick time (s) | Total cycle (s) | Notes |
|---|---|---|---|---|
| 1 |  |  |  |  |
| 2 |  |  |  |  |
| 3 |  |  |  |  |
| 4 |  |  |  |  |
| 5 |  |  |  |  |
| 6 |  |  |  |  |
| 7 |  |  |  |  |
| 8 |  |  |  |  |
| 9 |  |  |  |  |
| **mean** |  |  |  |  |
| **max** |  |  |  |  |

Derived: implied max pick rate = 60 / (mean total cycle) picks/min.

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
|  |  |  |  |  |  |
|  |  |  |  |  |  |
|  |  |  |  |  |  |
|  |  |  |  |  |  |
|  |  |  |  |  |  |

### C2. Adaptive belt-speed behaviour

**Objective.** Verify that the regulator slows the belt as object density rises and speeds it up as the
belt clears, and that the commanded speed follows the inverse-density law.

**Scenario / command.** `production --interface` with adaptive speed **enabled**.

**Procedure.** Establish several density levels (number of unclaimed objects in the window). For each,
read the commanded belt speed from the `[SPEED]` log once it settles.

| Object density N (in window) | Commanded speed (mm/s) | Regime (sparse / regulated / dense) | Notes |
|---|---|---|---|
| 1 |  |  |  |
| 2 |  |  |  |
| 3 |  |  |  |
| 4 |  |  |  |
| 5 |  |  |  |
| 6 |  |  |  |

> Expectation: speed clamps at the ceiling when sparse, falls roughly as 1/N in the regulated band,
> and clamps at the floor when dense. Plot commanded speed versus N for the report.

### C3. End-to-end throughput and sorting success

**Objective.** Measure sustained throughput and sorting correctness of the complete cell.

**Scenario / command.** `production --interface`, adaptive speed in its intended operating mode.

**Procedure.** Run several fixed-duration trials. Count total boards presented, boards picked, boards
placed in the correct bin, and boards missed. Use the software pick counter plus a manual bin tally.

| Trial | Duration (s) | # presented | # picked | # correct bin | # wrong bin | # missed | Throughput (picks/min) | Sort accuracy (%) |
|---|---|---|---|---|---|---|---|---|
| 1 |  |  |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |  |  |
| 4 |  |  |  |  |  |  |  |  |
| 5 |  |  |  |  |  |  |  |  |
| **mean** |  |  |  |  |  |  |  |  |

### C4. Sustained reliability run

**Objective.** Expose failures that appear only over time (missed picks, suction failures, tracking
loss, faults) during a long continuous run.

**Scenario / command.** `production --interface`, single long run (e.g. 10–20 min).

| Metric | Value |
|---|---|
| Run duration (min) |  |
| Total boards presented |  |
| Total picked |  |
| Total correct bin |  |
| Missed picks |  |
| Suction failures (gripped but dropped) |  |
| Tracking losses / re-detections |  |
| Faults / manual interventions |  |
| Mean throughput (picks/min) |  |

> Record qualitative failure notes (when and why each miss/fault occurred). Their interpretation
> belongs in the Discussion (report section 5.7).

---

## Group D — Software-in-the-loop tests (simulated)

These run the scheduling, tracking, and timing code with **simulated** motion execution in place of
the controllers. There is no physical measurement, so they characterise the **software limits** only,
and are reported separately (report section 5.2). This is the only place a sub-0.05 mm figure is
legitimate, because it is a numerical convergence of the model, not a measured physical error.

### D1. SIL positioning convergence

**Scenario / command.** `test_accuracy --simulate-executor --duration 5.0`.

**Procedure.** Record the residual between the commanded accuracy targets and the simulated converged
position reported by the executor.

| Target # | Commanded (x, y) | Converged (x, y) | Residual e_r (mm) |
|---|---|---|---|
| 1 |  |  |  |
| 2 |  |  |  |
| 3 |  |  |  |
| **max residual** |  |  |  |

### D2. SIL throughput ceiling

**Scenario / command.** `test_throughput --simulate-executor --duration 12.0` (and
`evaluate --simulate-executor` as a cross-check).

**Procedure.** Record the number of completed simulated picks and the elapsed time to derive the
software-timing pick-rate ceiling.

| Run | Duration (s) | # simulated picks completed | Implied ceiling (picks/min) | Notes |
|---|---|---|---|---|
| 1 |  |  |  |  |
| 2 |  |  |  |  |
| **mean** |  |  |  |  |

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
