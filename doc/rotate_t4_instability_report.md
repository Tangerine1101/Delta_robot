# T4 (Suction-Cup Rotation) Instability — Data Report

**Date:** 2026-07-06 (updated same day — root cause resolved, see §6)
**Status of the rest of the system:** operating normally. This is the one open defect.
**Scope of this report:** symptom description + measured/simulated data only. Root-cause
analysis is intentionally out of scope here (see the separate fix plan).

## 0. Update — root cause confirmed and fixed

A real `production` hardware log (`production.log`, 12 picks) became available after this
report was written and was cross-correlated (`[ROTATE]` × `[GATE]` × `[DETECT]`), superseding
§5's "does not show" caveat. Summary (full detail in `doc/ai_context.md`'s
2026-07-06c status entry):

- **Not a PLC/ST retrigger issue.** The log falsifies it directly: one pick succeeded with no
  `change_speed` interleaved between the two `rotate_absolute` calls, another failed *with* one
  interleaved.
- **Actual cause: a PC-side dispatch bug.** The post-grip rotate was gated on sampling
  `pose.z` inside a ~2mm band out of a ~13mm excursion at a 50ms poll rate — the command was
  silently **never sent** on 6/12 real picks (not delayed, not partial: `rotate_at_end_deg` was
  exactly `0.0`, matching the cup's unchanged home angle).
- **Fixed**: wider descent detection + a modeled-time fallback dispatch
  (`modules/scheduler.py:_wait_for_arm_arrival`), a gate-time angle refresh with an outlier
  guard, and per-track marker-heading memory in `modules/image_processing.py` (fixes the
  separate ~90° angle jump on marker dropout also visible in the log, e.g. `yolo-000009`).
- The `QFP`/`TQFP` numerical-identity and `offset_by_class`-uncalibrated observations in §4
  still stand (unrelated to this bug). The `rotate_sign` doc/config discrepancy noted in §4 is
  also still open — separate from this fix, still needs the hardware sweep in `test_rotate.py`.

## 1. Symptom

On the physical line, picked boards land in the sorting bin at **random, per-object wrong
orientations**, even though the software angle-conversion chain has already been audited and
two constant errors in it were fixed (`doc/ai_context.md`, status block dated 2026-07-06 /
2026-07-06b; commit `cd8bab8`). The defect is intermittent and object-dependent, not a fixed
offset — i.e. it does not look like a single wrong constant, which is what motivated collecting
data before proposing any further fix.

## 2. Confirmed design (for reference, matches current code)

- **Timing:** the axis is homed to R-frame 0 rad while the arm is still in flight toward the
  pick point (dispatched together with the `goto` packet, off the critical path) —
  `modules/scheduler.py:905-918`. The board is rotated to its target angle **after** grip, once
  the arm has lifted back to `z ≥ pre_pick_height` during the pick-phase trajectory —
  `modules/scheduler.py:977-987` (`pre_pick_z=plan.trajectory_goto[-1].z`).
- **Why precision matters:** QFP and TQFP boards are nominally square/rectangular, but their
  pin/pad layout is not equally symmetric, so the cup must land the board at a precise angle,
  not just "close enough." This is why a **marker** is glued to each board: it gives the vision
  layer an unambiguous 360° vector (`heading_from_marker_vector`,
  `modules/image_processing.py:467-474`) instead of relying on the OBB box angle alone, which
  folds into a symmetric range and cannot disambiguate the board's true orientation by itself.

Both points match the intended design as described by the project owner; no drift found here.

## 3. Data collection method

Two tools, neither modifies production code:

1. **`modules/test_rotate.py`** (pre-existing, hardware-only): sweeps the physical Siemens axis
   with the cup free and checks remap/settle, implied axis speed, visual spin direction, and
   CommandID retrigger behavior. Requires a live PLC connection; **not run for this report**
   (no hardware session was part of this task) — its four candidate hardware causes are already
   catalogued in `doc/ai_context.md` and are not repeated here.
2. **`modules/rotate_sweep_sim.py`** (new, pure software, no hardware): sweeps a synthetic raw
   marker angle over the full 0-360 deg range, once per configured PCB class, through the exact
   production functions (imported, not reimplemented):
   `offset_by_class` add → `ConveyorFrame.vision_heading_to_robot_rad` → the scheduler's
   `wrap_rad(rotate_sign * (rotate_offset_rad - board_heading_rad))` → `robot_rad_to_wire_deg`.
   Run with:
   ```bash
   python3 -m modules.rotate_sweep_sim --step-deg 1 --csv <output.csv>
   ```

## 4. Results

360 samples per class (1 deg step), both `QFP` and `TQFP`:

```
TQFP   n=360  cmd_deg[min=-180.00 max=+179.00 mean_abs=90.00]  near_180_boundary(|cmd|>170deg)=19/360  wrap_discontinuities=1
QFP    n=360  cmd_deg[min=-180.00 max=+179.00 mean_abs=90.00]  near_180_boundary(|cmd|>170deg)=19/360  wrap_discontinuities=1
```

Observations (data only, no causal claims):

- The mapping raw-marker-angle → `rotate_cmd_deg` is a **clean, deterministic, monotonic**
  function with exactly **one discontinuity** (the expected `+179 -> -180` wrap at the
  half-turn boundary — `wrap_rad`'s designed shortest-turn representation, not an anomaly).
  Feeding the same input twice always produces the same output (checked by construction: the
  functions are pure).
- **~5.3% of all possible board headings (19/360) require a commanded swing within 10 deg of
  the physical ±180 deg limit** — i.e. close to the largest single rotation the axis can be
  asked to do in one post-grip move. This is a geometric fact about the current
  `rotate_offset_deg=0`, `rotate_sign=1` configuration, not a claim about whether the axis can
  or cannot complete such a swing in time.
- **`QFP` and `TQFP` currently produce numerically identical results** because
  `vision.orientation.offset_by_class` is `{"QFP": 0.0, "TQFP": 0.0}` in the live
  `modules/config.json` — the per-class correction that the marker-based design exists to
  support is present in the config schema but not yet calibrated to a non-zero value for either
  class.
- **Documentation/config discrepancy observed:** `doc/ai_context.md`'s latest status entry
  states `rotate_sign: -1` "per the user," but the live `modules/config.json` currently has
  `"rotate_sign": 1`. Recorded here as a fact found while pulling the config for the sweep; which
  value is correct is not evaluated in this report.
- No hardware-side timing data (`rotate_at_gate_deg` / `rotate_at_end_deg` from the `[ROTATE]`
  log, `modules/scheduler.py:991-1010`) exists yet: that log only fires inside
  `RealtimePickExecutor.execute()`, which requires a live `RealtimeState` fed by real Siemens
  status reads — it does not run under any `--simulate-executor` scenario. Separately, the
  built-in simulated object source (`SimulatedImageProcessing`,
  `modules/image_processing.py:157-243`) never sets `angle_deg` on its fake detections (always
  the dataclass default `0.0`), so none of the existing `test_accuracy` / `test_throughput` /
  `evaluate` scenarios exercise the rotation pipeline at all — this is why a dedicated sweep
  script was needed to get any rotation-angle data without hardware.

## 5. What this data does and does not show

- **Does show:** the PC-side angle math itself is internally consistent across the full input
  range, for both classes, under the current configuration.
- **Does not show:** why boards land at wrong orientations on hardware. That requires either
  running `modules/test_rotate.py` against the live Siemens axis, or collecting `[ROTATE]` log
  lines from a real `production` run — both out of scope for this report by request.

Full per-sample CSV (both classes, 1 deg step) available by re-running the command in §3.
