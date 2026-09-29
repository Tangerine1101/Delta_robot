# Pick-Accuracy Investigation — Gate/Timing Defects

> **Status (2026-09-16).** T2 and T6–T9 are fixed in code. **T1's conclusion did not survive
> the PLC simulator**: with an ideal (no servo lag) port of `Matching_Code_10`, the existing
> gate lead lands centred up to 120 mm/s, and adding the modelled descent misses every pick.
> The T1 evidence below came from a plant model that uses the S-curve descent. T3 (a bench
> measurement of dispatch→contact time and contact depth) is now the lead item. Current status
> of each ID: [`open-issues.md`](open-issues.md) §C.1d; resolutions:
> [`decision-log.md`](decision-log.md).

> **Scope.** Why picks land off-centre and the error grows with belt speed, worst above
> ~120 mm/s. Covers the pick-position predictor, the positional gate and the adaptive-speed
> commit policy as they were in `modules/scheduler.py` on 2026-08-28, cross-checked against the
> parallel implementation in the research repository `../python for scheduling`.
>
> **Code references** name the functions of that date. Since the 2026-09-25 refactor
> (`decision-log.md` §3) they live at: gate status → `runtime/pick_gate.object_gate_status`;
> gate lead, descent time, contact point, intercept solver → `core/delta.DeltaArm`
> (`gate_lead_s`, `descent_time_s`, `contact_position`, `predict`); trajectory-time model →
> `core/motion.trajectory_time` / `corner_v_end`; pick executor and its waits →
> `runtime/pick_executor.RealtimePickExecutor`; speed commits → `runtime/speed.SpeedController`
> and `scheduling/commit.commit_step`; belt sample → `runtime/speed_source.ConveyorSpeedSource`.
>
> **Method.** Every number below was reproduced by an automated test (the 2026-08-28
> investigation suite, rerun that day; its lasting parts are now `tests/test_runtime.py` and
> `tests/test_pick_gate.py`) or read directly from the code at the line cited — nothing here is
> inferred without a test or a line reference. No file was changed while gathering this
> evidence.
>
> **Source.** A Vietnamese narrative report (2026-08-28), the investigation's `REPORT.md`,
> addendum and result files, and a verification report — all archived locally since
> 2026-09-25. This file is the English defect register derived from them; the architecture itself is already documented
> in [`basis-theory.md`](basis-theory.md) §4 (tracking/interception) and §6 (adaptive speed) —
> read those first. Cross-references to the PLC review use the IDs of
> [`plc/version-diff-and-defects.md`](plc/version-diff-and-defects.md).

## 1. Why the architecture is sound and the bugs are still costly

The design is not being second-guessed here: encoder-anchored tracking (`basis-theory.md`
§4.1) and the live positional gate (§4.4) make the pick immune to belt-speed *estimate* noise
— accuracy depends on exactly one thing, whether the gate's lead offset matches the true
dead time between gate-fire and suction contact. Simulation with correct parameters gives a
1.7–3.0 mm baseline error up to 150 mm/s. Every defect below is a way that lead ends up wrong,
or a way the one-sided gate's assumptions are violated.

The key relation used throughout: **error (mm) = belt speed (mm/s) × unpaid time (s)**.

## 2. Defects

### T1 — Gate lead omits the soft-start and descent time — **Blocker** (accuracy)

`_object_pick_gate_status` fires the pick gate at

```
threshold = u_pick − _belt_lead_offset_mm(v_belt, command_delay_s)
```

with `command_delay_s = robot_movement_delay_s + ethernet_delay_s + gate_sampling_latency_s`
(old `scheduler.py:1580-1581`). The true dead time from
dispatch to suction contact is dispatch delay + wire time + the **State-10 soft start**
(80 ms) + the **16 mm descent** (modelled at 0.31 s by
`_descent_time_s`):

| Component | Paid by the lead? | Value |
|---|---|---|
| `ethernet_delay_s` + `robot_movement_delay_s` | yes | 0.186 s |
| soft start (State 10) | **no** | 0.08 s |
| 16 mm descent (S-curve model) | **no** | 0.31 s |

At the current interpolator settings the unpaid ≈0.35–0.39 s costs, at contact:

| Belt speed | Unpaid travel |
|---|---|
| 30 mm/s | 11.7 mm |
| 60 mm/s | 23.4 mm |
| 100 mm/s | 39.0 mm |
| 120 mm/s | 46.8 mm |
| 150 mm/s | 58.5 mm |

A QFP board is 25.4 mm — at 120 mm/s the cup lands almost two board-widths behind. The
comment at the gate (old `scheduler.py:1575-1580`) explains
the omission is intentional **only while oblique descent is slanting the contact point to
meet the board** (§4.5): in that case adding $t_d$ to the lead would double-count the travel.
With `oblique_descent_enabled = false` (the correct setting — see T2/T3) the compensation
mechanism is off, but the descent time is still not added back. The system is stuck between
the two designs.

`robot_movement_delay_s = 0.17` has never been calibrated (`open-issues.md` **C2**) and is
absorbing part of this gap by accident — which is why the system "works" at one speed and not
another: a single scalar tuned at one belt speed cannot also fix an error that scales with it.

**Fix** (`open-issues.md` §A, tag NEAR): once T2 is fixed, use
`threshold = u_pick − v_belt · (T_delay + t_soft + t_d + sampling)` for vertical descent, and
calibrate the combined $\Delta t$ from measured error at two belt speeds rather than guessing
the individual terms (§3 below).

### T2 — `[GATE] t_d_model_s` is biased when oblique descent is off — **High** (blocks C2)

`_contact_position` calls
`_descent_time_s(settings, v_belt)` **before** checking `oblique_descent_enabled`. That
function folds the oblique *slant* into the descent segment's length, so with oblique off
(vertical descent) the value it returns is still the duration of a slanted descent that never
happens:

| Belt speed | `t_d_model_s` reported | True (vertical) descent | Bias |
|---|---|---|---|
| 0 | 0.390 s | 0.390 s | 0 |
| 60 mm/s | 0.492 s | 0.390 s | −0.10 s |
| 120 mm/s | 0.625 s | 0.390 s | −0.23 s |
| 150 mm/s | 0.683 s | 0.390 s | −0.29 s |

The calibration procedure `open-issues.md` **C2** prescribes — subtract `t_d_model_s` from
`dispatch_to_contact_s` in the `[GATE]` log to get `robot_movement_delay_s` — subtracts a
speed-dependent, inflated number. **Every `robot_movement_delay_s` ever derived this way is
structurally wrong, and wrong by a different amount at each belt speed.** This is also why the
earlier attempt to enable oblique descent failed: at 120 mm/s the model shifts the contact
point 75 mm downstream for a descent that is really much shorter (simulation E6: −65 mm error,
9% hit rate with oblique on).

**Fix**: report `_descent_time_s(settings, 0.0)` (or the value measured in T3) when
`oblique_descent_enabled` is false. Until fixed, no `robot_movement_delay_s` derived from the
`[GATE]` log should be used.

### T3 — Three conflicting models of the same 16 mm descent — needs a bench measurement

The same `PickPlan` carries three different durations for the pick-phase descent:

| Source | Value | Model |
|---|---|---|
| `trajectory_pick[0].time_s` (sent to the PLC) | 0.080 s | `max(0.08, Δz / nominal_z_speed)` |
| `plan.descend_time_s` | 0.390–0.683 s | S-curve + soft start (+ slant if oblique, T2) |
| PLC State 10 (`plc/motion-fbs.md` §2.1) | 0.080 s | 20-scan linear joint ramp |

These disagree by a factor of ~5 and nothing in software resolves it. If State 10 is the whole
descent, it demands ~200 mm/s average from a standing start in 80 ms — a real servo almost
certainly lags that, by an amount nobody has measured either.

**Resolution — one measurement settles it** (§3.1): run `production` with the belt stationary
and read `dispatch_to_contact_s` from `[GATE]`. ≈0.08 s + wire time confirms State 10 is the
whole descent; ≈0.39 s confirms the S-curve model. This also decides T1's $\Delta t$ and
whether oblique descent (§4.5) is worth re-enabling.

### T4 — Planning-feasibility cliff near 120 mm/s — independent of T1

T1 is a linear-in-$v$ error; it does not by itself explain a *cliff*. Sweeping the shipped
`_predict_realtime_pick_position` for the latest object
position at which a plan (from the QFP bin) is still accepted:

| Belt speed | Latest plannable $u$ | Margin over workspace edge ($u_\min=188$) |
|---|---|---|
| 60 mm/s | 270 mm | +82 mm |
| 90 mm/s | 226 mm | +38 mm |
| 120 mm/s | 180 mm | **−8 mm** |
| 150 mm/s | 134 mm | −54 mm |

From 120 mm/s the object must be committed to **before it enters the workspace**, but the main
loop only plans when idle — whether a given board gets planned in time becomes a function of
the previous pick's phase. Every accepted plan also parks with only ~80 ms of margin over the
travel-time estimate (`intercept_lead_time_s = 0.8` is never binding because travel time
always exceeds it), and the gate is **one-sided** (`reached = u_now ≥ threshold`, no late
bound, no abort-on-overshoot) — model error beyond that 80 ms goes straight into pick error
with no ceiling.

**Fix** (medium-term, `open-issues.md` §C): plan while the object is still under the camera
(pipeline planning, not only when idle), or accept a speed ceiling near 100–110 mm/s and let
the adaptive law (§6) hold the belt under it.

### T5 — Trajectory time model has no backward pass; neither does the PLC — cross-ref P7

`_trajectory_total_time` chains segment exit speeds
forward only. Measured effect: the model enters a final descent segment (12 mm goto / 5 mm
pick) at 261 mm/s, a speed that segment cannot stop from within its own length (needs 34 mm to
stop from 261 mm/s at `a_max=1000`), so the model runs ≈0.171 s long per cycle versus a
two-pass (backward-checked) model.

This is not purely a PC-side bug: `plc/version-diff-and-defects.md` **P7** (fault 2) confirms
the deployed `MC_Inter_Curve_Vel` FB is itself forward-only — the PLC is *commanded* to do the
physically impossible at the end of every descent, and its real behaviour there (overshoot,
setpoint snap-back) is undocumented and unmeasured. The sandbox's two-pass model is the
*correct* one; the shipped PC model matches the PLC's own bug rather than physical motion.

**Fix**: port the sandbox's backward pass into `_trajectory_total_time`/`_corner_v_end` so the
PC's *prediction* is correct; the PLC's own behaviour is a separate fix (P7 patch, requires a
Sysmac data trace — `plc/version-diff-and-defects.md` §5 bench check 5).

### T6 — Adaptive-speed commit right at the grip instant — cross-ref L9

`execute()` clears `gate_critical` and calls `_commit_adaptive_speed` **immediately after**
dispatching the pick packet (old `scheduler.py:1044-1053`),
i.e. exactly when the cup begins the dead-time chain the lead assumed a steady belt for.
The runtime-sequencing test recorded the dispatch order goto → rotate-home →
pick → **change_speed** → post-grip rotate. Each such commit can drift the belt up to ~3.7 mm
before contact. This is the concrete instance of `open-issues.md` **L9**'s general warning.

**Fix**: move the grip-instant commit to *after* contact is observed (or after the modelled
$t_d$), not immediately on dispatch.

### T7 — `gate_critical` is not enforced while waiting for the goto — cross-ref L9

The critical-window guard (`remaining_mm ≤ v · _GATE_CRITICAL_LEAD_S` with
`_GATE_CRITICAL_LEAD_S = 2.0`) is computed only inside
`_wait_for_object_arrival`. While the arm is still flying to
its park position (`_wait_for_arm_arrival`), `_commit_adaptive_speed` runs unconditionally
every poll with no critical-window check at all. `test_runtime_sequencing` recorded a commit
with the object only 43 mm from the gate (critical window at that speed is 240 mm); its 0.9 s
settle ramp cannot finish before the gate fires 0.36 s later. This breaks the "belt is steady
at gate-fire time" guarantee `basis-theory.md` §6.5 states.

**Fix**: gate every `_commit_adaptive_speed` call site on the object's distance to threshold,
not only the ones inside the pick-gate wait loop — or set `gate_critical` as soon as a plan is
built and the object is already inside the critical window.

### T8 — Belt-position sample timestamped before the PLC round trip

`ConveyorSpeedSource.sample` records `now` (passed in by the
caller, taken *before* the blocking `request_status()` call) against the position read back
*after* the round trip completes. Camera-latency backdating (`position_at`, §4.2) therefore
reads a systematically early belt position by up to one RTT. Measured: +6.0 mm at 120 mm/s
with a 50 ms RTT.

**Fix**: timestamp after `request_status()` returns (or at the round-trip midpoint) before
writing to `decoder`/history.

### T9 — Fictional pose recorded after a failed pick

In the realtime loop, when `executor.execute()` returns failure, `scheduler.current_position`
is set to the **planned** end of the goto trajectory, whether or not the arm ever reached it
(`_run_realtime_pick_loop`, the `else` branch after `if success: ... else: goto_end = ...`).
The next plan is then built from a pose the arm was never at.

**Fix**: read `state.robot_pose` (the last real PLC-reported pose) instead of the planned
trajectory endpoint.

### (Related, already registered) `pick_arrival_tolerance_max_mm = 50` — see `open-issues.md` **G6**

At ≥100 mm/s the arm is accepted as "arrived" up to 50 mm from its park point. If the motion
model is off, the gate starts tracking the object while the arm is still moving, and the
descent absorbs that lateral error too. Confirmed: tightening to 5 mm *before* T1/T2 are fixed
makes hit rate worse (11%), so the band must stay wide until those are closed — this is
symptom management, not a separate root cause.

## 3. Fix roadmap

### 3.1. One measurement resolves T1/T2/T3 (needs hardware, ~15 min)

Run `production` with the **belt stationary**, read `dispatch_to_contact_s` from `[GATE]`
over several picks. ≈0.08 s + wire time ⇒ State-10 model correct, revise `descend_time_s`
down to that; ≈0.39 s ⇒ S-curve model correct, `plc/motion-fbs.md` §2.1 needs a note that the
real servo lags the commanded ramp. Either result fixes the $\Delta t$ to add in T1.

### 3.2. Fix the calibration instrument before trusting any calibration (T2)

Report vertical-descent `t_d_model_s` when oblique is off. No `robot_movement_delay_s`
derived from `[GATE]` before this fix should be kept.

### 3.3. Pay the descent time and collapse the lead to one measured constant (T1)

After 3.1–3.2, set `threshold = u_pick − v_belt · (T_delay + t_soft + t_d + sampling)` for
vertical descent. Then calibrate: run at two belt speeds (e.g. 60 and 120 mm/s), measure mean
error at each, solve for one combined $\Delta t$ — more accurate than summing guessed terms,
because the error is pure gain (`error = v · Δt`, confirmed in simulation E2). Acceptance:
mean error at 60 and 120 mm/s within 5 mm of each other (residual slope ⇒ still-unpaid time).

### 3.4. Logic fixes that need no calibration (T6–T9)

Independently fixable now, no hardware measurement required.

### 3.5. Medium-term, after 3.1–3.3

- Re-tighten `pick_arrival_tolerance_*` only with data, once T1/T2 are closed.
- Re-consider oblique descent only with a measured $t_d$ from 3.1.
- T5's PLC-side half needs the ST patch in `plc/version-diff-and-defects.md` §4 (P7); a
  config-only palliative (raising `slope_transition_height`/`clearance_height` toward
  −245/−230 to slow the final approach) is worth trying once as a *diagnostic*, not a fix.
- T4's cliff is a feasibility limit, not a calibration: needs pipelined planning or an
  enforced speed ceiling from the adaptive law.

### 3.6. Constraints for the scheduler/speed-controller redesign (phase NEAR)

Keep or replace with an equivalent:

1. Encoder-anchored tracking (`basis-theory.md` §4.1) and the live positional gate (§4.4) —
   do not revert to clock-based firing.
2. The "belt steady across the gate window" guarantee — a redesigned speed controller that
   commits more freely must carry an equivalent suppression rule, or implement the
   accelerating-belt lead form already derived in §4.4 (`open-issues.md` **L9**).
3. Processing time $p_j$ is *observed*, not controllable (`open-issues.md` **L1**), and is
   bounded below by the dead-time chain measured here — any new objective function must take
   it as a hard constant, not a decision variable.
4. Policy comparisons must run on the production code path; the PLC simulator
   (`plc/simulator.md`) and the sandbox, which runs the production plugins and arm model
   (`sandbox/README.md`), are what make that possible offline.

## 4. Cross-repo comparison (for context)

The research repository `../python for scheduling` implements the same geometry (17/17 automated checks
match to float precision: 7-point template, coordinate transforms, per-segment time formulas)
but differs in exactly the two places it does better: its `contact_delay_s` includes all three
dead-time terms (T1), and its `segment_schedule` has the backward pass (T5). Both differences
are deliberate design corrections documented in its own source, not accidental drift.
