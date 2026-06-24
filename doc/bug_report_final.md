# Bug Report (Final) — Multi-Object Pick Lag

> **Status:** Verified against code, the `test_conveyor.log` capture, and `git` history.
> **Date:** 2026-06-24
> **Scope:** `modules/scheduler.py`, `modules/conveyor.py`, `modules/config.json`
> **Severity:** High — every second-and-later pick in a multi-object burst is a silent miss.
>
> This file supersedes the two earlier drafts (`bug_report_1.md`, `bug_report_2.md`).
> One of those drafts (the blocking-scheduler analysis) was correct; the other
> (PLC Z-clamp at `-290` + 5 mm tolerance hang) was **disproven** — see §4.

---

## 1. Symptom

- Single-object pick-and-place is reliable.
- With **2+ objects** on the belt at nearly the same time, the **first** pick
  succeeds and **every later** pick lands *behind* the moving part ("grabs air"),
  even though the object is still nominally inside the workspace window.
- The system still runs the full sequence and reports `status="completed"`, so
  the miss is **silent**. Metrics for the captured run: `planned_picks=4`,
  `completed_picks=4`, `queue_peak=1`.

---

## 2. Root Cause (two layers)

The lateness is **not** a stale-math bug. Every plan is re-solved with a fresh
`p_now`, and the dispatch-time B2 re-prediction
([`_repredicted_pick_packet`](../modules/scheduler.py#L701-L769)) refreshes the
pick target from the latest encoder reading. The numbers are right. Two distinct
mechanisms make the **physical arm** arrive late anyway.

### Layer 1 — Blocking, single-threaded scheduler (the lateness itself)

`executor.execute(plan)` runs **synchronously inside the main loop**
([scheduler.py:2140](../modules/scheduler.py#L2140)) and blocks for the entire
`goto → wait-dispatch → pick → place` cycle. While it blocks, the loop performs
**no** belt sampling, vision polling, detection ingest, re-anchoring, or planning
for any other object. Picks are therefore **strictly serialized**.

**Measured from `test_conveyor.log`** (gap between the last `[SPEED]` sample
before a plan and the first `[SPEED]` after its pick — i.e. how long the loop was
frozen):

| Plan | Loop frozen | Plan `duration_s` |
|------|-------------|-------------------|
| 1    | **6.20 s**  | 1.18 s            |
| 2    | **6.60 s**  | 1.18 s            |
| 3    | **7.32 s**  | 1.18 s            |
| 4    | **3.80 s**  | 1.18 s            |

The loop is dark for 4–7 s per pick. During that window object #2 only advances
by dead reckoning ([conveyor.py:293-296](../modules/conveyor.py#L293-L296))
`current_uv = u_anchor + (p_now − belt_pos_anchor)`; when `execute()` returns,
`p_now` has jumped by ~ (belt_speed × 6–7 s). At ~52 mm/s that is **300–360 mm** —
object #2 is at or past the downstream edge `u_max` before its plan can even be
built. `queue_peak=1` confirms objects are never processed in parallel.

**Compounding factor:** plan #2's goto starts from the sorting bin, not home
(log line 493 shows goto-start `pos_EE = [30, 30, -282]` = the place point), so
the next path is longer → later predicted pick time → even less margin.

### Layer 2 — Re-prediction lead-time budget omits horizontal travel

This is why even the fresh dispatch-time correction still lands behind the part.
Both the dispatch gate ([scheduler.py:836](../modules/scheduler.py#L836)) and the
re-prediction ([scheduler.py:730](../modules/scheduler.py#L730)) compute the
contact point as:

```
u_contact = u_now + speed_uv * (command_delay_s + descent_time_s)
```

That budget contains **only two terms**:

- `command_delay_s` = `robot_movement_delay_s (0.05)` + `ethernet_delay_s (0.00016)` ≈ **0.05 s**
- `descent_time_s`  = **vertical** pre_pick→pickup descent only
  ([`_segment_duration`](../modules/scheduler.py#L1540): `|-295 − (-305)| = 10 mm ÷ nominal_z_speed 120`) ≈ **0.083 s**

Total lead ≈ **0.133 s** → only **~7 mm** at a 52 mm/s belt.

The model **assumes the arm is already hovering directly above the object** and
just needs to drop. It does **not** account for:

1. **Horizontal traverse time** from the current pre-pick hover XY to the new
   pick XY. During the goto the arm parked above the *old* predicted point; once
   the object has drifted (the failure case), the pick trajectory
   ([`_build_pick_geometry`](../modules/scheduler.py#L1598)) must move sideways
   before descending. That time is entirely outside the 0.133 s budget.
2. **Omron firmware ignores `argument_time` and runs at a fixed max speed**
   (docstring at [scheduler.py:875](../modules/scheduler.py#L875)). The
   `descent_time` derived from `nominal_z_speed` is only an estimate; if the real
   descent is slower, the lead is short by even more.

Contrast with the **planning-time** predictor
([`_predict_pick_position`](../modules/scheduler.py#L1463-L1485)), which adds the
**full** `sum(goto_times)` flight time + descent + command_delay. That is why
**object #1 hits** — it is flown in with the lead fully modeled. The re-prediction
drops the traverse term, so it is correct only for *small* corrections (object
not drifted). Exactly when object #2 has drifted, it **under-leads systematically**:
dispatch fires when the object is ~7 mm upstream of the parked arm, the arm then
spends ~0.3–0.4 s traversing and descending, the object travels another ~15–20 mm
in that time, and the gripper closes ~8–13 mm **behind** the part. The
`[REPREDICT] delta_u ≈ 6–7 mm` log entries confirm the correction is tiny and
never compensates for the pick's own flight time.

---

## 3. Why the run still logs success

When object #2 is marginally inside the window, B2 passes its window check and
runs the pick, setting `status="completed"`. The spatial check ("is it in the
window now?") is not a temporal feasibility check ("will the arm reach it before
it moves on?"). When the object has already passed `u_max`, B2 instead returns
`None` → `pick_aborted_outside_workspace`. Both outcomes look benign in logs.

---

## 4. Disproven hypothesis (the rejected draft)

The second draft blamed a **PLC hard clamp at `Z = -290`** that, against the
configured `pre_pick_height = -295`, left a `distance ≈ 5.0002 mm > 5.0 mm`
tolerance so `_wait_for_phase_completion` allegedly hung to its hard deadline
every phase. This is **not** what happened:

1. **The log predates the current config.** The captured log shows predicted
   `pick z = -300` and goto-end `pos_EE z = -290`. `git` history shows commit
   `2461c8f` set `pickup_height = -300, pre_pick_height = -290`, and commit
   `e2bfb75` (the same day this analysis was done) changed them to `-305 / -295`.
   So at capture time the goto target Z *was* `-290`, and the arm reached it
   **exactly** → `distance ≈ 0`, not 5 mm. No tolerance hang occurred in the log.
2. **`-290` is the pre-pick hover height, not a hardware clamp.** There is no
   evidence of a clamp.
3. **Logical contradiction:** a real clamp at `-290` would stop the arm reaching
   the `-300/-305` pick depth, so *no* pick — including the first — could ever
   succeed. The agreed symptom is that the first pick is reliable, which refutes
   the clamp.

A genuine *latent fragility* the draft accidentally surfaced: with the current
config the goto target is `-295`, and `_wait_for_phase_completion` uses
`distance > position_tolerance_mm` with a default tolerance of exactly `5.0`
([main.py:339](../main.py#L339)). If a future run ever parks the arm at `-290`
while commanding `-295`, the ~5.0002 mm vs 5.0 mm boundary is dangerously tight.
Worth hardening, but it is **not** the cause of the observed misses.

---

## 5. Suggested Remediation (not implemented)

1. **Primary — decouple execution from perception/planning.** Run
   `executor.execute` on a separate thread/process so the main loop keeps
   sampling the belt, polling vision, re-anchoring, and pre-planning object #2
   *while* the arm still handles object #1. This removes Layer 1.
2. **Fix the re-prediction lead budget (Layer 2).** Include the actual
   horizontal pick-traverse time in `u_contact` — e.g. iterate to convergence the
   way `_predict_pick_position` does (add the rebuilt pick-phase travel time, not
   just `command_delay + descent_time`). Prefer empirically measured arm speed
   over `nominal_*_speed`, since the Omron ignores `argument_time`.
3. **Defensive — widen `position_tolerance_mm`** (or, equivalently, keep
   `pre_pick_height` reachable) so the exactly-5.0 mm boundary in
   `_wait_for_phase_completion` can never deadlock to the hard deadline.

---

## 6. Verification

```bash
# Compile check (per CLAUDE.md §3)
python3 -m py_compile main.py modules/cli.py modules/EthernetCom.py \
  modules/image_processing.py modules/scheduler.py modules/test_module.py \
  modules/conveyor.py modules/interface.py

# Real belt with >=2 objects — after a fix, the [SPEED] stream must NOT freeze
# for seconds around each plan, and [REPREDICT] delta_u must track the live
# object position rather than lagging it.
python3 main.py --scheduler --scenario test_conveyor --interface --duration 30
```
