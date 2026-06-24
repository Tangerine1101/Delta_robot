# GPT Bug Report — Multi-Object Pick Lag

> Date: 2026-06-24  
> Scope: root-cause verification before implementation changes  
> Files inspected: `doc/video_analysis_report.md`, `doc/video_analysis_report_2.md`,
> `doc/bug_report_final.md`, `test_conveyor.log`, `modules/scheduler.py`,
> `modules/conveyor.py`, `modules/image_processing.py`, `modules/config.json`.

## 1. Symptom

Single-object picks are accurate, but in a burst of two or more boards the lead
board is usually aligned while later boards are missed behind/upstream. The miss
is silent: the system still completes the trajectory and increments completed
pick metrics.

Both video reports describe the same pattern:

- The first board in a calm interval is hit on center.
- Later boards in the same burst continue moving while the arm completes the
  previous cycle.
- When the arm returns, it descends onto empty belt behind or between boards.
- The objects are still physically reachable/visible, so this is not primarily a
  workspace-boundary issue.

## 2. Verified Root Cause

The scheduler's main decision loop is blocked by synchronous robot execution.

`run_scheduler_scenario()` performs speed sampling, vision polling, detection
ingest, stale pruning, planning, and execution in one loop. When a plan exists,
it calls `executor.execute(plan)` inline. For the real robot this call covers the
whole `goto -> wait for dispatch -> late re-predict -> pick/place` sequence.
While it is running, the scheduler does not:

- sample the belt encoder,
- poll the vision detection queue,
- ingest or re-anchor detections,
- prune objects that have passed the workspace,
- build plans for newly seen objects.

The vision thread can still see boards during this interval, but its detections
remain queued until the current robot execution returns. This explains the video
symptom exactly: object #2 is detected while object #1 is being handled, but it
is not planned until it has already moved far downstream.

## 3. Log Evidence

From `test_conveyor.log`, the gap between the last `[SPEED]` sample before a
plan enters execution and the first `[SPEED]` sample after its pick phase:

| Plan | Scheduler dark time | Belt displacement |
|------|--------------------:|------------------:|
| `plan-000001` | 6.195 s | 305.4 mm |
| `plan-000002` | 6.603 s | 311.3 mm |
| `plan-000003` | 7.320 s | 303.6 mm |
| `plan-000004` | 3.797 s | 191.3 mm |

The configured workspace window is `u = 188..363 mm`, only 175 mm wide. During a
single blocked execution window the belt often advances farther than the entire
pickable `u` span. Dense bursts therefore cannot be handled by the current
serialized scheduler.

An especially strong trace:

- During `plan-000003` execution, the vision thread logs `NEW id=4`.
- The scheduler does not emit `[DETECT]` / `[PLAN]` for id 4 until after
  `plan-000003` returns.
- This proves perception is alive, but the decision loop is not consuming it.

## 4. Secondary Mechanism

Queued detections are anchored to the current encoder position when the scheduler
eventually polls them.

`VisionImageProcessing` emits detections with their own timestamp, but
`run_scheduler_scenario()` later calls `image_processing.poll(now)` and passes a
single current `sample.position_mm` into `scheduler.ingest_detections()`. The
tracker then sets `belt_pos_anchor = p_now`.

If a detection waited in the queue for several seconds while the robot executed,
old camera coordinates are incorrectly treated as current belt coordinates. This
adds an upstream/behind bias and makes the serialized-loop problem worse.

## 5. Disproven Primary Cause

The PLC Z-clamp / 5 mm tolerance theory is not the primary root cause of the
reported multi-object lag.

The captured log shows pick targets at `z = -300` and pre-pick telemetry near
`z = -290`, which matches an older height configuration, not the current
`pickup_height = -305` / `pre_pick_height = -295` configuration. More
importantly, the observed multi-second gaps align with full synchronous pick
cycles and dispatch waiting, not with an isolated Z tolerance boundary.

The Z tolerance remains a latent robustness risk if commanded pre-pick heights
sit exactly on the physical limit, but it does not explain why first objects hit
and later objects in bursts miss.

## 6. Fix Direction

The primary fix should decouple perception/planning from robot execution:

1. Keep the main scheduler loop polling speed and vision continuously while the
   robot is busy.
2. Track robot busy/idle state separately from detection ingest and planning.
3. Either pre-plan queued objects while the current plan is executing or at
   least maintain live belt-anchored object state so the next plan starts from
   current coordinates.
4. Drop or correct stale queued detections using detection timestamps and encoder
   history before anchoring them to the belt.

After the fix, real conveyor logs should show `[SPEED]` and `[DETECT]` continuing
at the normal loop cadence during robot motion instead of multi-second gaps.
