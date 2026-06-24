# Rebuild: Real-Time Two-Thread Pick Scheduler (fresh start)

## Context

**Why.** The multi-object pick keeps missing at density. Root cause, now backed by data
measured from `test_conveyor.log` + the real timing functions:

- The belt-speed estimate the scheduler consumes swings **31–89 mm/s** (p50 46, p95 79) while the
  commanded belt is a constant **50 mm/s** — i.e. **±60% encoder/EMA noise**.
- The current predictor `_predict_pick_position` freezes **one** noisy belt-speed sample and
  multiplies it over a long horizon to place an **absolute downstream park point** `u_park`. A
  single 80 mm/s sample places `u_park` ~35 mm too far downstream → the arm **arrives and then
  waits** (mistimed) → the object never lines up → miss. This is the "arrive-then-wait-then-miss"
  the user observed, and it survived removing the Layer-2 re-prediction because the error is in
  *where the arm parks*, not *when it triggers*.
- Per-cycle flight is only ~2.0 s / 103 mm of belt travel (window is 175 mm wide, dwell 3.5 s), so
  flight is **not** the limiter — the injected, mistimed wait is.
- `task_state` is stuck at `2` in every status frame, so completion already relies solely on
  `pos_EE`.

**User directive (fresh start, no patching).** Tear out the old real-pick decision/wait/execute
flow and rebuild it as:

1. **(1.a)** Raise `intercept_lead_time_s` to **1.6 s**; **keep** the iterative position solver
   `_predict_pick_position`.
2. **(1.b)** Pick priority: objects in the **danger zone** (downstream 1/3 of the workspace) are
   highest priority; next, objects whose pick point is **closest to their own bin**.
3. **(1.c)** After solving pick position + time, build the **gotoplan** and **pickplan**.
4. **(2.a)** **Two threads**: a *main* thread (select → predict → build plans → dispatch →
   execute) and a *perception* thread (update **all** object positions + belt + robot pose in real
   time).
5. **(2.b)** After the arm reaches the pick point, **do not compute the object's arrival time**.
   Simply **wait until the object reaches the arm's current pick position**, then run the pickplan
   (closed-loop on the live object — immune to the ±60% speed noise).
6. **(3)** Reserve an **empty offset hook** (belt-speed + network/mechanism latency). Leave it
   returning 0 for now — negligible at 50–100 mm/s; it grows in production.

**Rules from the user.**
- Except explicitly-kept pieces (the iterative predictor, the trajectory/packet geometry), the
  rewritten parts **must not reuse old code** — reusing the old wait/execute/select logic risks
  reintroducing the bug and wastes compute. Implement them as **new** functions/classes.
- **Minimize PLC changes** — this plan needs **zero PLC changes** (all PC-side).

> First implementation step (per user): author this design as a detailed English report at
> **`doc/realtime_pick_redesign.md`** before/with the code, then keep it in sync.

---

## Architecture overview

```
            ┌──────────────────────────── shared state (guarded) ───────────────────────────┐
            │  belt_position_mm, belt_speed_mm_s   (BeltPositionTracker, updated by P)        │
            │  robot_pose pos_EE                   (last_status, updated by P)                │
            │  BeltTracker objects[]               (ingest/prune by P; read/claim by M)        │
            │  claimed_object_ids                  (set by M, honoured by P's prune + select)  │
            └────────────────────────────────────────────────────────────────────────────────┘
   Thread P (perception/state)                         Thread M (decision + execution)
   loop @ ~25 ms:                                      loop:
     with ipc_lock: sample = speed_source.sample()       snapshot objects (state_lock)
       → 1 IPC read: belt + pos_EE                        select highest-priority unclaimed (1.b)
     update belt pos/speed + pose (state_lock)            predict (kept solver + 1.6s floor/clamp)
     poll vision → tracker.ingest (state_lock)            build gotoplan + pickplan (1.c)
     prune non-claimed past u_max (state_lock)            mark object CLAIMED (state_lock)
     emit dashboard events                                with ipc_lock: dispatch(gotoplan)
                                                          wait arm arrival  (read shared pos_EE)
                                                          wait OBJECT arrival (read shared tracker, 2.b)
                                                          with ipc_lock: dispatch(rotate, pickplan)
                                                          wait place done   (read shared pos_EE)
                                                          unclaim, advance arm current_position
```

**Two locks (PC-side, new):**
- `ipc_lock` (`threading.Lock`): wraps **every** `dispatch()` / `request_status()` round-trip.
  This is mandatory — `_wait_for_response` (`main.py:138`) **drains and discards** responses whose
  `req_id` doesn't match, so two concurrent IPC callers eat each other's replies. The lock keeps
  exactly one round-trip in flight. Round-trips are ~ms, so contention is negligible (P reads
  ~25 ms; M dispatches a few times per pick).
- `state_lock` (`threading.Lock`): guards `BeltTracker` mutation (P ingest/prune) vs M read/claim,
  and the belt/pose snapshot. Reads take a short critical section (snapshot then release).

**Why 2 threads are now safe** (they weren't before): the perception thread owns **all** IPC
*reads*; the main thread only **dispatches**. A single `request_status()` already returns belt
position *and* `pos_EE` together (`ConveyorSpeedSource.sample`, `scheduler.py:509`), so the wait
loops on thread M read pose/object state from shared memory and issue **no IPC of their own** —
removing the wait-loop IPC contention entirely. The `ipc_lock` covers the only remaining overlap
(P's periodic read vs M's occasional dispatch).

---

## Component design

### A. Shared state container (new)
A small `RealtimeState` holding: `position_mm`, `speed_mm_s`, `pos_EE`, a reference to the
`BeltTracker`, `claimed_object_ids: set[str]`, the two locks, and a `stop` event. One instance,
shared by both threads. (New code; does not reuse the old `on_idle`/`perceive_tick` pump.)

### B. Thread P — perception / real-time position updater (2.a) (new)
Loop @ ~25 ms until `stop`:
1. `with ipc_lock: sample = speed_source.sample(now)` — single IPC read → belt + `pos_EE`.
2. `with state_lock:` store `position_mm`, `speed_mm_s` (from `BeltPositionTracker`), `pos_EE`
   (from `speed_source.last_status`).
3. `detections = image_processing.poll(now)`; `with state_lock: tracker.ingest_detection(...)` for
   each (reuse the **kept** `BeltTracker.ingest_detection`).
4. `with state_lock: tracker.prune(...)` **but skip claimed ids** (so an in-flight target is never
   pruned out from under thread M). Needs a small prune variant / claimed-aware guard (new).
5. Emit `[SPEED]`/`[DETECT]` + dashboard `event_sink` (rewritten, not the old `perceive_tick`).

> Object positions are *derived on read* via the kept `TrackedObject.current_uv(p_now)` /
> `BeltTracker.current_position_R` — "updating all objects in real time" = keeping `position_mm`
> and the vision anchors fresh, which this loop does.

### C. Thread M — decision + execution (main thread) (new loop + new executor)
Replaces both the old `run_scheduler_scenario` main loop (for real scenarios) and the old
`RealRobotExecutor.execute`. New `RealtimePickExecutor` (or inline functions). Per iteration:
1. `with state_lock:` snapshot tracked objects + belt pos/speed.
2. **Select** highest-priority unclaimed, catchable object (§E).
3. **Predict** pick position/time (§D).
4. **Build** gotoplan + pickplan (§F) — reuse kept `_build_pick_plan` geometry/packets.
5. `with state_lock: claimed.add(object_id)` (do **not** remove from tracker — M must keep
   tracking it to detect arrival).
6. `with ipc_lock: dispatch(goto_packet)`.
7. **Wait arm arrival**: poll shared `pos_EE` until it converges (`position_tolerance_mm`) on the
   goto end waypoint, with a `wait_margin_s` ceiling. No IPC here — reads shared pose (new code,
   not the old `_wait_for_phase_completion`).
8. **Wait object arrival (2.b)**: poll the claimed object's live `u = current_uv(belt_pos)` from
   shared state; fire when `u >= u_pick - _belt_lead_offset_mm(speed)` (§G). Pure position gate, no
   time math, no IPC. Safety ceiling so a stalled belt can't hang.
9. `with ipc_lock: dispatch(rotate); dispatch(pick_packet)`.
10. **Wait place arrival**: poll shared `pos_EE` to the place waypoint (ceiling-bounded).
11. `with state_lock: claimed.discard(id); tracker.remove(id)`; advance arm `current_position` to
    the bin.

### D. Prediction with the 1.6 s lead (1.a) — keep the solver, add lead in the caller
**Keep `_predict_pick_position` exactly as-is** (the kept iterative solver returns the earliest
goto-feasible pick time/position). The new **caller** applies the lead so the kept function is not
edited:
- `earliest = _predict_pick_position(obj, sample, now)`; if `None` → skip (already past `u_max`).
- `final_t = max(earliest_time, now + intercept_lead_time_s)`  → **1.6 s floor**.
- `u_pick = u_now + speed * (final_t - now)`.
- **Clamp to window** so danger-zone objects are caught at the edge instead of rejected:
  `if u_pick > u_max - edge_margin: u_pick = u_max - edge_margin; final_t = now + (u_pick-u_now)/speed`.
- **Feasibility**: arm arrives at `now + command_delay + goto_total(u_pick)`. If that is **after**
  `final_t` (object reaches `u_pick` before the arm can) → genuinely too late → skip.
- Else pick at `frame.to_robot(u_pick, v)`.

This reconciles **1.a + 1.b**: objects with room get the generous 1.6 s downstream park (arm parks
~27 mm ahead, object arrives from upstream → the wait-for-object model always works); danger-zone
objects clamp to the downstream edge and are grabbed ASAP.

### E. Priority selection (1.b) (new)
Replaces the old `plan_next` "sort by `predicted_pick_time`". Over unclaimed, catchable objects:
- `u_danger = u_min + 2/3*(u_max - u_min)` (= 304.7 mm for the current window).
- **Tier 1 (highest): danger zone** `u_now >= u_danger`, sorted **most-downstream first**
  (closest to exit = most urgent).
- **Tier 2: the rest**, sorted by **distance(pick_R, its sorting bin) ascending** ("closest to its
  own bin" → shortest place leg → faster cycle).
- Skip claimed ids and any object failing the §D feasibility check.

### F. Plan building (1.c) — kept geometry/packets
Reuse the **kept** `_build_pick_plan` → `_build_goto_geometry` / `_build_pick_geometry` /
`_build_*_timing` / `_trajectory_packet` and the kept `PickPlan` / `to_robot_packets`. The goto
ends hovering over `u_pick`; the pickplan descends straight down (gripper ON) → lift → bin → place.
Keep a reference to the live `TrackedObject` on the plan for the §C.8 arrival gate.

### G. Positional pick trigger + empty offset hook (2.b, 3) (new)
```python
def _belt_lead_offset_mm(belt_speed_mm_s: float) -> float:
    # TODO(production): compensate command + network + mechanism latency.
    # Negligible at the nominal 50-100 mm/s belt; intentionally 0 for now.
    return 0.0
```
Arrival gate fires when `object_u >= u_pick - _belt_lead_offset_mm(speed)`.

> Building out a speed-dependent offset model is **deferred** and tracked as a roadmap item —
> see `README.md` §5 Future Roadmap (the single source of truth). The hook stays `0.0` here.

---

## Kept vs rewritten vs removed

**Kept (reused, allowed):** `_predict_pick_position` (the iterative solver), `_build_goto_geometry`
/ `_build_pick_geometry` / `_build_goto_timing` / `_build_pick_timing` / `_segment_*` /
`_trajectory_total_time` / `_trajectory_packet`, `PickPlan` + `to_robot_packets`, `ConveyorFrame`,
`BeltTracker` / `BeltPositionTracker` / `TrackedObject`, `ConveyorSpeedSource`, the IPC worker in
`main.py` (extended with `ipc_lock`).

**Rewritten as new code (no reuse of old flow):** the real-scenario scheduler loop, the executor
(`RealtimePickExecutor`), object selection (priority), arm-arrival + object-arrival waits, the
perception/state thread, the lead/clamp wrapper around the predictor, the offset hook.

**Removed (dead after switch):** `RealRobotExecutor.execute`, `_wait_until_pick_dispatch`,
`_wait_for_phase_completion`, `_pump_and_sample`, `on_idle`; the old `perceive_tick` pump and the
old `PickScheduler.plan_next` time-sort (for the real path).

**Untouched:** the simulated path (`SimulatedExecutor`, `SimulatedSpeedSource`,
`SimulatedImageProcessing`) + `test_throughput` / `test_accuracy` / `evaluate` and their unit
tests — they are a test harness, not the buggy real logic.

## Files

- `modules/scheduler.py` — new `RealtimePickExecutor`, new real-time loop, new selection, the
  lead/clamp wrapper, the offset hook; remove the old real executor + waits.
- `main.py` — add `ipc_lock`; wire real scenarios to the new loop; keep sim wiring.
- `doc/realtime_pick_redesign.md` — **new** detailed English design report (step 1).
- `tests/test_trajectory_planning.py` — drop `PerceptionPumpTests` (old pump gone); add tests for
  the lead/clamp wrapper, the priority selection tiers, and the positional arrival gate (stub
  tracker/belt; no threads needed).
- `modules/config.json` — set `scheduler.intercept_lead_time_s = 1.6`.

## PLC impact
**None.** All changes are PC-side (Python threads + a PC-side IPC lock). The Omron/Siemens packet
contracts, DB offsets, and command IDs are unchanged.

## Verification

1. Compile (CLAUDE.md §3): `python3 -m py_compile main.py modules/cli.py modules/EthernetCom.py
   modules/image_processing.py modules/scheduler.py modules/test_module.py modules/conveyor.py
   modules/interface.py`
2. Sim regression unaffected: `test_throughput` (9/9) + `test_accuracy` clean (`--simulate-executor`).
3. `python3 -m unittest tests.test_trajectory_planning -v` (new selection/lead/gate tests).
4. Real belt, dense burst: `python3 main.py --scheduler --scenario test_conveyor --interface
   --duration 30`. Expect: `[SPEED]`/`[DETECT]` never go dark during a pick (perception thread);
   the arm parks ~1.6 s downstream and **descends straight down when the live object reaches it**
   (no mistimed wait, no sideways jerk); danger-zone objects grabbed first; trailing
   cluster members skipped cleanly (serial arm).

## Flagged interpretations (please confirm at approval)
- **1.b secondary** "closest to its own bin" read as *pick→bin distance ascending* (shortest place
  leg). If you meant something else (e.g. closest to the arm's current position), say so.
- **1.a + 1.b reconciliation**: danger-zone objects are **clamped to the downstream edge** and
  grabbed ASAP (rather than skipped when the 1.6 s lead overshoots `u_max`).
