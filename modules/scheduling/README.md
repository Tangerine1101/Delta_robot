# modules/scheduling — which part next, how fast the belt runs

This package decides; it never acts. The realtime loop (`modules/runtime`) and the offline bench
(`sandbox/`) call the same functions, so a plugin written here runs on the robot and in the
sandbox without change. Nothing here knows about threads, PLCs or the camera.

```
arm becomes free ─┐
                  ▼
   gate open? ──► speed law(view) ──► commit (band, step, deadband)     "how fast"
                  ▼
   planner(jobs, ctx) ──► first entry that still validates ──► pick     "which part"
                  │
   cup contact ───┴──► gate open? ──► speed law ──► commit
```

There are three kinds of plugin. Each is one decorated function; config selects it by name.

| Kind | Decorator | Receives | Returns | Config key |
|---|---|---|---|---|
| Dispatch rule | `@dispatch_rule("name")` | feasible `jobs`, `ctx` | one `Job` or `None` | `scheduling.planner` |
| Sequence planner | `@planner("name")` | all `jobs`, `ctx` | `list[ScheduledPick]` | `scheduling.planner` |
| Speed law | `@speed_law("name", default_gate=...)` | a `SpeedView` | `SpeedDecision` | `speed.law` |

A fourth, the **setpoint gate** (`gates.py`), decides *when* a speed law runs. Most laws use a
built-in gate; see below.

`python3 -m modules.scheduling` lists everything registered.

---

## 1. The data a plugin sees (`types.py`)

**`Job`** — one part, already costed by the arm model from the arm's current position:

| Field | Meaning |
|---|---|
| `obj` | `ObjectSnapshot`: `object_id`, `object_type`, `u`, `v` (belt mm), `bin`, `detected_at` |
| `release` | r_j — when the camera first saw it |
| `arrival` | a_j — when it enters the workspace (now if already inside) |
| `deadline` | d_j — when it leaves the workspace at the forecast belt speed |
| `deadline_safe` | d_j minus `scheduling.safety_margin_s` |
| `processing` | p_j — arm occupancy for this pick from where the arm is now (inf if unreachable) |
| `path_mm` | arm → contact → bin path length |
| `intercept` | where and when the cup would meet it (`pick_time`, `gate_fire_time`, ...) |
| `feasible` | the arm can pick it at all from here |
| `slack` | `deadline_safe − intercept.pick_time` |

**`PlanContext`** (`ctx`) — `now`, `arm_position`, `forecast` (the belt), and three questions a
planner that walks a sequence may ask:

```python
ctx.predict(obj, position, start)            # Intercept if the arm leaves `position` at `start`
ctx.costs(obj, position, start, intercept)   # (grab_s, occupancy_s)
ctx.next_position(obj)                       # where the arm is after placing obj
```

`jobs.extend(job, position, free_at, taken, ctx)` wraps the three: it appends one pick to a
partial schedule and returns `(free_at, position, ScheduledPick)` or `None` when the pick would
be late. Every sequence planner in `planners.py` is built from it.

**`SpeedView`** — what a speed law may read:

| Field | Meaning |
|---|---|
| `now`, `event` | the decision instant and what opened the gate |
| `setpoint_mm_s`, `measured_mm_s` | last commanded and measured belt speed |
| `band` | (v_min, v_max) every commit is clamped to |
| `max_step_mm_s`, `within_step(v)` | the step one commit may take |
| `static_mm_s` | `speed.static_mm_s` |
| `objects_u`, `n_in_window` | belt position of every unclaimed part between u = 0 and u_max |
| `workspace_window_uv`, `arm_cycle` | the workspace and the assumed pick-cycle times |
| `detection_times` | first sighting of each recent part |
| `forecast_for(v)` | the belt forecast if the setpoint became `v` |
| `schedule_for(forecast, planner=None)` | the schedule a planner makes under that forecast |

---

## 2. A dispatch rule (start here)

A rule picks one part from the feasible jobs. The registry turns it into a full ranking by
applying it again to the remaining jobs, so the loop can fall back to the next part when the
first no longer validates. It also exists as a rollout planner, `<rule>_rollout`.

```python
# modules/scheduling/rules.py
@dispatch_rule("nearest_deadline_first")
def nearest_deadline_first(jobs: list[Job], ctx: PlanContext) -> Job | None:
    """The part that leaves the workspace soonest, ties to the shorter path."""
    return min(jobs, key=lambda job: (job.deadline, job.path_mm), default=None)
```

```bash
python3 -m sandbox run --set scheduling.planner=nearest_deadline_first
```

## 3. A sequence planner

A planner returns the parts it intends to pick, in order, with predicted times; the parts it
leaves out are abandoned on purpose. The loop commits only the first entry and re-plans when
the arm is free again.

```python
# modules/scheduling/planners.py
@planner("greedy_on_time")
def greedy_on_time(jobs: list[Job], ctx: PlanContext) -> list[ScheduledPick]:
    """Walk the belt order; keep a part when it can still be picked on time."""
    free_at, position, taken, schedule = ctx.now, ctx.arm_position, frozenset(), []
    for job in sorted(jobs, key=lambda j: j.arrival):
        step = extend(job, position, free_at, taken, ctx)
        if step is None:
            continue                      # late from here: leave it
        free_at, position, pick = step
        schedule.append(pick)
        taken |= {job.obj.object_id}
    return schedule
```

A planner with parameters declares a dataclass; its fields are read from
`scheduling.planners.<name>` and passed as a third argument:

```python
@dataclass(frozen=True)
class GreedyConfig:
    max_parts: int = 10

@planner("greedy_on_time", config=GreedyConfig)
def greedy_on_time(jobs, ctx, cfg: GreedyConfig): ...
```

```yaml
scheduling:
  planner: greedy_on_time
  planners:
    greedy_on_time: {max_parts: 6}
```

## 4. A speed law

A law proposes a target speed. It never sends it: the controller clamps it to the band, limits
the step, holds it inside the deadband and — whatever the law or the gate say — never moves the
setpoint between goto dispatch and cup contact (open-issues L9).

```python
# modules/scheduling/speed_laws.py
@dataclass(frozen=True)
class QueueLengthConfig:
    mm_s_per_part: float = 10.0

@speed_law("queue_length", default_gate="arm_free", config=QueueLengthConfig)
def queue_length(view: SpeedView, cfg: QueueLengthConfig) -> SpeedDecision:
    """Slow down by a fixed step for every part waiting on the belt."""
    v_min, v_max = view.band
    return SpeedDecision(v_max - cfg.mm_s_per_part * view.n_in_window, {"n": view.n_in_window})
```

```yaml
speed:
  law: queue_length
  laws:
    queue_length: {mm_s_per_part: 8}
```

A model-based law scores candidate speeds with the planner (`predictive_rank` is the worked
example):

```python
schedule = view.schedule_for(view.forecast_for(v))       # the plan if the belt ran at v
score = picks_within(schedule, view.now + horizon_s)
```

Return the winning schedule in `SpeedDecision(schedule=..., schedule_planner=...)` and the loop
executes it instead of planning again — the executed schedule is the scored one.

`deadband=False` in the decorator is for a law whose candidates are already discrete and
step-limited, where the deadband would suppress a move the law scored.

## 5. Setpoint gates

| Gate | The law runs |
|---|---|
| `never` | never (the `constant` law) |
| `arm_free` | when the arm becomes free, then every `speed.control_period_s` while it stays free |
| `at_contact` | as `arm_free`, plus the instant the cup touches the part |
| `periodic` | every `speed.control_period_s`, also while a part is carried to the bin |
| `always` | at every decision point |

A law names its default gate; `speed.setpoint_gate` overrides it. No gate is ever asked between
goto dispatch and cup contact.

## 6. Checking a plugin

* Unit test: `tests/test_scheduling.py` builds toy jobs with `tests/helpers.toy_jobs` — a
  textbook instance with fixed processing times, where exact planners must match brute force.
* Offline: `python3 -m sandbox run --set scheduling.planner=<name>` or a sweep spec in
  `sandbox/experiments/`.
* On the PLC simulator: `python3 main.py --scheduler --sim --set scheduling.planner=<name>`.
