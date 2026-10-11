# sandbox/ — offline algorithm bench

Runs the robot's own scheduling plugins (`modules/scheduling`) and arm model
(`modules/core/delta.py`) in simulated time, fast enough for sweeps of hundreds of runs.

| Mode | Question it answers |
|---|---|
| `main.py --scheduler --sim` (`modules/plc_sim`) | Is the **code** right? Commands, threads, the PC ↔ PLC contract, in real time |
| `python3 -m sandbox` | Is the **algorithm** good? Many seeds, many cases, in seconds |

The sandbox imports `modules.core`, `modules.scheduling` and `modules.settings` only; nothing
under `modules/` imports it.

## Running

```bash
python3 -m sandbox list                               # planners, speed laws, gates, feeders, arm models
python3 -m sandbox run                                # one run of sandbox/config.yaml
python3 -m sandbox run --set scheduling.planner=dp --set speed.law=predictive_rank
python3 -m sandbox run --profile                      # where the time goes
python3 -m sandbox sweep sandbox/experiments/speed_laws.yaml [--workers N]
python3 -m sandbox check                              # model files vs modules/config.yaml
python3 -m sandbox run --out DIR --set sim.stacking=true ...   # also write parts.csv, flow.csv, waits.json
python3 -m sandbox dips DIR|parts.csv                 # where the output left the input (robot logs too)
python3 -m sandbox exp --list                         # named experiments, each with a report of tables + charts
bash sandbox/run_experiments.sh phase0|phase1 [--quick]   # the pre-cell experiments, one command
```

Experiments (how to run, read and write one): [experiments/README.md](experiments/README.md),
Vietnamese [experiments/README.vi.md](experiments/README.vi.md). `exp` runs `sweep` specs and
`replay` specs (a logged cell run replayed on its own arrivals and compared with the cell,
`replay.py`); `report DIR` redraws the tables and charts of a finished sweep (`report.py`).
Always use `.venv/bin/python` (the system Python lacks `ruamel`).

A sweep writes `sandbox/results/<spec>-<timestamp>/runs.csv` (one row per run) and
`manifest.json` (git revision, spec, resolved config of every case). `results/` is not tracked.

## Configuration

* `config.yaml` — one run: `sim` (clock, capture tolerance), `feeder`, `arm` (which model file,
  plant deviations) and the same `scheduling:` / `speed:` sections as `modules/config.yaml`.
* `models/<name>.yaml` — one arm: `model` (the registered builder), `params` (the builder's own
  parameters), `plant` (dotted overrides that turn the model into the plant) and `cell` (robot,
  conveyor, object types, pick gate — the shape of `modules/config.yaml`).
* `experiments/<name>.yaml` — `base` overrides, `seeds`, and a `grid` and/or named `cases`.

Every file goes through the strict loader of `modules/settings.py`: an unknown key is an error.

## What a run models

Parts land at u = 0 (the camera origin) from the feeder and are known to the planner at once.
The decision cycle is the robot's: while the arm is free, every `sim.replan_period_s` the speed
law runs at its setpoint gate, the planner makes a schedule and the first entry that still
validates is committed. The **plant** then flies the goto, the gate fires at
`u_pick − v·lead` (the **model's** lead), and the cup comes down after the plant's contact
delay: the part is gripped if it is within `sim.capture_tolerance_mm` along the belt. The
setpoint is frozen from goto dispatch to contact (open-issues L9).

`sim.stacking: true` models stacking like the robot's `simulate_feeder`: a part landing on
parts in the camera view covers those it overlaps (footprints from the feeder's headings and
the part sizes); with `scheduling.stack_aware` jobs carry blockers, and a cup that comes down
on a part still covered is booked a miss (`covered_misses`). As on the robot only a *picked*
part frees what lies under it.

`feeder.max_parts` (default 0 = none) keeps the first parts of the feeder, as the robot's key does; set
`sim.duration_s` long enough for the tail. Every run also records the wall time of each speed-law
decision and commit planning (`law_ms_*`, `plan_ms_*`, queue length `plan_jobs_*`).
`python3 -m sandbox confirm` selects levels on some seeds and confirms them on others
(`confirm.py`; experiments `p1_*.yaml`, `doc/experiment_notes.md`).

The plant's leg times are the trajectory model's plus a fixed overhead, identified from the cell's
pose stream (`python3 -m modules.tools.identify_plant log/<session>`): `sim.goto_extra_s`
(+0.005 s) and `sim.pick_extra_s` (-0.019 s), set in `config.yaml`. `null` adds the model's
command delay (0.228 s, the scheduler's lead for the gate) to each leg instead, which makes
the arm 0.28 s per pick slower than the cell. `sim.carry_scale` (default 1) scales the time from
gate fire to arm free: a what-if. `experiments/phase0_replay*.yaml` check the result against
logged cell runs.

Starvation is measured, not eyeballed. A commit whose contact is more than `sim.long_wait_s`
away is a **long wait**; every other part the arm could have picked sooner from where it stood
is **bypassed** (`bypassed`), and one that then left the workspace unpicked is `bypassed_lost`.
`dips` / `dip_s` count the stretches where the trailing-60 s throughput left the input
(`sandbox/flow.py`, the same finder reads the `parts.csv` of a robot run: the dips of the
dashboard's *Input and throughput* chart). `waits.json` lists each long wait with the parts
passed over. To replay a logged run exactly: `feeder.kind: script` with
`feeder.script.arrivals_csv: <run>/arrivals.csv`. `experiments/starvation.yaml` is the
one-hour seed-110 case of the 2026-10 issues.

Not modelled: the camera's latency and noise, the PLC's own motion defects (`plc_sim` has
those), the real motion finishing earlier than the time model (G8), and the robot's absolute
clock (the float-edge flicker of the intercept window depended on it; `tests/test_intercept_edge.py`
pins that one).

## Model and plant

The scheduler plans with the model; the simulated arm moves like the plant. Give them different
parameters to measure what a wrong estimate costs:

```yaml
arm:
  model: delta
  plant: {pick_gate.robot_movement_delay_s: 0.22}   # the real delay is 50 ms longer than modelled
```

A `params.` prefix reaches the model's own parameters, e.g. `params.gate_sampling_latency_s`.

## Adding things

* **A dispatch rule, planner or speed law:** one decorated function in `modules/scheduling`
  (see its README). It is available to the robot and the sandbox at once.
* **A feeder:** one function in `modules/core/feeders.py` decorated with
  `@feeder("<name>", <ConfigClass>)`; its parameters are `feeder.<name>` in the run config. The
  robot's `simulate_feeder` scenario draws its virtual parts from the same functions.
* **An arm model:** a class implementing `modules.core.arm_model.ArmModel` and a builder
  decorated with `@arm_model("<name>", config=<ConfigClass>)` in `models/`, plus a
  `models/<name>.yaml`. `models/point_mover.py` is a complete example.

## Relation to the published results

The paper's numbers were produced by the research repository (`../python for scheduling`,
frozen at tag `paper-submitted`), whose delta model has a backward pass, different motion
parameters and a different per-segment formula. This bench uses the robot's model instead, so
its absolute numbers differ; compare trends and rankings, and reproduce the paper from the tag.
