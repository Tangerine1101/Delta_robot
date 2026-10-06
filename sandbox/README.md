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
```

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

Not modelled: the camera's latency and noise, stacking, the PLC's own motion defects
(`plc_sim` has those), and the real motion finishing earlier than the time model (G8).

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
