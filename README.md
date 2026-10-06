# Delta Robot Pick-and-Place Sorting Cell

Python control software for a delta-robot sorting cell. A control PC runs the vision pipeline
and the pick scheduler, and drives two PLCs over Ethernet:

* **Omron NX1P2** (EtherNet/IP, `pylogix`) — delta arm motion and vacuum gripper.
* **Siemens S7-1200** (Modbus TCP, `snap7`) — conveyor speed and the 4th-DOF suction-cup
  rotation.

Parts (QFP / TQFP boards) are detected on the moving belt by a YOLO-OBB model, tracked against
the belt encoder, intercepted by the arm while still inside the reachable window, and dropped
into per-class bins.

---

## Current phase

The cell is built and operational. Active work is a **research project on online pick
scheduling integrated with conveyor speed control** — each part carries a deadline set by when
it leaves the reachable window, and belt speed is treated as a scheduling decision rather than
a separate control loop. The proposal sources are in [`doc/proposal/`](doc/proposal/).

Planners and belt-speed laws are plugins of one framework
([`modules/scheduling/`](modules/scheduling/README.md)): an algorithm is one function, runs on
the cell and in the offline bench [`sandbox/`](sandbox/README.md) unchanged. Before proposing or
planning any change, read **[`doc/open-issues.md`](doc/open-issues.md)** — the single register
of everything unresolved, including which parameters have never been calibrated.

---

## Setup

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

Requires a CUDA-capable GPU for real-time YOLO-OBB inference, a UVC camera capable of 1080p30
MJPEG, and network access to both PLCs. All settings live in
[`modules/config.yaml`](modules/config.yaml) (every key, its type and default:
[`modules/settings.py`](modules/settings.py)); check `plc` and the `robot` geometry before
running anything against real hardware.

Sanity-check the configuration first:

```bash
python3 calibrate_everything.py --check
```

---

## Running

```bash
# Interactive CLI against real PLCs
python3 main.py --cli

# Interactive CLI against an in-process fake PLC (no hardware)
python3 main.py --cli --dummy

# The real pick loop against the in-process PLC simulator, no hardware
python3 main.py --scheduler --scenario production --sim --duration 60
python3 main.py --scheduler --scenario production --sim --set speed.law=predictive_rank

# Vision only — real camera, no robot, live web dashboard on http://localhost:8000
python3 main.py --scheduler --scenario test_vision_only --interface --duration 20

# Production run on real hardware
python3 main.py --scheduler --scenario production --interface

# Repeatable data runs: seeded virtual parts on the real belt and arm (drop --sim on the cell)
python3 main.py --scheduler --scenario simulate_feeder --sim --duration 120 --set feeder.seed=3
python3 -m modules.tools.flow_report log/          # input vs throughput across every run

# Offline algorithm bench: every planner / speed law in seconds
python3 -m sandbox run --set scheduling.planner=kim
python3 -m sandbox sweep sandbox/experiments/speed_laws.yaml

# Tests (no hardware)
python3 -m unittest discover -s tests -t .
```

`--interface` serves an in-process web dashboard (annotated MJPEG plus live telemetry) and
suppresses the native OpenCV window. Without it, real-camera scenarios open a local overlay
window; set `vision.show_window = false` to run headless.

`--set key=value` overrides one config value for a run; the file is never written. The full
verification command list — tests, simulator runs, calibration probes — is in
[`doc/basis-programming.md`](doc/basis-programming.md) §9.

### Scenarios

| Scenario | Vision | Robot | Conveyor |
|---|---|---|---|
| `production` | camera (simulated under `--sim`) | picks | startup speed + the configured speed law |
| `simulate_feeder` | virtual parts from `feeder.seed` riding the belt encoder | picks | startup speed + the configured speed law |
| `test_vision_only` | camera (simulated under `--sim`) | idle | no command; feedback from the PLC, or static with `--no-plc` |

New scenarios are registered in `modules/runtime/scenarios.py`. Every run with a run log also
writes its part record — `parts.csv`, `flow.csv`, `summary.json`: each part picked, missed on
the grip or missed for time (`doc/basis-programming.md` §2.6).

**Operator console.** `python3 main.py --interface` on its own opens a web console on port
8000 (add `--sim` to drive the simulator). It offers:
- manual moves, pump, belt and cup rotation;
- scenario start/stop, with the planner, speed law and virtual feeder chosen per run;
- the live views: an animated top view of the cell, input and throughput charts, the miss list.

---

## Documentation

Everything technical lives in [`doc/`](doc/) — this README intentionally holds no
configuration reference, roadmap or bug list.

| File | Contents |
|---|---|
| [`doc/context.md`](doc/context.md) | **Start here.** Project phase, document map, directory structure, rules for AI assistants |
| [`doc/basis-theory.md`](doc/basis-theory.md) | Coordinate transforms, delta kinematics, trajectory profiles, tracking and interception, orientation chain, belt-speed laws, planners |
| [`doc/basis-programming.md`](doc/basis-programming.md) | Layers and modules, thread/process architecture, decision cycle, scheduling framework, PLC data contracts, trajectory templates, scenarios, config-key reference, verification commands |
| [`modules/scheduling/README.md`](modules/scheduling/README.md) | Writing a dispatch rule, planner or speed law |
| [`sandbox/README.md`](sandbox/README.md) | The offline algorithm bench |
| [`doc/open-issues.md`](doc/open-issues.md) | Single register of unresolved problems: pending calibration, config inconsistencies, architectural limits |
| [`doc/decision-log.md`](doc/decision-log.md) | History: superseded designs and why they were replaced |
| [`doc/dev-note.md`](doc/dev-note.md) | Developer's bench notes |

`doc/plc/` describes the PLC programs, the PC ↔ PLC data contract and the PLC defect review, and
`doc/Manuals/` the hardware datasheets.
