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

The scheduler and belt-speed controller currently in the repository are the **baseline being
redesigned**. Before proposing or planning any change, read
**[`doc/open-issues.md`](doc/open-issues.md)** — the single register of everything unresolved,
including which parameters have never been calibrated.

---

## Setup

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

Requires a CUDA-capable GPU for real-time YOLO-OBB inference, a UVC camera capable of 1080p30
MJPEG, and network access to both PLCs. All settings live in
[`modules/config.json`](modules/config.json); check `ip_address` / `siemens_ip` and the
`scheduler` geometry before running anything against real hardware.

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

# Simulated pick pipeline, no hardware
python3 main.py --scheduler --scenario test_throughput --duration 12.0 --simulate-executor

# Vision only — real camera, no robot, live web dashboard on http://localhost:8000
python3 main.py --scheduler --scenario test_vision_only --interface --duration 20

# Production run on real hardware
python3 main.py --scheduler --scenario production --interface
```

`--interface` serves an in-process web dashboard (annotated MJPEG plus live telemetry) and
suppresses the native OpenCV window. Without it, real-camera scenarios open a local overlay
window; set `vision.show_window = false` to run headless.

The full verification command list — compile check, fake-PLC dry run, calibration probes — is
in [`doc/basis-programming.md`](doc/basis-programming.md) §8.

### Scenarios

| Scenario | Vision | Robot | Conveyor speed |
|---|---|---|---|
| `production` | real camera | real, two-thread realtime loop | Siemens PLC |
| `test_vision_only` | real camera | idle | Siemens PLC |
| `test_throughput` | simulated | sim or real | synthetic |
| `test_accuracy` | simulated | sim or real | static |
| `test_acceptance` | simulated | sim or real | static |
| `evaluate` | simulated | sim or real | synthetic |

`production` requires live PLC belt-position feedback and cannot be dry-run with
`--simulate-executor`. Note that only `production` and `test_vision_only` exercise the
two-thread realtime loop; the other scenarios run a separate single-threaded harness
(`doc/open-issues.md` L7/L8).

---

## Documentation

Everything technical lives in [`doc/`](doc/) — this README intentionally holds no
configuration reference, roadmap or bug list.

| File | Contents |
|---|---|
| [`doc/context.md`](doc/context.md) | **Start here.** Project phase, document map, directory structure, rules for AI assistants |
| [`doc/basis-theory.md`](doc/basis-theory.md) | Coordinate transforms, delta kinematics, trajectory profiles, tracking and interception, orientation chain, adaptive belt-speed law |
| [`doc/basis-programming.md`](doc/basis-programming.md) | Thread/process architecture, PLC data contracts, trajectory templates, scenario matrix, config-key reference, verification commands |
| [`doc/open-issues.md`](doc/open-issues.md) | Single register of unresolved problems: pending calibration, config inconsistencies, architectural limits |
| [`doc/decision-log.md`](doc/decision-log.md) | History: superseded designs and why they were replaced |
| [`doc/dev-note.md`](doc/dev-note.md) | Developer's bench notes |

`doc/PLC_Program_description/` holds rung-by-rung breakdowns of the PLC programs, and
`doc/Manuals/` the hardware datasheets.
