# Delta Robot Pick-and-Place Project

This repository contains Python-side control tooling for a Delta Robot sorting system communicating with PLCs (Omron NX1P2 and Siemens S7-1200) over Ethernet. It supports an interactive CLI mode for direct hardware commands and an offline scheduler simulation/benchmark tool.

---

## 1. Quickstart & Usage

### 1.1. Setup & Environment
Ensure you have the required packages installed. Pylogix is used for communicating with the Omron PLC.
```bash
pip install pylogix
```
Settings are loaded from `modules/config.json`. Check `ip_address`, `port`, and `scheduler` geometry values before executing commands on real hardware.

### 1.2. Run the Offline Scheduler Simulation
Simulates the pick scheduler, simulated object detections, and conveyor speed streams without hitting physical hardware:
```bash
# Run throughput scenario
python3 main.py --scheduler --scenario test_throughput --duration 10.0 --simulate-executor

# Run accuracy tracking scenario
python3 main.py --scheduler --scenario test_accuracy --duration 5.0 --simulate-executor
```

### 1.3. Run the Fake PLC TCP Server
Useful to test Python communication interfaces and telemetry log output without real controllers:
```bash
python3 -m modules.test_module --port 1502 --self-test --duration 1.0
```

### 1.4. Run the Real CLI or Auto-Scheduler
Execute these commands once connected to real PLCs:
```bash
# Start interactive CLI mode
python3 main.py --cli

# Run auto-scheduler with Omron RealRobotExecutor
python3 main.py --scheduler --scenario test_throughput
```

---

## 2. Basic Logic & Architecture

```
                  ┌──────────────────────┐
                  │      main.py         │
                  └──────────┬───────────┘
            ┌────────────────┴────────────────┐
            ▼                                 ▼
   ┌─────────────────┐               ┌─────────────────┐
   │ CLI Interactive │               │  Scheduler Loop │
   │   (cli.py)      │               │ (scheduler.py)  │
   └────────┬────────┘               └────────┬────────┘
            ▼                                 ▼
   ┌──────────────────────────────────────────────────┐
   │             EthernetCom.py (Gateway)             │
   └────────────────────────┬─────────────────────────┘
                            ▼
   ┌──────────────────────────────────────────────────┐
   │                PLC Hardware Layer                │
   └──────────────────────────────────────────────────┘
```

* **Threading Model**: 
  - Main Process: CLI Parser / Auto-Scheduler planning loop.
  - Worker Process (`multiprocessing` queue): PLCGateway communication to eliminate network latency blocking.
* **PLC Package Contract**: Fixed 4-slot coordinate arrays sent to the `pc_package` tag on the Omron PLC. Unused elements are zero-padded.
* **Interception Math**: Predicts conveyor interception using the object's initial position, dynamic 2D speed vector `[vx, vy]`, and a fixed-point iteration search.
* **4-Point/2-Phase Trajectory**: Moves in a `goto` phase (clearance travel to pre-pick position) followed by a `pick` phase (descent, grab, transfer to bin, release).
* **Timing Compensation**: Command is dispatched ahead of interception to account for mechanics and communication:
  $$t_{\text{dispatch}} = t_{\text{pick}} - t_{\text{robot\_movement\_delay}} - t_{\text{ethernet\_delay}}$$

---

## 3. Documentation Index

Detailed documentation files are available in the `doc/` directory:
* [system_reference.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/system_reference.md): Detailed specifications, coordinate constraints, trajectory math formulas, and code logic descriptions.
* [human_ideas.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/human_ideas.md): Human research notes, academic thesis topics, database schemas, and future ideas (AI should avoid editing this file).
* [ai_context.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/ai_context.md): Compact summary of codebase facts, command maps, and verification scripts for quick AI context updates.

---

## 4. Updates & Roadmap

### Recent Updates (23/5)
* **4 DOF and Siemens PLC Integration**: Added support for 4th degree of freedom (end-effector suction rotation via stepper) and conveyor speed adjustments handled by a secondary Siemens S7-1200 PLC. Defined new command IDs: `rotate_absolute` (7), `change_speed` (8), and `plan_siemen` (9).
* **2D Speed Vectors**: Updated conveyor speed calculations from a scalar speed to a 2D velocity vector `[vx, vy]` in `modules/scheduler.py` and `modules/config.json`.
* **Config Safety Constraints**: Added automated safety verification in `modules/scheduler.py` to assert:
  $$\text{clearance\_height} > \text{pre\_pick\_height} > \text{pickup\_height}$$

### Future Roadmap
1. **Vertical Motion Blending**: Smooth liftoff/descent vertical transitions (parabolic trajectory profiles instead of sharp gate corners).
2. **Calibration Utility**: Develop `modules/calibration.py` to auto-profile Ethernet round-trip latency and mechanical movement delays.
3. **Vision Integration**: Connect simulated perception queues to real camera segmentation streams.
