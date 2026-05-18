# System Description — Delta Robot Pick-and-Place Project

> **Last updated:** 2026-05-18
> **Project type:** Graduation thesis
> **Topic:** Delta robot control for product sorting on a variable-speed conveyor, using future image processing and a scheduler

> **Disclaimer:** This document is a practical project overview aligned to the current repository state. Some future-system details are still provisional.

---

## 1. Project Overview

### 1.1. Problem Statement

The delta robot must pick products from a moving conveyor and place them into sorting areas. The main difficulty is synchronization:

- the product is moving
- the robot needs to predict where the product will be
- the conveyor speed may change over time
- the robot must still reach the right point at the right time

This creates two connected problems:

1. reliable **motion execution** between PC and PLC
2. reliable **pick scheduling and prediction** before motion is sent

### 1.2. Full System Vision

```
┌─────────────────────────────────────────────────────────────────────┐
│                        FULL SYSTEM VISION                           │
│                                                                     │
│ Camera / Vision ─► Scheduler / PickPlan ─► Motion / PLC ─► Robot    │
│                      ▲                    │                          │
│                      │                    ▼                          │
│               Conveyor speed       Sorting zones / bins             │
└─────────────────────────────────────────────────────────────────────┘
```

Target full system components:

1. **Image processing** — detect object position and type
2. **Scheduler** — predict pick timing/position and select destination
3. **Trajectory planning** — build 6-point robot legs for pick/place
4. **PLC communication** — send fixed packages to PLC and read status
5. **Robot execution** — PLC + EtherCAT + servo system
6. **Adaptive conveyor** — later provide speed input and possibly closed-loop adjustment

### 1.3. Current Repository State

The repository is no longer only a raw PLC-motion prototype. It now contains two implemented software tracks:

1. **CLI mode**
   - manual commands to send PLC packages
   - useful for direct motion testing

2. **Offline scheduler mode**
   - simulated object stream
   - simulated conveyor speed
   - `PickPlan` generation
   - benchmark scenarios for throughput and accuracy

What is still simulated:

- image processing input
- conveyor-speed input from `EthernetCom`
- executor feedback for scheduler scenarios

---

## 2. Current Architecture

### 2.1. Hardware

| # | Component | Model | Qty | Role |
|---|-----------|-------|-----|------|
| 1 | Machine Controller | Omron NX1P2-1140DT | 1 | Motion master, PLC communication endpoint |
| 2 | Servo Driver | Panasonic MADLN05BE | 3 | EtherCAT slave |
| 3 | Servo Motor | Panasonic MSMF012L1T2 | 3 | Actuate the 3 robot axes |
| 4 | Delta Robot | 3-DOF parallel mechanism | 1 | Pick-and-place structure |
| 5 | PC | Laptop / Desktop (Python) | 1 | CLI, scheduler, simulator, communication |

### 2.2. Software Modules in the Repository

| Component | Current status | Description |
|-----------|----------------|-------------|
| `main.py` | Implemented | Entry point for CLI and scheduler modes |
| `EthernetCom.py` | Implemented | PLC package normalization and read/write gateway |
| `cli.py` | Implemented | Manual command parser |
| `image_processing.py` | Implemented as simulator | Fake object source for scheduler testing |
| `scheduler.py` | Implemented | Pick-plan generation, trajectory template, scenario runner |

### 2.3. Current Operating Modes

| Mode | Command | Purpose |
|------|---------|---------|
| CLI | `python3 main.py --cli` | Manual PLC command testing |
| Scheduler benchmark | `python3 main.py --scheduler --scenario test_throughput` | Continuous simulated pick planning |
| Scheduler accuracy test | `python3 main.py --scheduler --scenario test_accuracy` | Repeated fixed-target planning with logging |

---

## 3. Data Contracts

### 3.1. PC → PLC package

The repository currently sends a fixed robot command struct with **6 slots**:

```python
{
    "commandID": int,
    "argument_number": int,
    "argument_x": [float] * 6,
    "argument_y": [float] * 6,
    "argument_z": [float] * 6,
    "argument_e": [float] * 6,
    "argument_time": [float] * 6,
}
```

Important meaning:

- `argument_number`: active points in the current command
- `argument_e`: end-effector state timeline along the trajectory
- array length must stay synchronized with the PLC struct

### 3.2. PLC → PC status

Current Python code reads:

- `plc_package.task_doing`
- `plc_package.task_state`

### 3.3. Scheduler input/output

Current scheduler input model:

- `object_id`
- `x`, `y`
- `object_type`
- `timestamp`
- conveyor speed sample

Current scheduler output model:

- `PickPlan`

`PickPlan` includes:

- predicted pickup time
- predicted pickup position
- sorting destination
- outbound 6-point trajectory
- inbound 6-point trajectory

---

## 4. Motion and Scheduling Logic

### 4.1. CLI path

CLI mode is the direct operator path:

1. user types a command
2. CLI converts it into a normalized PLC package
3. worker process sends that package to the PLC
4. main process reads and prints PLC status

### 4.2. Scheduler path

Scheduler mode is the planning/operator path:

1. simulated image-processing module emits fake detections
2. simulated speed source emits conveyor speed samples
3. scheduler predicts pickup timing and position
4. scheduler chooses the sorting destination from config
5. scheduler builds a `PickPlan`
6. the plan contains:
   - outbound leg to the conveyor
   - inbound leg to the sorting position

### 4.3. 6-point leg template

Each leg currently uses a 6-point template.

Outbound leg intent:

1. leave current position
2. rise to safe clearance
3. move through blended corner
4. approach intercept zone
5. arrive slightly early above the predicted object location
6. descend and enable suction

Inbound leg intent:

1. leave pickup point
2. rise to clearance
3. move through blended corner
4. approach sorting zone
5. arrive above sorting target
6. descend and release

---

## 5. Development Phases

### Phase 1 — Motion execution

Implemented:

- PLC package generation
- CLI manual commands
- PLC worker process
- connection-state handling in `EthernetCom`

### Phase 1.5 — Offline scheduler simulation

Implemented:

- fake object stream
- fake speed stream
- `PickPlan` generation
- `test_throughput` scenario
- `test_accuracy` scenario
- `data.log` trace output for accuracy runs

### Phase 2 — Real integration

Planned next steps:

- replace fake detections with real image-processing output
- replace fake speed with real speed from `EthernetCom`
- connect `PickPlan` execution to real PLC motion commands
- validate pickup timing on the real conveyor

---

## 6. Main Constraints

- PLC-side struct must remain synchronized with Python
- current fixed array length is **6**
- scheduler must be testable without real image processing
- object type routing must come from config
- pick logic must support one full cycle: pick leg + place leg

---

## 7. Open Questions

| # | Question | Current state |
|---|----------|---------------|
| 1 | Real PC ↔ PLC transport details | Still needs final confirmation |
| 2 | Real conveyor-speed source shape | Planned through `EthernetCom` |
| 3 | Real image-processing interface | Not integrated yet |
| 4 | Driver/motor power compatibility | Still needs verification |
| 5 | Whether adaptive conveyor will become closed-loop output, not only input | Still open |

---

## 8. Document Index

| File | Description |
|------|-------------|
| `system_configuration.md` | Hardware and software configuration, including config variables |
| `workflow.md` | Current repository workflow for CLI and scheduler |
| `adaptive_conveyor.md` | Conveyor prediction assumptions and future control notes |
| `python_scripts_beginner_guide.md` | Beginner-oriented explanation of the Python code |

---

*This document is the high-level project map. For exact runtime parameters, use `system_configuration.md` and `modules/config.json` as the primary references.*
