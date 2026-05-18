# Workflow — Current Repository Operation

> **Last updated:** 2026-05-18
> **Phase:** Phase 1 motion execution + offline scheduler simulation
> **Status:** Implemented in Python repository
> **References:** `system_configuration.md`, `modules/config.json`

---

## 1. Objectives of the Current Workflow

The repository currently serves two purposes:

1. **manual robot command testing** through CLI
2. **offline scheduler testing** without real image processing or real conveyor-speed input

So the workflow is now split into two execution paths:

- **CLI path** for real PLC command sending
- **Scheduler path** for simulated planning and benchmark runs

---

## 2. Main Runtime Modes

| Mode | Entry command | Purpose |
|------|---------------|---------|
| CLI | `python3 main.py --cli` | Send manual commands to the PLC |
| Scheduler throughput | `python3 main.py --scheduler --scenario test_throughput` | Continuous simulated planning |
| Scheduler accuracy | `python3 main.py --scheduler --scenario test_accuracy` | Repeated fixed-target planning with logging |

---

## 3. CLI Workflow

### 3.1. Process layout

```
User terminal
    │
    ▼
main.py
    │
    ├── main process: CLI interaction
    └── worker process: PLC communication through EthernetCom
```

### 3.2. CLI data flow

```
User command
    │
    ▼
modules/cli.py
    │  parse text command
    ▼
RobotPacket / normalized package
    │
    ▼
worker process in main.py
    │
    ▼
modules/EthernetCom.py
    │  write fields into pc_package
    ▼
PLC
    │
    ▼
status read from plc_package.task_doing / task_state
```

### 3.3. Supported CLI commands

- `stop`
- `go <theta1> <theta2> <theta3>`
- `goto <x> <y> <z>`
- `go_trajectory <demo|square|home>`
- `calib`
- `pick`
- `release`
- `status`

### 3.4. Current PLC package shape

The Python code always sends a fixed struct of length 6:

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

Rules:

- `argument_number` tells how many points are active
- all arrays must still have length 6 even if only 1 point is used
- `argument_e` defines end-effector state along the trajectory

---

## 4. Scheduler Workflow

### 4.1. Purpose

Scheduler mode exists to test planning logic before real image processing and real conveyor-speed integration are available.

Current scheduler output:

- one `PickPlan` at a time

Current scheduler inputs:

- simulated object detections
- simulated conveyor speed samples
- object-type to sorting-position mapping from config

### 4.2. Scheduler data flow

```
SimulatedImageProcessing
    │
    ├── ObjectDetection stream
    ▼
PickScheduler
    ▲
    └── SimulatedSpeedSource
        speed samples

PickScheduler
    │
    ▼
PickPlan
    │
    ▼
SimulatedExecutor
    │
    └── optional trace logging to data.log
```

### 4.3. Scheduler loop

At each polling cycle:

1. collect new detections
2. collect the latest speed sample
3. prune stale detections
4. reject detections outside pickup workspace
5. resolve sorting destination from object type
6. predict pickup position and pickup time
7. build outbound and inbound trajectories
8. emit one `PickPlan`
9. execute it in the simulator
10. update metrics

### 4.4. Current scheduler scenarios

#### `test_throughput`

- emits a continuous fake object stream
- varies object appearance over lanes
- keeps planning until the run ends or the user interrupts it
- reports queue and planning metrics

#### `test_accuracy`

- emits a repeated sequence of 3 fixed points
- builds repeated `PickPlan`s
- logs simulated trajectory samples to `data.log`
- useful for comparing expected motion timing and trace output

---

## 5. PickPlan and Motion Template

### 5.1. PickPlan role

`PickPlan` is the scheduler's planning result. It is meant to sit between perception/speed input and future real robot execution.

It currently contains:

- object id
- object type
- source position
- predicted pickup time
- predicted pickup position
- sorting destination
- outbound trajectory
- inbound trajectory

### 5.2. One pick cycle

One full cycle includes:

1. **outbound leg** to the conveyor
2. **inbound leg** to the sorting position

### 5.3. 6-point trajectory template

Both legs currently use 6 points because the PLC package now has 6 slots.

#### Outbound leg intent

1. leave the current position
2. move to clearance
3. follow a blended horizontal corner
4. approach the intercept path
5. arrive slightly early above the predicted pickup point
6. descend and switch suction on

#### Inbound leg intent

1. leave the pickup point
2. move to clearance
3. follow a blended horizontal corner
4. approach the sorting path
5. arrive above the sorting point
6. descend and release

### 5.4. `argument_e` usage

- outbound leg typically ends with `argument_e = 1.0`
- inbound leg typically ends with `argument_e = 0.0`

So the gripper state is embedded into the trajectory timeline, not only sent as a separate pick/release command.

---

## 6. Current File Responsibilities

| File | Responsibility |
|------|----------------|
| `main.py` | Entry point, mode selection, CLI worker process, scheduler scenario launch |
| `modules/EthernetCom.py` | PLC gateway, package normalization, connection state handling |
| `modules/cli.py` | Manual command parsing and package building |
| `modules/image_processing.py` | Fake object-detection source |
| `modules/scheduler.py` | `PickPlan` generation, trajectory template, scenarios, metrics |
| `modules/config.json` | PLC and scheduler runtime configuration |

---

## 7. Current Limitations

- scheduler still uses simulated detections
- scheduler still uses simulated speed
- scheduler does not yet send real `PickPlan` output to the PLC execution path
- PLC status reading currently only checks `task_doing` and `task_state`
- real conveyor feedback and real object tracking are not integrated yet

---

## 8. Expected Near-Term Evolution

Next logical integration steps:

1. replace simulated detections with real image-processing output
2. read real conveyor speed through `EthernetCom`
3. connect `PickPlan` execution to real PLC motion command generation
4. validate the 6-point template on the actual robot

---

*This workflow document now describes the behavior that actually exists in the current Python repository, instead of the older draft binary-packet design.*
