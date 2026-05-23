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

The Python code always sends a fixed struct of length 4:

```python
{
    "commandID": int,
    "argument_number": int,
    "argument_x": [float] * 4,
    "argument_y": [float] * 4,
    "argument_z": [float] * 4,
    "argument_e": [byte] * 4,
    "argument_time": [float] * 4,
    "doing_bit": byte,
}
```

Rules:

- `argument_number` tells how many points are active
- all arrays must still have length 4 even if only 1 point is used
- `argument_e` defines end-effector state along the trajectory
- `0` means release/open, `1` means pick/on
- `doing_bit` is set to `1` by PC when a new command is sent, then reset by PLC

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
7. build `goto` and `pick` trajectory phases
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
- pick dispatch time after delay compensation
- sorting destination
- `goto` trajectory
- `pick` trajectory

### 5.2. One pick cycle

One full cycle includes:

1. **goto phase** to the pre-pick point above the conveyor object
2. **pick phase** to descend, suction-pick, move to sorting zone, and release

### 5.3. 4-point trajectory template

Both phases currently use 4 points because the PLC package has 4 trajectory slots.

#### Goto phase intent

1. leave the current position
2. move through the first blended corner at clearance
3. approach the conveyor side of the smoothed-square path
4. stop at `D_goto`, slightly above `A_pick`

#### Pick phase intent

1. move from `D_goto` to `A_pick` and enable suction
2. move through `B_pick`, which is the same Cartesian point as `C_goto`
3. move through `C_pick`, which is the same Cartesian point as `B_goto`
4. move to `D_pick` at the sorting point and release

### 5.4. `argument_e` usage

- `goto` phase keeps `argument_e = 0`
- `pick` phase starts with `argument_e = 1` and ends with `argument_e = 0`

So the gripper state is embedded into the trajectory timeline, not only sent as a separate pick/release command.

### 5.5. Pick dispatch timing

Because PLC-side trajectory time planning is not available yet, Python sends the `pick` package at the corrected pickup dispatch time:

```text
t_p(real) = t_p(theory) - robot_movement_delay_s - ethernet_delay_s
```

Current defaults:

- `robot_movement_delay_s = 0.05`
- `ethernet_delay_s = 0.002`

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
4. validate the 4-point, 2-phase template on the actual robot

---

*This workflow document now describes the behavior that actually exists in the current Python repository, instead of the older draft binary-packet design.*
