# 16_5 Update

This document summarizes the current codebase logic after the scheduler, 4-point trajectory, timing compensation, and fake PLC test module updates.

## 1. Current System Shape

The repository currently has four main runtime areas:

1. `main.py`
2. `modules/EthernetCom.py`
3. `modules/scheduler.py`
4. `modules/test_module.py`

`main.py` is the entrypoint for real operation modes. `EthernetCom.py` owns the PLC package contract and pylogix communication. `scheduler.py` owns simulated perception, speed input, pick planning, trajectory generation, and executor logic. `test_module.py` is an independent fake PLC server for testing package and timing logic without the real robot or real PLC.

## 2. Fixed PLC Package Contract

The PC-to-PLC package is fixed to 4 slots:

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

The important rule is that the array length must always stay aligned with the PLC struct. Even when a command only uses one point, all arrays are still sent with 4 elements and unused values are padded with `0.0`.

`argument_e` is only meaningful for trajectory control. In trajectory packages, `0` means the end effector is open/released, and `1` means pick/suction is active.

## 3. Command IDs

The command mapping is defined in `modules/EthernetCom.py`:

```python
COMMAND_ID = {
    "stop": 0,
    "goto_relative": 1,
    "goto_absolute": 2,
    "go_trajectory": 3,
    "calibrate": 4,
    "pick": 5,
    "release": 6,
}
```

The same IDs are reused by CLI, scheduler, real PLC execution, and the fake PLC test module.

## 4. Configuration

Runtime config is loaded from `modules/config.json`.

Current key values:

- `ip_address = "192.168.250.1"`
- `port = 502`
- `interpolar_points = 4`
- `object_types = {"object_A": "object_A"}`
- `object_A = [140.0, -120.0, -205.0]`

Scheduler timing and geometry values include:

- `home_position`
- `clearance_height`
- `pickup_height`
- `pre_pick_height`
- `place_height`
- `corner_blend_xy`
- `nominal_xy_speed`
- `nominal_z_speed`
- `robot_movement_delay_s`
- `ethernet_delay_s`

`robot_movement_delay_s` is currently a constant approximation for the short movement from `D_goto` to `A_pick`. `ethernet_delay_s` is currently fixed at `0.002` seconds.

## 5. CLI Logic

CLI mode is started with:

```bash
python3 main.py --cli
```

The CLI accepts human-readable commands, converts them into normalized PLC packages, and sends them through the worker process.

Supported commands:

- `stop`
- `go <theta1> <theta2> <theta3>`
- `goto <x> <y> <z>`
- `go_trajectory <demo|square|home>`
- `calib`
- `pick`
- `release`
- `status`

`goto` no longer has a `w` argument. End-effector state is controlled through `pick`, `release`, or `argument_e` inside trajectory packages.

## 6. Main Process And PLC Worker

`main.py` starts a separate worker process for PLC communication.

The main process sends messages to the worker through a multiprocessing queue:

- `send`
- `status`
- `shutdown`

The worker owns the `PLCGateway` instance. This avoids mixing user input, scheduler logic, and PLC communication inside one blocking loop.

In scheduler real-execution mode, `main.py` creates a `RealRobotExecutor`, injects worker-backed `dispatch()` and `request_status()` functions, then runs the scheduler scenario.

## 7. EthernetCom Logic

`modules/EthernetCom.py` owns:

- config loading
- command IDs
- package normalization
- pylogix PLC connection
- package writes to `pc_package`
- status reads from `plc_package`

`PLCGateway.connect()` now reflects real connection state by probing PLC status tags. `self.connected = True` is only set after the probe succeeds. `disconnect()` closes the pylogix connection and updates `self.connected`.

Important point: the current real PLC path uses `pylogix`. A simple TCP fake server cannot fully emulate EtherNet/IP/pylogix. That is why `test_module.py` is a standalone JSON-lines test harness, not a drop-in replacement for a real PLC in `EthernetCom.py`.

## 8. Scheduler Input

The scheduler currently uses simulated inputs:

- fake object detections from `SimulatedImageProcessing`
- fake conveyor speed from `SimulatedSpeedSource`

The intended future real inputs are:

- object id
- 2D object location
- object type
- timestamp
- current conveyor speed from Ethernet/PLC integration

The current fake object type is `object_A`, which maps to the sorting position stored in config.

## 9. PickPlan Output

The scheduler outputs one `PickPlan`.

A `PickPlan` contains:

- `plan_id`
- `object_id`
- `object_type`
- source 2D position
- assumed conveyor speed
- predicted pick time
- compensated pick dispatch time
- predicted pick position
- sorting position
- `trajectory_goto`
- `trajectory_pick`

`PickPlan.to_robot_packets(interpolar_points)` converts the plan into two real `go_trajectory` PLC packages:

1. `goto` package
2. `pick` package

## 10. Current Trajectory Logic

The trajectory is now 4 points and 2 phases.

The `goto` phase moves the robot from its current/cycle start position to `D_goto`, which is slightly above the predicted pickup point.

The `pick` phase moves from independent `D_goto` down to independent `A_pick`, enables suction, then follows the shared points from the drawing:

- `B_pick` is the same Cartesian point as `C_goto`
- `C_pick` is the same Cartesian point as `B_goto`
- `D_pick` is the final sorting point

`trajectory_goto` uses end-effector values:

```python
[0.0, 0.0, 0.0, 0.0]
```

`trajectory_pick` uses end-effector values:

```python
[1.0, 1.0, 1.0, 0.0]
```

## 11. Pick Timing Compensation

Because PLC-side trajectory time planning is not implemented yet, Python controls when the `pick` package is sent.

The current formula is:

```text
t_p(real) = t_p(theory) - robot_movement_delay_s - ethernet_delay_s
```

Meaning:

- `t_p(theory)` is the predicted moment when the object should be picked.
- `robot_movement_delay_s` approximates the physical delay from `D_goto` to `A_pick`.
- `ethernet_delay_s` approximates communication delay.
- `t_p(real)` is when Python sends the `pick` package.

The real executor sends `goto` first. Before sending `pick`, it waits until `pick_dispatch_time`.

## 12. Scheduler Scenarios

Scheduler mode is started with:

```bash
python3 main.py --scheduler --scenario test_throughput
```

Offline mode avoids sending packages to the real PLC:

```bash
python3 main.py --scheduler --scenario test_throughput --simulate-executor
```

Current scenarios:

- `test_throughput`: continuously emits fake moving objects.
- `test_accuracy`: repeatedly targets fixed points and writes trace samples to `data.log`.

## 13. RealRobotExecutor

`RealRobotExecutor` receives a `PickPlan` and turns it into two PLC packages.

Execution order:

1. Send `goto` trajectory.
2. Wait for the PLC phase duration/status window.
3. Wait until `pick_dispatch_time`.
4. Send `pick` trajectory.
5. Wait for the PLC phase duration/status window.
6. Mark the plan completed.

Completion detection is still conservative. It mainly uses package duration plus margin, with optional polling of `task_doing`.

## 14. test_module.py

`modules/test_module.py` is a small standalone fake PLC server for logic testing.

Run it with:

```bash
python3 -m modules.test_module
```

It reads the default port from `modules/config.json`, which is currently `502`.

For normal non-root development, use a high test port:

```bash
python3 -m modules.test_module --port 1502 --self-test --duration 1.0
```

The module:

- opens a TCP port
- accepts one JSON package per line
- normalizes the incoming package to 4 slots
- simulates robot motion with simple linear interpolation
- logs fake `plc_package` output with elapsed runtime timestamps in milliseconds
- writes JSON-lines logs to `test_module.log`

Example log shape:

```json
{
  "timestamp_ms": 203.451,
  "event": "motion_sample",
  "plc_package": {
    "pos_angular": [0.1, -0.2, -2.1],
    "pos_EE": [10.0, -20.0, -210.0],
    "task_doing": 3,
    "task_state": 1,
    "end_effector": 1.0
  }
}
```

This module is intentionally simple. It is not a full EtherNet/IP PLC emulator and does not replace real `pylogix` communication.

## 15. Current Limitations

The current codebase still has these known limits:

- Real image processing is not integrated yet.
- Real conveyor speed from another PLC is not integrated yet.
- The fake PLC module uses JSON-lines, not pylogix/EtherNet-IP.
- PLC task completion semantics need a confirmed PLC-side state contract.
- Some generated files such as `data.log` and `__pycache__` are currently tracked by git.

## 16. Current Verification Commands

Useful checks:

```bash
python3 -m py_compile main.py modules/cli.py modules/EthernetCom.py modules/image_processing.py modules/scheduler.py modules/test_module.py
python3 main.py --scheduler --scenario test_throughput --duration 1.0 --simulate-executor
python3 main.py --scheduler --scenario test_accuracy --duration 0.2 --simulate-executor
python3 -m modules.test_module --port 1502 --self-test --duration 1.0
```

The first three commands verify the main code paths. The last command verifies the standalone fake PLC module.
