# Delta Robot

This repository contains Python-side tooling for a delta robot that exchanges fixed PLC packages over Ethernet and now also includes an offline scheduler simulator.

## Current modes

- `--cli`: manual PLC command mode
- `--scheduler`: offline scheduler simulation and benchmark mode

## Fixed PLC package

The PLC struct must keep the same members and the same fixed array length.

Current fixed slot count:

- `interpolar_points = 4`

Outgoing PC package:

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

Notes:

- `argument_number` is the number of active points
- unused elements must still be sent as `0.0`
- `argument_e` is used for end-effector state along trajectory points
- `0` means release/open, `1` means pick/on
- `doing_bit` is a byte handshake bit: PC sends `1` when a new command is ready, then PLC resets it

## Configuration

Main configuration lives in [modules/config.json](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/config.json).

It now contains:

- PLC connection defaults
- fixed `interpolar_points = 4`
- scheduler timing delays: `robot_movement_delay_s` and `ethernet_delay_s`
- object-type mapping
- sorting position, for example `object_A`
- scheduler motion and benchmark settings

## Setup and Usage

Before running the project, check these values in [modules/config.json](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/config.json):

- `ip_address` and `port`: real PLC connection target
- `interpolar_points`: must stay synchronized with the PLC struct
- `object_types` and `object_A`: object routing and sorting position
- `scheduler.pickup_height`, `scheduler.pre_pick_height`, `scheduler.place_height`: main Z positions
- `scheduler.robot_movement_delay_s` and `scheduler.ethernet_delay_s`: pick timing compensation

Recommended usage order:

1. Run offline scheduler logic first:

```bash
python3 main.py --scheduler --scenario test_throughput --simulate-executor
```

2. Run the fake PLC test module when you want to test package flow without the real PLC:

```bash
python3 -m modules.test_module --port 1502 --self-test --duration 1.0
```

3. Run the real CLI or real scheduler only after the connection settings are correct:

```bash
python3 main.py --cli
python3 main.py --scheduler --scenario test_throughput
```

## CLI usage

Run the PLC CLI:

```bash
python3 main.py --cli
```

Optional example:

```bash
python3 main.py --cli --ip 192.168.250.1 --port 502 --interpolar-points 4
```

Supported commands:

- `stop`
- `go <theta1> <theta2> <theta3>`
- `goto <x> <y> <z>`
- `go_trajectory <demo|square|home>`
- `calib`
- `pick`
- `release`
- `status`

## Scheduler usage

Run the throughput scenario on the real PLC worker:

```bash
python3 main.py --scheduler --scenario test_throughput
```

Run the accuracy scenario for a limited time on the real PLC worker:

```bash
python3 main.py --scheduler --scenario test_accuracy --duration 5
```

Run the scheduler in offline mode without sending packages to the PLC:

```bash
python3 main.py --scheduler --scenario test_throughput --simulate-executor
```

Scheduler characteristics:

- input: simulated object detections plus simulated conveyor speed
- future integration point: image processing module and EthernetCom speed source
- output: `PickPlan`
- default executor: converts each `PickPlan` into 2 real `go_trajectory` PLC packages
- offline executor: available through `--simulate-executor`
- each cycle includes 2 trajectory phases:
  - `goto`: move to the pre-pick point above the predicted object
  - `pick`: descend, suction-pick, move to sorting zone, and release
- each phase uses a fixed 4-point trajectory template
- the `pick` package is dispatched at `predicted_pick_time - robot_movement_delay_s - ethernet_delay_s`

### Default scheduler scenarios

- `test_throughput`
  - continuous fake object stream
  - keeps planning until stopped or until `--duration` expires

- `test_accuracy`
  - repeatedly targets 3 fixed points
  - logs simulated position updates to `data.log`

## File overview

- [main.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/main.py): entrypoint for CLI and scheduler modes
- [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py): PLC communication and packet normalization
- [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py): manual command parser
- [modules/image_processing.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/image_processing.py): fake image-processing object source
- [modules/scheduler.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/scheduler.py): pick-plan logic, trajectory template, and scenario runner

## Issues & Roadmap (Updated: 23/5)

Here is the list of identified issues and the sequential plan to address them:

### 1. Hardcoded Conveyor Axis (X-Axis Speed Limitation)
* **Problem:** The conveyor speed prediction model assumes the conveyor runs strictly parallel to the $X$ axis (`predicted_x = detection.x + speed * dt`).
* **Impact:** If the conveyor runs along the $Y$ axis, or at an angle relative to the robot's base frame, the prediction will be incorrect.
* **Resolution Plan (Phase 1):**
  1. Add a 2D velocity vector configuration `conveyor_velocity_vector: [v_x, v_y]` in `config.json` instead of a scalar speed.
  2. Refactor `modules/scheduler.py` (`_predict_pick_position`) to calculate predictions in 2D using the velocity vector:
     $$\mathbf{P}_{\text{pick\_xy}} = \mathbf{P}_{\text{detect\_xy}} + \mathbf{v}_{\text{conveyor}} \times \Delta t$$
  3. Update `modules/image_processing.py` to spawn objects and move them using the 2D velocity.

### 2. Overspecified Parameters Without Clear Documentation
* **Problem:** `config.json` contains a large number of interrelated height and timing parameters without sufficient documentation.
* **Impact:** High configuration complexity makes manual calibration error-prone, potentially leading to mechanical crashes.
* **Resolution Plan (Phase 2):**
  1. Document the exact physical meaning and safe ranges of each setting in a dedicated configuration guide (`doc/configuration_guide.md`).
  2. Add automatic validation logic in `modules/scheduler.py` during initialization to enforce logical safety invariants (e.g., verifying `clearance_height` > `pre_pick_height` > `pickup_height` in robot coordinates) and raise clear, helpful errors before running.

### 3. Incorrect Plane for Smooth Motion Blending
* **Problem:** The current trajectory corner blending (`corner_blend_xy`) operates within the horizontal $XY$ plane at `clearance_height`.
* **Impact:** In a standard gate-shaped pick-and-place cycle, the corner smoothing should occur in the vertical plane perpendicular to the $XY$ plane (blending the vertical $Z$ liftoff/descent with the horizontal $XY$ travel to create a smooth arch). The current implementation instead rounds the path horizontally while maintaining a constant $Z$, which does not reduce vertical acceleration peaks or create the desired 3D arch.
* **Resolution Plan (Phase 3):**
  1. Re-engineer the trajectory template in `_build_goto_geometry` and `_build_pick_geometry` to blend the $Z$ and $XY$ components.
  2. Map the 3D path coordinates to form a smooth vertical arch (blended parabola or splined transition) at the transition points.
  3. Update the segment timing calculation (`_build_goto_timing`) to reflect the new 3D path length.

### 4. Lack of Calibration and Profiling Tools
* **Problem:** There is currently no tool to measure key physical parameters of the cell automatically.
* **Impact:** Delays such as `robot_movement_delay_s` and `ethernet_delay_s` must be estimated manually.
* **Resolution Plan (Phase 4):**
  1. Develop a standalone calibration utility `modules/calibration.py`.
  2. Implement network latency profiling (calculating average round-trip times by reading/writing mock or real tags repeatedly).
  3. Implement mechanical profiling (sending move commands, polling the PLC status until completion, and logging execution duration).
  4. Implement an automatic tuning script to write measured timing delays back to `config.json`.
