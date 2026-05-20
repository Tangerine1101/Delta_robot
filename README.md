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
