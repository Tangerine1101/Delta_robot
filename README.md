# Delta Robot

This repository contains Python-side tooling for a delta robot that exchanges fixed PLC packages over Ethernet and now also includes an offline scheduler simulator.

## Current modes

- `--cli`: manual PLC command mode
- `--scheduler`: offline scheduler simulation and benchmark mode

## Fixed PLC package

The PLC struct must keep the same members and the same fixed array length.

Current fixed slot count:

- `interpolar_points = 6`

Outgoing PC package:

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

Notes:

- `argument_number` is the number of active points
- unused elements must still be sent as `0.0`
- `argument_e` is used for end-effector state along trajectory points

## Configuration

Main configuration lives in [modules/config.json](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/config.json).

It now contains:

- PLC connection defaults
- fixed `interpolar_points = 6`
- object-type mapping
- sorting position, for example `object_A`
- scheduler motion and benchmark settings

## CLI usage

Run the PLC CLI:

```bash
python3 main.py --cli
```

Optional example:

```bash
python3 main.py --cli --ip 192.168.250.1 --port 502 --interpolar-points 6
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

Run the throughput scenario:

```bash
python3 main.py --scheduler --scenario test_throughput
```

Run the accuracy scenario for a limited time:

```bash
python3 main.py --scheduler --scenario test_accuracy --duration 5
```

Scheduler characteristics:

- input: simulated object detections plus simulated conveyor speed
- future integration point: image processing module and EthernetCom speed source
- output: `PickPlan`
- each cycle includes:
  - outbound leg to conveyor pickup
  - inbound leg to sorting zone
- each leg uses a 6-point trajectory template

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
