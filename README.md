# Delta Robot

This repository contains the current Python-side control code for a delta robot that exchanges fixed PLC packages over Ethernet.

At the moment, only `--cli` mode is implemented. Other execution modes can be added later without changing the PLC package structure.

## Current scope

- Start a worker process that talks to the PLC through `pylogix`
- Open an interactive CLI for manual testing
- Send fixed-structure command packages to the PLC tag `pc_package`
- Read basic PLC status from the PLC tag `plc_package`

## Important rule: do not change the package structure

The PLC struct is already declared on the PLC side, so the Python package must keep the same members and the same fixed array length.

Current outgoing PC package:

```python
{
    "commandID": int,
    "argument_number": int,
    "argument_x": [float] * interpolar_points,
    "argument_y": [float] * interpolar_points,
    "argument_z": [float] * interpolar_points,
    "argument_e": [float] * interpolar_points,
    "argument_time": [float] * interpolar_points,
}
```

Important notes:

- `interpolar_points` is the fixed array length of the struct.
- `argument_number` is the number of valid points used by the current command.
- If a field is unused, it must still be sent and padded with `0.0`.
- `argument_e` is only meaningful for trajectory commands.
- For non-trajectory commands, `argument_e` is sent as all zeros.

## PLC tags

- Write from PC to PLC: `pc_package`
- Read from PLC to PC: `plc_package`

The current status probe reads:

- `plc_package.task_doing`
- `plc_package.task_state`

## Requirements

- Python 3.10 or newer is recommended
- `pylogix`
- A PLC program that already defines compatible `pc_package` and `plc_package` structs

Install dependency:

```bash
pip install pylogix
```

## Configuration

Default configuration is stored in [modules/config.json](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/config.json):

```json
{
    "ip_address": "192.168.250.1",
    "port": 502,
    "period_s": 0.1,
    "interpolar_points": 4
}
```

Meaning:

- `ip_address`: PLC IP address
- `port`: PLC port
- `period_s`: reserved for future use
- `interpolar_points`: fixed array size of the PLC package

Do not change `interpolar_points` unless the PLC struct is changed to exactly match it.

## How to run

Run the CLI:

```bash
python3 main.py --cli
```

Optional arguments:

```bash
python3 main.py --cli --ip 192.168.250.1 --port 502 --interpolar-points 4 --prompt "robot> "
```

Available arguments:

- `--cli`: start interactive CLI mode
- `--ip`: override PLC IP address
- `--port`: override PLC port
- `--interpolar-points`: fixed array length for the package
- `--prompt`: custom CLI prompt text

If `--cli` is not provided, the program exits with an error because no other mode is implemented yet.

## CLI commands

After the CLI starts, these commands are available:

- `stop`
- `go <theta1> <theta2> <theta3>`
- `goto <x> <y> <z>`
- `go_trajectory <demo|square|home>`
- `calib`
- `pick`
- `release`
- `status`
- `help`
- `quit`
- `exit`

### Command meanings

- `stop`
  Sends command ID `0` to stop motion.

- `go <theta1> <theta2> <theta3>`
  Sends a relative joint move using command ID `1`.

- `goto <x> <y> <z>`
  Sends an absolute Cartesian move using command ID `2`.

- `go_trajectory <demo|square|home>`
  Sends a predefined trajectory using command ID `3`.

- `calib`
  Sends command ID `4`.

- `pick`
  Sends command ID `5` to close or activate the end effector.

- `release`
  Sends command ID `6` to open or release the end effector.

- `status`
  Reads the current PLC status package.

### Examples

```text
robot> go 5 0 -5
robot> goto 0 0 -200
robot> go_trajectory home
robot> pick
robot> release
robot> status
```

## Trajectory presets

The current CLI contains these built-in presets in [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py):

- `demo`
- `square`
- `home`

Each trajectory point contains:

```python
{
    "x": float,
    "y": float,
    "z": float,
    "e": float,
    "time": float,
}
```

Meaning:

- `x`, `y`, `z`: Cartesian target
- `e`: end-effector state along the trajectory
- `time`: segment time

Convention for `e`:

- `0.0`: open / release
- `1.0`: pick / close

## Example package: `go_trajectory home`

With `interpolar_points = 4`, the preset `home` currently has one active point:

```python
{
    "commandID": 3,
    "argument_number": 1,
    "argument_x": [0.0, 0.0, 0.0, 0.0],
    "argument_y": [0.0, 0.0, 0.0, 0.0],
    "argument_z": [-200.0, 0.0, 0.0, 0.0],
    "argument_e": [0.0, 0.0, 0.0, 0.0],
    "argument_time": [0.6, 0.0, 0.0, 0.0],
}
```

Why it looks like this:

- Only the first slot is active because `argument_number = 1`
- The remaining slots must still exist because the PLC struct has fixed-length arrays
- `argument_e` is present even when not used by this preset

## Connection behavior

PLC communication is handled by `PLCGateway` in [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py).

Current behavior:

- `connect()` does not just set a flag
- it tries to read PLC status tags first
- `self.connected` becomes `True` only if that probe succeeds
- read/write failures force `self.connected = False`
- `disconnect()` closes the communication object and then marks the gateway as disconnected

This is meant to keep the Python-side connection state closer to the real communication state.

## File overview

- [main.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/main.py): entry point, argument parsing, worker process, CLI startup
- [modules/cli.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py): command parsing and package creation for CLI commands
- [modules/EthernetCom.py](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py): PLC communication, packet normalization, config loading
- [Algorithm.md](/home/tangerine/Share/Global%20Share/Documents/Delta_robot/Algorithm.md): high-level workflow notes

## Known limitations

- Only CLI mode is implemented
- Only a small PLC status subset is read
- Trajectories are currently hardcoded presets
- Image processing and scheduler mode mentioned in `Algorithm.md` are not implemented here yet
