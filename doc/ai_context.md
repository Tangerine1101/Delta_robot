# AI Context Summary: Delta Robot
> **Target Audience**: AI Coding Assistants, Subagents, and compact context updates during chat session resets.
> **Status**: Core hardware and CLI modes functional, scheduler simulation handles 2D conveyor velocity vectors and safety checks, 4-DOF rotation command interfaces defined.

---

## 1. Quick Technical Reference

### 1.1. Codebase Structure & Directory Guidelines

#### Directories to Read:
* **Root**:
  * [main.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/main.py): Primary orchestrator for CLI and scheduler modes.
  * [README.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/README.md): Quickstart and repository entry overview.
* **`modules/`**: Contains the active logic of the system:
  * [scheduler.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/scheduler.py): Core path planning, safety checks, simulated speed/perception, and executor management.
  * [EthernetCom.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py): PLC communication gateway (PLCGateway) using `pylogix` for Omron.
  * [cli.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py): Command parser.
  * [image_processing.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/image_processing.py): Mock perception queue.
  * [test_module.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/test_module.py): TCP fake PLC simulator.
  * [config.json](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/config.json): Active configuration file.
* **`doc/`**:
  * [system_reference.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/system_reference.md): Full technical, mathematical, and architectural reference manual.
  * [ai_context.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/ai_context.md): This file.

#### Directories to AVOID / Ignore:
* **`.trash/`**: Contains consolidated legacy documents (historical backups only). **DO NOT read, edit, or recover files from here** unless explicitly requested.
* **`doc/Manuals/`**: Contains large PDF files of Omron/Panasonic manuals. **DO NOT read** unless looking for a very specific register or hardware specification.
* **`doc/human_ideas.md`**: Dedicated space for human brainstorming/research. **AI should NOT edit this file**. Read it only if you need context on what research ideas are planned.
* **`.git/`, `.venv/`, `.agents/`, `__pycache__/`, `modules/__pycache__/`**: System metadata, virtual environment, and python cache files. **IGNORE**.

### 1.2. Command Mapping (COMMAND_ID)
Used by the CLI, scheduler, real PLC, and test module:
```python
COMMAND_ID = {
    "stop": 0,
    "goto_relative": 1,
    "goto_absolute": 2,
    "go_trajectory": 3,
    "calibrate": 4,
    "pick": 5,
    "release": 6,
    "rotate_absolute": 7,  # 4-DOF suction cup rotation (Siemens PLC)
    "change_speed": 8,     # Conveyor speed setting (Siemens PLC)
    "plan_siemen": 9       # Planning command specifically for Siemens
}
```

### 1.3. PLC Data Contract
PC-to-PLC packet sent to the Omron NX CPU:
* Struct tag: `pc_package` (fixed array slot count = `4`).
* Value layout:
  ```python
  {
      "commandID": int,
      "argument_number": int,
      "argument_x": [float] * 4,
      "argument_y": [float] * 4,
      "argument_z": [float] * 4,
      "argument_e": [byte] * 4,     # 0 = gripper OFF, 1 = gripper ON
      "argument_time": [float] * 4,  # Segment duration in seconds
      "doing_bit": byte              # 1 = command ready (PC writes, PLC resets)
  }
  ```
* Invariant: Even if a command uses $< 4$ points, the arrays must always be padded to `4` elements with `0.0`.

Siemens S7-1200 package structure:
```python
{
    "CommandID": int,
    "rotate": float,
    "speed": float
}
```

---

## 2. Mathematical Equations & Timing

### 2.1. Coordinate Conventions
* Delta Robot Z-axis is negative (downward). Points closer to `0.0` are higher.
* Bounding Box checking is enforced via `pickup_window_x` and `pickup_window_y`.
* Safety rule: `clearance_height` > `pre_pick_height` > `pickup_height` must hold.

### 2.2. Interception and Dispatch
* Position prediction formula:
  $$\mathbf{P}_{\text{pick\_xy}} = \mathbf{P}_{\text{detect\_xy}} + \mathbf{v}_{\text{conveyor}} \times \Delta t$$
* Real-time pick descent command dispatch formula:
  $$t_{\text{dispatch\_real}} = t_{\text{pick\_theory}} - t_{\text{robot\_movement\_delay}} - t_{\text{ethernet\_delay}}$$
  *(Defaults: $t_{\text{robot\_movement\_delay}} = 0.05$ s, $t_{\text{ethernet\_delay}} = 0.002$ s)*

---

## 3. Current Limitations & Key Development Constraints
1. **Conveyor Speed Vector**: Speed has been updated to a 2D velocity vector `[vx, vy]`. Simulated components support this, but physical S7-1200 integration is pending.
2. **Vision Integration**: Active perception is mocked using `SimulatedImageProcessing`. Real camera frames are not integrated yet.
3. **4-DOF Physical Actuation**: Command IDs $7, 8, 9$ are defined for S7-1200 rotation and conveyor adjustment, but actual driver communication code needs integration.
4. **Git clean state**: Ensure log files (`data.log`, `test_module.log`) and cache files (`__pycache__`) are ignored by git in local development.

---

## 4. Verification Commands

Run these commands to verify that code changes did not break the existing modules:

```bash
# 1. Compile check all python files
python3 -m py_compile main.py modules/cli.py modules/EthernetCom.py modules/image_processing.py modules/scheduler.py modules/test_module.py

# 2. Run scheduler simulation throughput scenario
python3 main.py --scheduler --scenario test_throughput --duration 1.0 --simulate-executor

# 3. Run scheduler simulation accuracy scenario
python3 main.py --scheduler --scenario test_accuracy --duration 0.2 --simulate-executor

# 4. Verify test module logic with a dry run
python3 -m modules.test_module --port 1502 --self-test --duration 1.0
```
