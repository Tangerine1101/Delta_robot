# AI Context Summary: Delta Robot
> **Target Audience**: AI Coding Assistants, Subagents, and compact context updates during chat session resets.
> **Status**: Phase 3 image processing integrated. `VisionImageProcessing` (YOLO26-OBB + centroid tracker) runs in-process via background thread and opens a live overlay window (boxes + id/pos/angle + CAM/PROC FPS). Belt position now arrives pre-decoded from Siemens as a single `conveyor_position` field (cm); raw `encoderA`/`encoderB` removed. Scenarios `production`, `test_conveyor`, `test_vision_only` wire the real camera into the scheduler. Object types: `QFP` / `TQFP`.

---

## 1. Quick Technical Reference

### 1.1. Codebase Structure & Directory Guidelines

#### Directories to Read:
* **Root**:
  * [main.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/main.py): Primary orchestrator for CLI and scheduler modes.
  * [README.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/README.md): Quickstart and repository entry overview.
* **`modules/`**: Contains the active logic of the system:
  * [scheduler.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/scheduler.py): Core path planning, safety checks, simulated speed/perception, and executor management. Now operates in conveyor C-frame for pickup prediction.
  * [conveyor.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/conveyor.py): Conveyor frame F, camera frame M, `BeltPositionTracker` (PLC `conveyor_position` cm → mm + velocity), `BeltTracker` for on-belt object tracking.
  * [EthernetCom.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py): PLC communication gateway (PLCGateway) using `pylogix` for Omron.
  * [cli.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/cli.py): Command parser.
  * [image_processing.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/image_processing.py): `SimulatedImageProcessing` (fake, no deps) + `VisionImageProcessing` (real YOLO26-OBB pipeline in background thread). Emits `ObjectDetection` with C-frame `(u, v)` and `angle_deg`. Opens a live overlay window (boxes + id/pos/angle + CAM/PROC FPS) by default for camera scenarios; disable with `vision.show_window=false`.
  * [test_module.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/test_module.py): TCP fake PLC simulator. Fake-emits `conveyor_position` (cm) integrated from `speed_current`.
  * [config.json](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/config.json): Active configuration file. Includes `conveyor` section (`conveyor_position_scale_mm`, length_mm, camera_window_uv, workspace_window_uv), `vision.show_window`, and per-PCB `w`/`h` dimensions.
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
* Struct tag: `pc_package` (fixed array slot count = `7`).
* Value layout:
  ```python
  {
      "commandID": int,
      "argument_number": int,
      "argument_x": [float] * 7,
      "argument_y": [float] * 7,
      "argument_z": [float] * 7,
      "argument_e": [byte] * 7,     # 0 = gripper OFF, 1 = gripper ON
      "argument_time": [float] * 7,  # Segment duration in seconds — Omron firmware ignores this; still send all 7 elements
      "bit_doing": byte              # 1 = command ready (PC writes, PLC resets to 0)
  }
  ```
* Invariant: Even if a command uses $< 7$ points, the arrays must always be padded to `7` elements with `0.0`.
* **`argument_time`**: Omron NX1P2 firmware **ignores** this field. Robots run at fixed maximum speed. PC-side `nominal_xy_speed` / `nominal_z_speed` are scheduler-side timing approximations only.
* **`task_state`**: PLC behavior is inconsistent — values 0/1/2; `2` = done; no error value exists. `bit_doing` has replaced its role as the primary completion signal. All code referencing `task_state` is intentionally preserved.
* **`grab`/`place` CLI commands**: Do **not** actuate suction on the real PLC. Command IDs 5/6 (pick/release) are no-ops; `grab`/`place` use `goto_absolute` with `argument_e=0`. Known limitation, not fixed in Phase 1.

Siemens S7-1200 package structure (PC → PLC):
```python
{
    "CommandID": int,
    "rotate": float,
    "speed": float
}
```

Siemens S7-1200 status structure (PLC → PC, **20 bytes**):
```python
{
    "rotate_current": float,
    "speed_current": float,
    "task_doing": int,
    "task_state": int,
    "conveyor_position": float,   # belt position in cm (REAL @ DB2 offset 16); PC ×10 → mm
}
```
> Replaces the former raw `encoderA`/`encoderB` quadrature counts. The PLC now
> accumulates and reports belt position directly. DB2 must be 20 bytes with
> "Optimized block access" disabled (byte layout must match `SiemensReceivePacket`).

### 1.4. Scenario Reference

All scenarios are selected with `--scenario <name>`. `main.py` runs all six;
`run_test.py` (subprocess + matplotlib plot) runs every scenario except `production`.

| Scenario | Vision | Robot | Belt source | Camera window | Entry points |
|----------|--------|-------|-------------|---------------|--------------|
| `test_throughput` | simulated | sim/real | synthetic | no | main.py, run_test.py |
| `test_accuracy` | simulated | sim/real | none (static) | no | main.py, run_test.py |
| `evaluate` | simulated | sim/real | synthetic | no | main.py, run_test.py |
| `test_vision_only` | **real camera** | none (`NullExecutor`) | sim / `conveyor_position` | **yes** | main.py (`--simulate-executor`), run_test.py |
| `test_conveyor` | **real camera** | real | `conveyor_position` (Siemens) | **yes** | main.py, run_test.py |
| `production` | **real camera** | real | `conveyor_position` (Siemens) | **yes** | main.py only |

* `--simulate-executor` swaps the real PLC for `SimulatedExecutor`/`NullExecutor` and the belt
  feed for `SimulatedSpeedSource` (fixed `test_conveyor_belt_speed_mm_s`). Without it, the
  scheduler uses `RealRobotExecutor` + `ConveyorSpeedSource` (reads `conveyor_position`).
* Camera-window scenarios also run standalone: `python3 -m modules.image_processing --duration N`
  runs YOLO and shows the same overlay window (`--no-window` for headless).

---

## 2. Mathematical Equations & Timing

### 2.1. Coordinate Conventions
* Delta Robot Z-axis is negative (downward). Points closer to `0.0` are higher.
* Workspace is a rectangle in conveyor C-frame: `conveyor.workspace_window_uv = [u_min, u_max, v_min, v_max]`. Sorting bins are outside this window by definition.
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
2. **Vision Integration**: `VisionImageProcessing` runs YOLO26-OBB in-process (background thread). **Calibration pending**: `M_VISION_TO_CONVEYOR` in `conveyor.py` is a placeholder — calibrate once physical rig is assembled. Vision deps: `ultralytics>=8.3.0`, `opencv-python>=4.9.0` in `.venv`. `YOLO_OBB/` is the teammate's repo (external, gitignored).
3. **Object types**: renamed to match vision classes — `QFP` and `TQFP`. Config keys and destinations updated throughout.
4. **4-DOF rotation**: `plan.rotate_deg` (from `angle_deg` + `rotate_offset_deg` config) is sent to the Siemens S7-1200 via `rotate_absolute` command at pick dispatch time.
5. **Git clean state**: Ensure log files (`data.log`, `test_module.log`) and cache files (`__pycache__`) are ignored by git in local development. `YOLO_OBB/` is gitignored (nested repo).
6. **PLC fixed motor speed (Omron NX1P2)**: The PLC firmware drives the servo motors at a fixed maximum speed and **ignores the `argument_time` field** of each trajectory point. PC-side `nominal_xy_speed` / `nominal_z_speed` only affect scheduler-side timing (pick-prediction, log timestamps) — they do not throttle actual robot motion. To measure true mechanism speed, gate phase progression on `pos_EE` convergence (see `evaluate` scenario) rather than wall-clock from `argument_time`.

---

## 4. Verification Commands

Run these commands to verify that code changes did not break the existing modules:

```bash
# 1. Compile check all python files
python3 -m py_compile main.py modules/cli.py modules/EthernetCom.py modules/image_processing.py modules/scheduler.py modules/test_module.py modules/conveyor.py

# 2. Run scheduler simulation throughput scenario
python3 main.py --scheduler --scenario test_throughput --duration 12.0 --simulate-executor

# 3. Run scheduler simulation accuracy scenario
python3 main.py --scheduler --scenario test_accuracy --duration 5.0 --simulate-executor

# 4. Verify test module logic with a dry run
python3 -m modules.test_module --port 1502 --self-test --duration 2.0

# 5. Run evaluate scenario (continuous box <-> 3 accuracy_points; Ctrl-C to stop).
python3 main.py --scheduler --scenario evaluate --simulate-executor --duration 10.0

# 6. Vision smoke test + overlay window (requires physical camera + board crossing trigger line)
python3 -m modules.image_processing                        # runs until q / Ctrl-C (--duration N, --no-window)

# 7. Production dry-run (real vision, simulated robot)
python3 main.py --scheduler --scenario production --simulate-executor --duration 20

# 8. Vision-only (real camera, no robot) — shows predicted picks on run_test.py plot
python3 run_test.py --scenario test_vision_only --duration 20

# 9. Conveyor test (real camera + robot + Siemens conveyor_position feedback)
python3 run_test.py --scenario test_conveyor --duration 30
```
