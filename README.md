# Delta Robot Pick-and-Place Project

This repository contains Python-side control tooling for a Delta Robot sorting system communicating with PLCs (Omron NX1P2 and Siemens S7-1200) over Ethernet. It supports an interactive CLI mode for direct hardware commands and an offline scheduler simulation/benchmark tool.

---

## 1. Quickstart & Usage

### 1.1. Setup & Environment
Ensure you have the required packages installed. Pylogix is used for communicating with the Omron PLC.
```bash
pip install pylogix
```
Settings are loaded from `modules/config.json`. Check `ip_address`, `port`, and `scheduler` geometry values before executing commands on real hardware.

### 1.2. Run the Offline Scheduler Simulation
Simulates the pick scheduler, simulated object detections, and conveyor speed streams without hitting physical hardware:
```bash
# Run throughput scenario
python3 main.py --scheduler --scenario test_throughput --duration 10.0 --simulate-executor

# Run accuracy tracking scenario
python3 main.py --scheduler --scenario test_accuracy --duration 5.0 --simulate-executor
```

### 1.3. Run the Fake PLC TCP Server
Useful to test Python communication interfaces and telemetry log output without real controllers:
```bash
python3 -m modules.test_module --port 1502 --self-test --duration 1.0
```

### 1.4. Run the Real CLI or Auto-Scheduler
Execute these commands once connected to real PLCs:
```bash
# Start interactive CLI mode
python3 main.py --cli

# Run auto-scheduler with Omron RealRobotExecutor
python3 main.py --scheduler --scenario test_throughput
```

---

## 2. Basic Logic & Architecture

```
                  ┌──────────────────────┐
                  │      main.py         │
                  └──────────┬───────────┘
            ┌────────────────┴────────────────┐
            ▼                                 ▼
   ┌─────────────────┐               ┌─────────────────┐
   │ CLI Interactive │               │  Scheduler Loop │
   │   (cli.py)      │               │ (scheduler.py)  │
   └────────┬────────┘               └────────┬────────┘
            ▼                                 ▼
   ┌──────────────────────────────────────────────────┐
   │             EthernetCom.py (Gateway)             │
   └────────────────────────┬─────────────────────────┘
                            ▼
   ┌──────────────────────────────────────────────────┐
   │                PLC Hardware Layer                │
   └──────────────────────────────────────────────────┘
```

* **Threading Model**: 
  - Main Process: CLI Parser / Auto-Scheduler planning loop.
  - Worker Process (`multiprocessing` queue): PLCGateway communication to eliminate network latency blocking.
* **PLC Package Contract**: Fixed 4-slot coordinate arrays sent to the `pc_package` tag on the Omron PLC. Unused elements are zero-padded.
* **Interception Math**: Predicts conveyor interception using the object's initial position, dynamic 2D speed vector `[vx, vy]`, and a fixed-point iteration search. The default simulated conveyor moves along positive Y while X stays fixed per lane.
* **Conveyor Speed Synchronization**: The Omron PLC has no awareness of actual conveyor speed. The PC is solely responsible for reading encoder speed from the Siemens S7-1200, planning the interception trajectory, computing the optimal pick timing, and sending pre-calculated static coordinates to the Omron PLC. The robot simply executes the received coordinates.
* **4-Point/2-Phase Trajectory**: Moves in a `goto` phase followed by a `pick` phase. `B_goto -> C_goto` and `B_pick -> C_pick` are mandatory 3D slope segments, not flat-then-vertical moves.
* **Timing Compensation**: Command is dispatched ahead of interception to account for mechanics and communication:
  $$t_{\text{dispatch}} = t_{\text{pick}} - t_{\text{robot\_movement\_delay}} - t_{\text{ethernet\_delay}}$$

---

## 3. Configuration Variables

All system settings are stored in [config.json](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/config.json).

### 3.1. Connection & PLC Settings
* `ip_address` (string): IP address of the Omron NX1P2 PLC (default: `192.168.250.1`).
* `port` (int): TCP port for Omron PLC connection (default: `44818` for EtherNet/IP).
* `siemens_ip` (string): IP address of the Siemens S7-1200 PLC (default: `192.168.250.2`).
* `siemens_port` (int): TCP port for Siemens PLC connection (default: `1502`).
* `period_s` (float): Status polling period or data update cycle (seconds).
* `interpolar_points` (int): Maximum number of trajectory points per PC packet (default: `4`).
* `object_types` (object): Map from object type identifier (e.g. `object_A`) to its sorting bin name.
* `object_A` (array): 3D coordinates `[x, y, z]` (mm) of the sorting bin for `object_A`.

### 3.2. Scheduler & Robot Geometry (`scheduler` block)
* `home_position` (array): 3D coordinates `[x, y, z]` of the robot's default rest (Home) position.
* `clearance_height` (float): Safe Z height (negative) for horizontal travel between bins and conveyor (e.g. `-290.0`).
* `slope_transition_height` (float): Z height at which the 3D slope segment begins to smooth the approach (e.g. `-295.0`).
* `pickup_height` (float): Z height at which the gripper picks the object off the conveyor (e.g. `-310.0`).
* `pre_pick_height` (float): Z height above the object just before the descent to pick (e.g. `-300.0`).
* `place_height` (float): Z height at which the gripper releases the object into the bin (e.g. `-290.0`).
* `corner_blend_xy` (float): XY corner blend radius at trajectory waypoints for smoother motion.
* `intercept_lead_time_s` (float): Minimum initial time estimate used to seed the interception convergence loop (seconds).
* `release_descent_time_s` (float): Dwell time at the release point while the suction cup deactivates (seconds).
* `nominal_xy_speed` (float): Nominal horizontal XY travel speed of the robot (mm/s).
* `nominal_z_speed` (float): Nominal vertical Z travel speed of the robot (mm/s).
* `stale_timeout_s` (float): Maximum time to track an object before dropping it from the queue (seconds).
* `speed_timeout_s` (float): Expiry time for conveyor speed data if no new sample is received (seconds).
* `poll_interval_s` (float): Scheduler loop repeat period (seconds).
* `default_speed` (array): Default conveyor velocity vector `[vx, vy]` (mm/s) used in simulation or when PLC is disconnected.
* `robot_movement_delay_s` (float): Mechanical response and acceleration lag of the physical robot (seconds).
* `ethernet_delay_s` (float): One-way Ethernet communication latency (seconds).
* `pickup_window_x` (array): X-axis bounding limit `[xmin, xmax]` of the valid pickup zone.
* `pickup_window_y` (array): Y-axis bounding limit `[ymin, ymax]` of the valid pickup zone.
* `throughput_object_types` (array): Object types spawned in the throughput simulation scenario.
* `throughput_lanes` (array): X coordinates of the conveyor lanes where simulated objects are spawned.
* `throughput_spawn_x` & `throughput_spawn_y` (float): Upstream spawn origin of simulated objects on the conveyor.
* `throughput_emit_interval_s` (float): Time interval between successive object spawns in the Throughput scenario.
* `accuracy_emit_interval_s` (float): Time interval between object spawns in the Accuracy scenario.
* `execution_margin_s` (float): Additional safety buffer added before a trajectory command expires (seconds).
* `accuracy_points` (array): List of static target coordinates used for tracking error profiling.
* `log_path` (string): File path for trajectory tracking data logs.

---

## 4. Documentation Index

Detailed documentation files are available in the `doc/` directory:
* [system_reference.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/system_reference.md): Detailed specifications, coordinate constraints, trajectory math formulas, and code logic descriptions.
* [human_ideas.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/human_ideas.md): Human research notes, academic thesis topics, database schemas, and future ideas (AI should avoid editing this file).
* [ai_context.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/ai_context.md): Compact summary of codebase facts, command maps, and verification scripts for quick AI context updates.

---

## 5. Updates & Roadmap

### Recent Updates (23/5)
* **4 DOF and Siemens PLC Integration**: Added support for 4th degree of freedom (end-effector suction rotation via stepper) and conveyor speed adjustments handled by a secondary Siemens S7-1200 PLC. Defined new command IDs: `rotate_absolute` (7), `change_speed` (8), and `plan_siemen` (9).
* **2D Speed Vectors**: Updated conveyor speed calculations from a scalar speed to a 2D velocity vector `[vx, vy]` in `modules/scheduler.py` and `modules/config.json`.
* **Config Safety Constraints**: Added automated safety verification in `modules/scheduler.py` to assert:
  $$\text{clearance\_height} > \text{slope\_transition\_height} > \text{pre\_pick\_height} > \text{pickup\_height}$$

### Future Roadmap
1. **Endianness Fix & 4th-Axis Rotation (Upcoming)**:
   - Change Siemens communication structs from `ctypes.Structure` to `ctypes.BigEndianStructure` in [modules/EthernetCom.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/EthernetCom.py#L28-L46) for automatic S7-1200 big-endian compatibility. *(Done)*
   - Remove the hardcoded 90.0° rotation value in `RealRobotExecutor` in [modules/scheduler.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/scheduler.py#L386-391) and replace with a dynamic $\theta$ angle supplied by the vision system.
2. **Vision Integration (Next milestone)**:
   - Set up a real camera and build an image processing module to classify PCB types (25×25 mm and 40×40 mm) and measure the PCB tilt angle $\theta$ using OpenCV or a YOLO model.
3. **PC-side Workspace Safety Check**:
   - Add a kinematic workspace boundary check on the PC as a redundant safety layer (current motion limits are hardcoded directly on the PLC to keep the Python layer flexible).
4. **Profile Smoothing**: Add jerk/acceleration-limited profiles on top of the mandatory 3D slope waypoints.
5. **Calibration Utility**: Fix and integrate `modules/calibration.py` to auto-profile Ethernet round-trip latency and mechanical movement delays.

---

## 6. Known Bugs & Limitations

### 6.1. Logic & Algorithm
1. **Early exit in pick position prediction (`_predict_pick_position` in `scheduler.py`)** — **[FIXED]**:
   - Workspace boundary check was inside the convergence loop. Fixed by bounding iterations to when the object enters the pickup window (`t_enter`).
2. **Hardcoded pick/release segment timing (`_build_pick_timing` in `scheduler.py`)** — **[FIXED]**:
   - Timing is now computed dynamically from actual diagonal blend distance and nominal speeds.
3. **Memory leak in scheduler** — **[FIXED]**:
   - `self.seen_object_ids` was an unbounded set; converted to a dict and pruned periodically using `stale_timeout_s`.
4. **Missing statistics counter** — **[FIXED]**:
   - `skipped_outside_workspace` counter added and incremented correctly when an object drifts past the lower workspace boundary.

### 6.2. PLC Integration & Simulation Limitations
1. **`argument_time` array has no effect on real hardware**:
   - The current PLC program does not implement trajectory time planning; actual robot motion speed is unaffected by the timing values sent from the PC.
2. **GOTO and PICK trajectories must be sent as separate phases**:
   - The Omron PLC has no internal trajectory time planner (segment travel time is opaque to the PC), so the PC must split the motion into two separate phases to control pick timing precisely. Real hardware testing confirmed smooth motion because the descent and ascent segments are short enough.
3. **Tag name inconsistency**:
   - The correct PLC-side command trigger tag is `bit_doing`. This key must be synchronized across all PC-side communication data structures.
4. **`goto_relative` command not implemented on PLC**:
   - Command ID 1 (`goto_relative`) has not been programmed on the physical PLC.
5. **Calibration test failure in `calibration.py`**:
   - The mechanical delay calibration sends a `stop` command (ID = 0) and waits for `task_doing == 1`. The mock PLC sets `task_doing` to the command ID (0 for `stop`), so the calibration step always times out.
6. **Hardcoded plot path in `run_test.py`**:
   - `generate_plots()` writes to a hardcoded path from an old session that may not exist or may not be writable.
