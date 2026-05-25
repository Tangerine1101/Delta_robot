# Delta Robot System Reference Document

This document serves as the comprehensive technical reference for the Delta Robot Pick-and-Place sorting system. It consolidates hardware specifications, coordinate system definitions, mathematical foundations, thread layouts, and detailed software logic for both AI context preservation and human developer review.

---

## 1. System Overview & Architecture

The Delta Robot system is designed to pick moving products from a conveyor belt and sort/place them into destination bins based on visual feedback and encoder-speed synchronization. 

The runtime logic is partitioned into four primary components:
1. **`main.py`**: The central entry point orchestrating operation modes.
2. **`modules/cli.py`**: Interactive command-line command builder and parser.
3. **`modules/EthernetCom.py`**: The communication gateway handling Ethernet connection (via `pylogix` to Omron PLC) and packet normalization.
4. **`modules/scheduler.py`**: The core decision engine containing trajectory generation, simulated perception/speed streams, pick planning, and execution scenarios.
5. **`modules/test_module.py`**: A standalone fake PLC simulator generating mock motion traces for dry-run testing.

```
                  ┌──────────────────────┐
                  │      main.py         │
                  └──────────┬───────────┘
                             │ (runs/spawns)
            ┌────────────────┴────────────────┐
            ▼                                 ▼
   ┌─────────────────┐               ┌─────────────────┐
   │ CLI Interactive │               │  Scheduler Loop │
   │   (cli.py)      │               │ (scheduler.py)  │
   └────────┬────────┘               └────────┬────────┘
            │ (sends packets)                 │ (sends plans)
            ▼                                 ▼
   ┌──────────────────────────────────────────────────┐
   │             EthernetCom.py (Gateway)             │
   └────────────────────────┬─────────────────────────┘
                            │ (Ethernet / pylogix)
                            ▼
   ┌──────────────────────────────────────────────────┐
   │                PLC Hardware Layer                │
   │      - Omron NX1P2 (Main 3-DOF Arm via CSP)      │
   │      - Siemens S7-1200 (4th DOF & Conveyor)      │
   └──────────────────────────────────────────────────┘
```

---

## 2. Hardware Specifications

The cell utilizes a hybrid control layout combining Omron and Siemens PLCs to coordinate motion and conveyor feedback.

```
┌──────────────────────────────────────────────────────────────────┐
│                       HARDWARE SYSTEM                            │
│                                                                  │
│  ┌──────────┐    Ethernet / EtherNet/IP / TCP                    │
│  │  PC      │ ──────────────────────────┬────────────────────────┐
│  │ (Python) │                           │                        │
│  └──────────┘                           ▼                        ▼
│                              ┌─────────────────────┐  ┌─────────────────────┐
│                              │  Omron NX1P2        │  │  Siemens S7-1200    │
│                              │  CPU: 1140DT        │  │  High Freq I/O      │
│                              │                     │  │                     │
│                              │  EtherCAT Master    │  │  Conveyor Steppers  │
│                              └────────┬────────────┘  │  4th DOF Stepper    │
│                                       │ EtherCAT      └─────────────────────┘
│                      ┌────────────────┼────────────────┐
│                      │                │                │
│                      ▼                ▼                ▼
│               ┌────────────┐   ┌────────────┐   ┌────────────┐
│               │ Driver #1  │   │ Driver #2  │   │ Driver #3  │
│               │ MADLN05BE  │   │ MADLN05BE  │   │ MADLN05BE  │
│               └─────┬──────┘   └─────┬──────┘   └─────┬──────┘
│                     │                │                │
│                     ▼                ▼                ▼
│               ┌────────────┐   ┌────────────┐   ┌────────────┐
│               │ Motor #1   │   │ Motor #2   │   │ Motor #3   │
│               │ MSMF012L1T2│   │ MSMF012L1T2│   │ MSMF012L1T2│
│               └────────────┘   └────────────┘   └────────────┘
│                     │                │                │
│                     └────────────────┼────────────────┘
│                                      │
│                                      ▼
│                              ┌───────────────┐
│                              │  DELTA ROBOT  │
│                              │  (4-DOF Arm)  │
│                              └───────────────┘
└──────────────────────────────────────────────────────────────────┘
```

### 2.1. Main Controller: Omron NX1P2-1140DT
* **CPU Family**: NX1P2 Machine Automation Controller.
* **Role**: Primary controller running EtherCAT Master. It coordinates the main 3-DOF arms.
* **Task Period**: Down to 0.5 ms (Primary Periodic Task).
* **Communication**: EtherNet/IP for communicating with Python PC (`pylogix` tag reads/writes).
* **Tags Exposed**:
  * `pc_package`: Commands from PC to PLC.
  * `plc_package`: Status from PLC to PC.

### 2.2. Axis Drivers: Panasonic MADLN05BE (MINAS A6BE)
* **Quantity**: 3 units.
* **Rated Power**: 50 W.
* **Protocol**: EtherCAT (CANopen over EtherCAT - CoE).
* **Control Mode**: **CSP (Cyclic Synchronous Position)**. The PLC calculates and sends target positions to all drives cyclically.
* **Feedback**: 23-bit absolute encoder feedback (8,388,608 pulses/revolution).

### 2.3. Axis Motors: Panasonic MSMF012L1T2 (MINAS A6 Family)
* **Quantity**: 3 units.
* **Rated Power**: 100 W.
* **Rated Torque**: 0.32 N·m (Peak 0.96 N·m).
* **Rated/Max Speed**: 3000 / 5000 RPM.
* **Caution**: The driver rated power (50W) is lower than the motor rated power (100W). Torque and current limits are configured defensively in Sysmac Studio.

### 2.4. Secondary Controller: Siemens S7-1200
* **Role**: Handles high-frequency inputs and stepper outputs that are timing-critical:
  * Reads the conveyor belt's physical encoder feedback to compute real conveyor speed.
  * Controls two conveyor drive steppers.
  * Controls the stepper motor driving the **4th DOF (suction cup rotation)**.
* **Interface**: Modbus TCP or Ethernet socket exchanging 4-DOF rotation targets and speed change commands.

---

## 3. Coordinate System & Geometry

The Delta Robot uses a right-handed Cartesian coordinate system centered under the robot's base plate.

### 3.1. Negative Z-Axis Rule
Due to physical parallel arm kinematics:
* **$Z \approx 0.0$ mm**: Arm is fully retracted (close to the base plate).
* **$Z < 0$ (negative)**: Arm is extended downwards toward the conveyor belt (e.g., $-290.0$ to $-310.0$ mm).

### 3.2. Safety Invariants
To prevent mechanical crashes and servo strain, the scheduler enforces the following height hierarchy at initialization:
$$\text{clearance\_height} > \text{slope\_transition\_height} > \text{pre\_pick\_height} > \text{pickup\_height}$$
*(Example values in `config.json`: $-290.0 > -295.0 > -300.0 > -310.0$)*

Furthermore, during travel:
* `clearance_height` (safe horizontal transfer plane) must be higher than or equal to `place_height`.
* Out-of-bounds target coordinates are filtered using the configured bounding box:
  * `pickup_window_x`: $[X_{min}, X_{max}]$ (e.g., $[-120.0, 50.0]$)
  * `pickup_window_y`: $[Y_{min}, Y_{max}]$ (e.g., $[-60.0, 60.0]$)

---

## 4. Mathematical Foundation

### 4.1. Conveyor Speed Model
In offline test scenarios, conveyor speed varies dynamically. In production, speed is retrieved as a 2D velocity vector $\mathbf{v}_{\text{conveyor}} = [v_x, v_y]^T$ derived from S7-1200 encoder pulses.

$$\mathbf{v}(t) = [v_x(t), v_y(t)]^T$$

The default conveyor model assumes the belt moves along **positive Y**. Simulated throughput objects spawn at a low Y value and keep X fixed on one of the configured lane positions.

### 4.2. Position & Interception Time Prediction
When a camera detects a product at time $t_{\text{detect}}$ at position $\mathbf{P}_{\text{detect\_xy}} = [x_{\text{detect}}, y_{\text{detect}}]^T$, the scheduler predicts the point of interception $\mathbf{P}_{\text{pick}}$:

$$\mathbf{P}_{\text{pick\_xy}}(t) = \mathbf{P}_{\text{detect\_xy}} + \mathbf{v}_{\text{conveyor}} \times (t_{\text{pick}} - t_{\text{detect}})$$

To solve for the interception time $t_{\text{pick}}$ (and its corresponding dispatch time), Python uses an iterative fixed-point algorithm:
1. Initialize guess: $t_{\text{pick}}^{(0)} = t_{\text{now}} + \max(t_{\text{intercept\_lead\_time}}, t_{\text{comm\_delay}})$.
2. Calculate predicted interception position $\mathbf{P}_{\text{pick}}^{(k)}$ using $t_{\text{pick}}^{(k)}$.
3. Build the virtual `goto` trajectory from the current robot position to $\mathbf{P}_{\text{pick}}^{(k)}$.
4. Compute total duration of this `goto` segment $\Delta t_{\text{goto}}$ based on nominal speeds:
   $$\Delta t_{\text{segment}} = \max\left(0.08, \frac{d_{xy}}{V_{\text{nom\_xy}}}, \frac{d_z}{V_{\text{nom\_z}}}\right)$$
5. Update guess: $t_{\text{pick}}^{(k+1)} = t_{\text{now}} + \sum \Delta t_{\text{goto}} + t_{\text{comm\_delay}}$.
6. Iterate until $|t_{\text{pick}}^{(k+1)} - t_{\text{pick}}^{(k)}| < 0.01$ s (typically converges in $< 6$ iterations).
7. If the final converged coordinates $\mathbf{P}_{\text{pick}}$ fall outside the `pickup_window`, the detection is dropped.

### 4.3. Pick Dispatch Delay Compensation
Because the Omron PLC does not schedule future trajectory executions internally, the PC dictates execution timing. The PC sends the `goto` package immediately, but delays sending the critical `pick` descent package:

$$t_{\text{dispatch}} = t_{\text{pick}} - t_{\text{robot\_movement\_delay}} - t_{\text{ethernet\_delay}}$$

* **$t_{\text{robot\_movement\_delay}}$**: Physical mechanical acceleration delay (from $D_{\text{goto}}$ down to $A_{\text{pick}}$), default $0.05$ s.
* **$t_{\text{ethernet\_delay}}$**: Network delay over EtherNet/IP connection, default $0.002$ s.

---

## 5. Trajectory Templates

Every pick-and-place operation consists of two sequential phases, each using a **4-point template** aligned with the 4-slot PLC package.

```
       B_goto (Clearance) ── mandatory 3D slope down ──> C_goto (Slope transition)
              ▲                                                  │
              │                                                  ▼
        A_goto/start                                     D_goto (Pre-pick)
                                                                  │
                                                                  ▼
                                                           A_pick (Suction ON)
                                                                  │
                                                                  ▼
       C_pick (Clearance) <── mandatory 3D slope up ──── B_pick (Slope transition)
              │
              ▼
       D_pick/place (Release)
```

### 5.1. Goto Trajectory (Moving to Pre-Pick)
Designed to move the arm from its current resting position $\mathbf{P}_{\text{start}}$ to the pre-pick point above the moving object.
* **Point A**: Vertical lift to `clearance_height` above $\mathbf{P}_{\text{start}}$. Gripper state: `0` (OFF).
* **Point B**: Clearance-height offset from start toward the predicted pickup XY by `corner_blend_xy`. Gripper state: `0` (OFF).
* **Point C**: Alignment directly above predicted pickup XY at `slope_transition_height`. The segment `B_goto -> C_goto` is a required 3D slope with both XY and Z motion. Gripper state: `0` (OFF).
* **Point D**: Vertical descent to `pre_pick_height`. Gripper state: `0` (OFF).

### 5.2. Pick Trajectory (Pick & Sort)
Designed to execute the physical grab, lift, transfer, and drop at the bin.
* **Point A**: Descent to `pickup_height` at the interception coordinate. Gripper state: `1` (ON).
* **Point B**: Vertical ascent at the pickup XY to `slope_transition_height`. Gripper state: `1` (ON).
* **Point C**: Clearance-height blended position offset from the sort bin. The segment `B_pick -> C_pick` is a required 3D slope with both XY and Z motion. Gripper state: `1` (ON).
* **Point D**: Final descent to `place_height` at the bin coordinate. Gripper state: `0` (OFF/Release).

---

## 6. Software Module Descriptions

### 6.1. `main.py`
The orchestration entrypoint. It reads command-line switches (`--cli` or `--scheduler`), parses optional overrides (`--ip`, `--port`, `--scenario`, `--simulate-executor`), and manages runtime lifecycle. It spawns the dedicated PLC communication worker process to ensure that network blocking does not stutter the main scheduler or user CLI thread.

### 6.2. `modules/cli.py`
A CLI interpreter translating user input strings into command packages. Supported operations include:
* `stop` (stops execution)
* `go <theta1> <theta2> <theta3>` (direct motor angle control)
* `goto <x> <y> <z>` (Cartesian absolute move)
* `go_trajectory <name>` (executes pre-packaged demo paths)
* `calib` / `pick` / `release` (utility inputs)
* `status` (polls PLC status tags)

### 6.3. `modules/EthernetCom.py`
Implements `PLCGateway` which wraps the `pylogix` EtherNet/IP driver. It loads `config.json`, matches commands to integer IDs, and normalizes command packets.
Every packet written to the PLC tag `pc_package` conforms to a 4-slot array layout:
* `commandID`: `int` (mapped ID)
* `argument_number`: `int` (count of active points)
* `argument_x`, `argument_y`, `argument_z`, `argument_time`: `list[float]` of length 4.
* `argument_e`: `list[int]` of length 4 (gripper state).
* `doing_bit`: `int` (PC sets to `1` on write, PLC resets to `0` upon ingestion).

### 6.4. `modules/scheduler.py`
Contains the core scheduler `PickScheduler`, settings parser `SchedulerSettings`, and scenario execution loop.
* **Scenarios**:
  * `test_throughput`: Simulates a continuous stream of moving items on different lanes to profile throughput queues.
  * `test_accuracy`: Emits fixed-point objects and writes high-resolution tracking logs to `data.log` for positional error profiling.
* **Executors**:
  * `SimulatedExecutor`: Increments plan counters and writes logs without hitting hardware.
  * `RealRobotExecutor`: dispatches trajectory packets sequentially, waits for completion based on trajectory duration, and polls the PLC status.

### 6.5. `modules/test_module.py`
A lightweight TCP JSON-lines mock server listening on port `1502` (or config default). It reads incoming command lines, interpolates mock robot coordinates over time, and outputs simulated trajectories to `test_module.log`. Used for system integration dry runs.

---

## 7. Threading Model

The full system vision operates across four concurrent threads to prevent locking:

| Thread | Responsibility | Mechanism |
|--------|----------------|-----------|
| **Thread 1: Communication** | Real-time EtherNet/IP and TCP reads/writes with PLCs. | Background Process (`multiprocessing` worker) |
| **Thread 2: Decision** | CLI parser or Auto Pick-Scheduler loop. | Main thread |
| **Thread 3: Perception** | Image frame ingestion, item segmentation, and tracking. | Future integration module |
| **Thread 4: User Interface** | Local diagnostic plots and future web GUI database. | Future web/dashboard thread |
