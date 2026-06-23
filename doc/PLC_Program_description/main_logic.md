# Main Program Logic: `Program0 / Section0`

This document presents the complete execution logic of the Delta Robot's main PLC program, combining **ladder logic** (rungs 0–21) and **inline Structured Text sections** (Section0–Section4). The program runs in the Omron NX1P2's **Primary Periodic Task** at a 4 ms cycle.

Source: Ladder images [Main_1–Main_13](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Program_Main/Main_img/) and [All_Sections.txt](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Program_Main/All_Sections.txt)

---

## 1. Program Architecture Overview

The main program is organized into **22 rungs** (0–21) that execute sequentially every 4 ms cycle. It combines ladder logic for hardware I/O and motion function blocks, with inline ST sections for command dispatching and trajectory coordination.

```
┌─────────────────────────────────────────────────────────────────┐
│                    PRIMARY PERIODIC TASK (4 ms)                  │
├─────────────────────────────────────────────────────────────────┤
│  Rung 0     │ Servo Power Enable (MC_Power × 3 axes)           │
│  Rung 1     │ Emergency Stop (MC_Stop × 3 axes) + Flag Reset   │
│  Rung 2     │ Servo ON Button Latch (Self-hold circuit)        │
│  Rung 3     │ Error Reset (MC_Reset × 3 axes) + System Reset   │
│  Rung 4     │ [ST] Section0: PC Command Dispatcher             │
│  Rung 5     │ Homing Trigger (MC_Home_Delta FB)                │
│  Rung 6     │ Post-Home Calibration (MC_MoveAbsolute × 3)      │
│             │ + MC_Home × 3 (Omron encoder home)               │
│  Rung 7     │ Home Complete Latch (AND of 3 Home_test.Done)     │
│  Rung 8     │ [ST] Section1: Home Status Feedback              │
│  Rung 9     │ Forward Kinematics (real-time position monitor)  │
│  Rung 10    │ IK Test Instance (calibration/debug)             │
│  Rung 11    │ Goto_Absolute FB (test/calibration move)         │
│  Rung 12    │ [ST] Section2: Goto_Abs Status Feedback          │
│  Rung 13–18 │ Trajectory Pipeline (MC_Inter_Curve_Vel × 6)     │
│  Rung 19    │ [ST] Section3: Pump Timer & Blend Sequencer      │
│  Rung 20    │ Pump & Valve Output Driver                       │
│  Rung 21    │ [ST] Section4: Telemetry Feedback to PC          │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Rung-by-Rung Logic

### Rung 0: Servo Power Enable

**Function blocks:** `Pw_Power_0`, `Pw_Power_1`, `Pw_Power_2` (all `MC_Power`)

| Input | Source |
| :--- | :--- |
| Axis | `MC_Axis1`, `MC_Axis2`, `MC_Axis3` respectively |
| Enable | `Pw_servo_on` AND `Mc_On_Btn_Flag` (self-held by Rung 2) |

**Logic:** When `Pw_servo_on` is activated (from the physical Start button or PC command) AND the button latch `Mc_On_Btn_Flag` is set, all three servo drives receive the `MC_Power` enable command. The `Status` output confirms each drive is energized and ready for motion commands.

---

### Rung 1: Emergency Stop & Flag Reset

**Function blocks:** `MC_Stop4`, `MC_Stop5`, `MC_Stop6` (all `MC_Stop`)

**Trigger condition:** `Home_Axis1` OR `Home_Axis2` OR `Home_Axis3` (limit switch inputs) AND `MC_Home_Ext` with toggle logic.

**Logic:** When any of the three homing limit switches are triggered while homing is NOT active, the MC_Stop blocks execute an immediate controlled deceleration on all three axes simultaneously. On the right side of the rung, the following flags are **reset (R)**:

| Reset Flag | Purpose |
| :--- | :--- |
| `MC_Goto_Abs` | Cancel any pending absolute move command |
| `ICV_Start_NewTurn` | Cancel trajectory pipeline initialization |
| `ICV_Blend_Done_0` | Clear segment 0 blend completion flag |
| `ICV_Blend_Done_1` | Clear segment 1 blend completion flag |
| `MC_Goto_Abs` | (second reset for redundancy) |

This ensures all active motion commands and trajectory pipeline states are fully cleared when an emergency stop occurs.

---

### Rung 2: Servo ON Button Latch (Self-Hold)

**Logic:** Standard self-hold (seal-in) circuit for the physical Servo ON push button.

```
    ┌── Mc_On_Btn ──┐
    │                ├──( Mc_On_Btn_Flag )
    └── Mc_On_Btn_Flag ──┘
```

When `Mc_On_Btn` (physical push button) is pressed momentarily, `Mc_On_Btn_Flag` latches ON and stays ON through the self-hold branch. This flag is used as a persistent enable signal for Rung 0 (servo power) and Rung 5 (homing).

The flag is **Reset (R)** by Rung 3 (error reset) to force a complete servo re-initialization after fault clearance.

---

### Rung 3: Error Reset

**Function blocks:** `MC_RES_DB1`, `MC_RES_DB2`, `MC_RES_DB3` (all `MC_Reset`)

**Trigger:** `Res_Err_Btn` (physical Reset button on the control panel)

**Logic:** When the reset button is pressed, `MC_Reset` is executed on all three axes simultaneously to clear servo drive fault codes (e.g., overcurrent, following error, communication loss). Additionally:

| Reset Flag | Purpose |
| :--- | :--- |
| `MC_Home_Ext` (R) | Clear any pending home command |
| `Mc_On_Btn_Flag` (R) | Force servo power off — operator must press Servo ON again |

This implements a **safe restart protocol**: after clearing errors, the operator must explicitly re-enable servo power and re-home before the robot can move.

---

### Rung 4: [ST] Section0 — PC Command Dispatcher

This is the **communication gateway** between the Python vision/scheduling PC and the PLC. It implements a command-response handshake protocol.

**Protocol:** The PC writes a command to `pc_package.commandID` and sets `pc_package.bit_doing ≠ 0x00` to signal a new command. The PLC processes the command in a single scan cycle, then immediately clears `commandID := -1` and `bit_doing := 0x00` as an acknowledgment handshake.

| Command ID | Name | Action |
| :---: | :--- | :--- |
| 2 | **GOTO ABSOLUTE** | Copies X/Y/Z coordinates, sets `MC_Goto_Abs := TRUE` |
| 3 | **GO TRAJECTORY** | Copies 7-point waypoint array (X/Y/Z/E), sets `ICV_Start_NewTurn := TRUE` |
| 4 | **HOME** | Sets `MC_Home_Ext := TRUE` |
| 5 | **PICK** | Sets `Pump_Ext := TRUE` (activate vacuum) |
| 6 | **RELEASE** | Sets `Pump_Ext := FALSE` (deactivate vacuum) |

**Status feedback:** The PLC writes `task_doing` (current command ID) and `task_state` (1=Done, 2=Busy, 3=Error) to `plc_package` for the PC to poll.

**Command 3 detail:** The trajectory command loads a **7-waypoint array** (`ICV_Pos_X[0..6]`, `ICV_Pos_Y[0..6]`, `ICV_Pos_Z[0..6]`) plus an **end-effector action array** (`ICV_Pos_E[0..6]`, where 1=pump ON, 0=pump OFF). This array defines the complete pick-and-place trajectory.

---

### Rung 5: Homing Trigger

**Function block:** `MC_Home_Delta_0` ([MC_Home_Delta](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Function_Blocks/MC_Home_Delta/mc_home.md))

**Trigger:** `Pw_servo_on` AND `Mc_On_Btn_Flag` AND `MC_Home_Ext` (toggle 1)

**Logic:** Executes the custom homing sequence. The `MC_Home_Delta` FB drives all three arms upward until the limit switches trigger, then resets the encoder positions. Output `Done` signal feeds Rung 6.

---

### Rung 6: Post-Home Calibration & Encoder Origin Set

**Function blocks:** `Calib_Abs1/2/3` (`MC_MoveAbsolute`) and `Home_test_1/2/3` (`MC_Home`)

**Trigger:** `MC_Home_Delta_0.Done`

**Logic:** After the custom homing routine completes, this rung performs two operations in parallel:

1. **Calibration Move** — `MC_MoveAbsolute` drives each axis to a known calibration angle:
   - Axis 1: **28.9°**
   - Axis 2: **27.5°**
   - Axis 3: **−27.7°** (inverted due to servo parameter direction)
   
   These angles correspond to the home position of the delta robot where the end-effector is at a known safe Cartesian coordinate.

2. **Encoder Origin** — `MC_Home` (Omron standard) sets the current position as the zero reference for each axis encoder, establishing the absolute coordinate system.

---

### Rung 7: Home Completion Latch

**Logic:** Simple AND gate:

```
Home_test_1.Done AND Home_test_2.Done AND Home_test_3.Done → Home_Done
```

`Home_Done` is set only when ALL three axes have completed their calibration move and encoder origin set. This flag triggers Section1 to report completion to the PC.

---

### Rung 8: [ST] Section1 — Home Status Feedback

**Logic:** When `Home_Done` or `Home_Error` is detected:
- Clears `MC_Home_Ext := FALSE` (one-shot trigger)
- Updates `plc_package.task_state` to **1** (Done) or **3** (Error)

---

### Rung 9: Forward Kinematics Monitor

**Function:** `Calc_Forward_Kinematic` ([documentation](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Functions/calc_forward_kinematic.md))

| Input | Source |
| :--- | :--- |
| `Theta1` | `MC_Axis1.Act.Pos` |
| `Theta2` | `MC_Axis2.Act.Pos` |
| `Theta3` | `MC_Axis3.Act.Pos` |

| Output | Destination |
| :--- | :--- |
| `Calc_OutX/Y/Z` | `Fwk_Calc_OutX/Y/Z` |

**Logic:** Runs **every scan cycle** (4 ms) to continuously convert the actual servo encoder positions into Cartesian coordinates. These values are displayed on the HMI and sent to the PC via Section4. This provides real-time position monitoring independent of the trajectory generator.

---

### Rung 10: Inverse Kinematics Test Instance

**Function:** `Calc_Inverse_Kinematics` ([documentation](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Functions/inverse_kinematics.md))

| Input | Source |
| :--- | :--- |
| `X, Y, Z` | `X_cal, Y_cal, Z_cal` (debug/test variables) |

| Output | Destination |
| :--- | :--- |
| `Theta1/2/3` | `Out_TestAngle10/20/30` |

**Purpose:** A standalone IK test instance for calibration and debugging. Allows the engineer to input arbitrary Cartesian coordinates and observe the computed joint angles without triggering any motion.

---

### Rung 11: Goto_Absolute (Test/Calibration Move)

**Function block:** `Goto_Abs` ([Goto_Absolute](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Function_Blocks/Goto_Absolute/goto_absolute.md))

**Trigger:** `MC_Goto_Abs` (set by Section0 Command 2, self-held until done)

| Input | Source |
| :--- | :--- |
| `X_0, Y_0, Z_neg300` | `Goto_abs_x, Goto_abs_y, Goto_abs_z` |

**Purpose:** Moves the robot to a single absolute position. Used for testing and workspace calibration only — **not for production trajectory execution** (which uses the MC_Inter_Curve_Vel pipeline).

---

### Rung 12: [ST] Section2 — Goto_Abs Status Feedback

**Logic:** When `Goto_Abs.Done` or `Goto_Abs_Error`:
- Clears `MC_Goto_Abs := FALSE`
- Updates `plc_package.task_state` to **1** (Done) or **3** (Error)

---

### Rungs 13–18: Trajectory Pipeline (6 × MC_Inter_Curve_Vel)

This is the **core motion execution engine** — a daisy-chained pipeline of 6 instances of the [MC_Inter_Curve_Vel](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Function_Blocks/MC_inter_curve_vel.md) function block, each handling one segment of the 7-point trajectory.

#### Pipeline Wiring Pattern

| Rung | Instance | Segment | A→B | Look-ahead C | Execute Trigger | V_Start_Req |
| :---: | :--- | :---: | :--- | :--- | :--- | :--- |
| 13 | `MC_Inter_Curve_Vel_0` | 0→1 | `Pos[0]→Pos[1]` | `Pos[2]` | `ICV_Start_NewTurn` | `0` (from rest) |
| 14 | `MC_Inter_Curve_Vel_1` | 1→2 | `Pos[1]→Pos[2]` | `Pos[3]` | `ICV_Blend_Done_0` | `ICV_Vend_0` |
| 15 | `MC_Inter_Curve_Vel_2` | 2→3 | `Pos[2]→Pos[3]` | `Pos[4]` | `ICV_Blend_Done_1` | `ICV_Vend_1` |
| 16 | `MC_Inter_Curve_Vel_3` | 3→4 | `Pos[3]→Pos[4]` | `Pos[5]` | `ICV_Blend_Done_2` | `ICV_Vend_2` |
| 17 | `MC_Inter_Curve_Vel_4` | 4→5 | `Pos[4]→Pos[5]` | `Pos[6]` | `ICV_Blend_Done_3` | `ICV_Vend_3` |
| 18 | `MC_Inter_Curve_Vel_5` | 5→6 | `Pos[5]→Pos[6]` | N/A | `ICV_Blend_Done_4` | `ICV_Vend_4` |

#### Chaining Mechanism

```
  ICV_Start_NewTurn
        │
        ▼
  ┌──────────┐   Blend_Done_0   ┌──────────┐   Blend_Done_1
  │  Seg 0   │ ───────────────→ │  Seg 1   │ ───────────────→ ...
  │ Pos[0→1] │   V_End_0        │ Pos[1→2] │   V_End_1
  └──────────┘                  └──────────┘

  ... ───→ ┌──────────┐   Blend_Done_4   ┌──────────┐
           │  Seg 4   │ ───────────────→ │  Seg 5   │ → STOP
           │ Pos[4→5] │   V_End_4        │ Pos[5→6] │   (Blend=0)
           └──────────┘                  └──────────┘
```

**Key details:**
- **Segments 0–4** (`Blend_mode = 1`): Continuous path blending. Each segment outputs `Out_Done_Blend` and `Out_V_End`, which become the next segment's `Execute` trigger and `V_Start_Req` respectively.
- **Segment 5** (`Blend_mode = 0`): Final segment — decelerates to a complete stop.
- **Shared parameters:** All instances use `V_max = ICV_Vmax (300)`, `A_max = ICV_Amax (1000)`, `D_max = ICV_Dmax (1000)`.
- **Segment 5 special:** Its `ICV_t5_out` output connects to `k_t`, which is latched by Section3 for pump timing calculations.
- **Time estimates:** Each instance's `t_total_estimate` is stored in `ICV_t[0..5]` for telemetry.

---

### Rung 19: [ST] Section3 — Blend Sequencer & Pump Timer Control

This section manages the vacuum pump timing in synchronization with the trajectory pipeline.

#### Part 1: Time Latching
```
IF k_t > 0.0 THEN
    Mem_ICV_t5 := k_t;
END_IF;
```
Captures the last segment's time estimate from `MC_Inter_Curve_Vel_5` for use in pump timing. The latch holds the value even after the FB resets.

#### Part 2: Step Tracking
Monitors the `ICV_Blend_Done_N` flags to track which trajectory segment is currently active:

| Flag Detected | Current_Step Set |
| :--- | :---: |
| `ICV_Start_NewTurn` | 0 |
| `ICV_Blend_Done_0` | 1 |
| `ICV_Blend_Done_1` | 2 |
| `ICV_Blend_Done_2` | 3 |
| `ICV_Blend_Done_3` | 4 |
| `ICV_Blend_Done_4` | 5 |
| `ICV_Blend_Done_5` | 6 |

#### Part 3: Pump State Machine

The pump is controlled based on the end-effector action array `ICV_Pos_E[0..6]` and the current trajectory step:

- **Steps 0–4:** `Pump_Ext := (ICV_Pos_E[step] = 1)` — pump ON/OFF follows the action array directly.
- **Step 5 (approaching release):** A **TON timer** (`fb_PumpTimer`) is started. The pump stays ON only while the timer's Q output is TRUE. The timer preset is calculated as:

$$T_{pump} = 0.5 \times t_{segment5} \text{ (in nanoseconds, converted to TIME)}$$

This means the pump releases the object at **50% of the way** through the final segment — giving the object time to detach before the arm moves away.

- **Step 6 and default:** `Pump_Ext := FALSE` — pump always OFF.

---

### Rung 20: Pump & Valve Physical Output

**Logic:** `Pump_Ext` drives two physical outputs:

| Output | Type | Function |
| :--- | :--- | :--- |
| `Pump_Out` | Normal coil (O) | Activates the vacuum pump motor via SSR |
| `Valve_Out` | Negated coil (/) | Controls the Airtac solenoid valve (inverted logic — valve opens when `Pump_Ext = TRUE`, closes when FALSE) |

The negated coil on `Valve_Out` implements normally-closed valve logic: the valve is energized (open) when pumping, and de-energized (closed, venting to atmosphere) when releasing.

---

### Rung 21: [ST] Section4 — Telemetry Feedback to PC

Packs real-time PLC data into `plc_package` for the PC to read via TCP/IP:

| Data | Source | Destination |
| :--- | :--- | :--- |
| Axis angles (actual) | `MC_Axis1/2/3.Act.Pos` | `plc_package.pos_angular[0..2]` |
| End-effector XYZ | `Fwk_Calc_OutX/Y/Z` | `plc_package.pos_EE[0..2]` |
| Segment time estimates | `ICV_t[0..6]` | `plc_package.Total_Time_Estimate[0..6]` |

All values are converted from `LREAL` (internal double-precision) to `REAL` (32-bit float for communication efficiency).

---

## 3. Complete Execution Flow Diagram

```
  ┌─────────────────────────────────────────────────────────────┐
  │                    SYSTEM POWER ON                          │
  └──────────────────────────┬──────────────────────────────────┘
                             ▼
                    [Rung 2: Press Servo ON]
                             │
                             ▼
                    [Rung 0: MC_Power × 3]
                             │
                             ▼
              [Rung 5: MC_Home_Delta (custom homing)]
                             │
                             ▼
        [Rung 6: Calibration Move + Encoder Origin Set]
                             │
                             ▼
                [Rung 7: Home_Done = TRUE]
                             │
            ┌────────────────┼────────────────┐
            ▼                                 ▼
  [CMD 2: Goto_Abs]                  [CMD 3: Trajectory]
  (test/calibrate)                   (production pick & place)
            │                                 │
            ▼                                 ▼
  [Rung 11: Goto_Abs FB]         [Rungs 13–18: 6-segment
            │                     blended trajectory pipeline]
            ▼                                 │
  [Rung 12: Status → PC]         [Rung 19: Pump timing]
                                              │
                                              ▼
                                  [Rung 20: Pump/Valve output]
                                              │
                              ┌───────────────┼───────────────┐
                              ▼                               ▼
                    [Rung 9: FK monitor]            [Rung 21: Telemetry → PC]
                    (runs every cycle)              (runs every cycle)
```

---

## 4. Communication Protocol Summary

### PC → PLC (Commands)

| Field | Type | Description |
| :--- | :--- | :--- |
| `pc_package.bit_doing` | WORD | Non-zero = new command pending |
| `pc_package.commandID` | INT | Command identifier (2–6) |
| `pc_package.argument_x/y/z[0..6]` | REAL[] | Coordinate arrays |
| `pc_package.argument_e[0..6]` | INT[] | End-effector action (1=pick, 0=release) |

### PLC → PC (Status)

| Field | Type | Description |
| :--- | :--- | :--- |
| `plc_package.task_doing` | INT | Currently executing command ID |
| `plc_package.task_state` | INT | 1=Done, 2=Busy, 3=Error |
| `plc_package.pos_angular[0..2]` | REAL[] | Actual servo angles (degrees) |
| `plc_package.pos_EE[0..2]` | REAL[] | End-effector Cartesian position (mm) |
| `plc_package.Total_Time_Estimate[0..6]` | REAL[] | Segment time estimates (seconds) |
