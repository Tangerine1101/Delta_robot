# Omron `Program0` — `delta_paper_0_2`

> Rung-by-rung reading of the target program (source: `OMRON/delta_paper_0_2/program0.md`
> and its ladder screenshots). Rungs execute top to bottom every 4 ms scan; later rungs see
> values written by earlier rungs **in the same scan**. Known defects are only flagged here
> with their ID — details and patches are in
> [`version-diff-and-defects.md`](version-diff-and-defects.md).

## 1. Rung map

| Rung | Kind | Purpose |
|---|---|---|
| 0 | Ladder | Servo power — `MC_Power` × 3 |
| 1 | Ladder | Home-switch stop — `MC_Stop` × 3, resets motion triggers |
| 2 | Ladder | Servo-ON push-button self-hold |
| 3 | Ladder | Error reset — `MC_Reset` × 3, resets homing/servo flags |
| 4 | ST `Main_Section0` | PC command dispatcher |
| 5 | Ladder | Homing trigger — `MC_Home_Delta` |
| 6 | Ladder | Post-homing calibration move + encoder origin set |
| 7 | Ladder | `Home_Done` latch |
| 8 | ST `Main_Section1` | Homing status → `plc_package` |
| 9 | Ladder | Forward kinematics monitor (every scan) |
| 10 | Ladder | IK debug instance |
| 11 | Ladder | `Goto_Absolute` FB |
| 12 | ST `Main_Section2` | `Goto_Abs` status → `plc_package` |
| 13 | ST | `FB_ICV_Sequencer` call + pump control |
| 14 | Ladder | Pump / valve physical outputs |
| 15 | ST `Main_Section4` | Telemetry → `plc_package` |
| 16 | ST | Teaching mode |

## 2. Rungs

### Rung 0 — Servo power
`Pw_Power_0/1/2` (`MC_Power`) on `MC_Axis1..3`, `Enable := Pw_servo_on OR Mc_On_Btn_Flag`.
`FB_TeachingMode` (rung 16) owns a second set of `MC_Power` instances on the same axes —
defect **P9**.

### Rung 1 — Home-switch stop
Condition: (`Home_Axis1` OR `Home_Axis2` OR `Home_Axis3`) AND NOT `MC_Home_Ext`. Executes
`MC_Stop` on all three axes and resets `MC_Goto_Abs` and `ICV_Start_NewTurn`. This is the
only over-travel protection: the home switches double as limit switches outside homing. It
does not abort the trajectory sequencer — defect **P10**.

### Rung 2 — Servo-ON latch
`Mc_On_Btn` (panel button) seals in `Mc_On_Btn_Flag`.

### Rung 3 — Error reset
`Res_Err_Btn` → `MC_Reset` on all axes, and resets `MC_Home_Ext`, `Mc_On_Btn_Flag`,
`Start_Home_Flag`, `Home_Done_Flag1..3`. It does **not** touch `ICV_Err_Reset` — see **P2**.

### Rung 4 — `Main_Section0`: PC command dispatcher
Runs only when `pc_package.bit_doing <> 0`; at the end of the section it writes
`bit_doing := 0` (the acknowledgement the PC may poll). Every branch writes
`commandID := -1`.

| `commandID` | Action | `task_doing` / `task_state` written |
|---|---|---|
| 0 (stop) | **commented out** — no effect | — |
| 2 goto_absolute | `Goto_abs_x/y/z := argument_x/y/z[0]`, `MC_Goto_Abs := TRUE` | 2 / 2 (busy) |
| 3 go_trajectory | `ICV_Start_NewTurn := TRUE`; copies `argument_x/y/z/e[0..6]` into `ICV_Pos_X/Y/Z/E[0..6]` | 3 / 2 (busy) |
| 4 calibrate (home) | `MC_Home_Ext := TRUE` | 4 / 2 (busy) |
| 5 pick | `Pump_Ext := TRUE` — overwritten by rung 13 in the same scan, so no effect | 5 / 1 |
| 6 release | `Pump_Ext := FALSE` — likewise no effect | 6 / 1 |
| other (1, 10, …) | ignored, but `bit_doing` is still cleared | — |

Command 3 does not read `pc_package.n_points` and copies only indices 0..6 — defect **P1**.
Nothing ever sets `task_state := 1` for command 3 — defect **P5**.

### Rungs 5–8 — Homing
* Rung 5: `MC_Home_Delta_0` runs on the rising edge of `MC_Home_Ext` (with servo on). See
  [`motion-fbs.md`](motion-fbs.md) §4.
* Rung 6: on `MC_Home_Delta_0.Done`, `MC_MoveAbsolute` drives the axes to
  **28.9° / 27.5° / −27.7°** (10 °/s, 20 °/s²), then `MC_Home` sets that pose as encoder zero.
  These three angles are the mechanical calibration of the arm zero.
* Rung 7: `Home_Done := Home_test_1.Done AND Home_test_2.Done AND Home_test_3.Done`.
* Rung 8 (`Main_Section1`): on `Home_Done` or `Home_Error`, clears `MC_Home_Ext` and, if
  `task_doing = 4`, writes `task_state` 1 or 3. `Home_Error` is never set anywhere.

### Rung 9 — Forward-kinematics monitor
`Calc_Forward_Kinematic(MC_Axis1..3.Act.Pos)` → `Fwk_Calc_OutX/Y/Z` every scan. This is the
source of `pos_EE` — i.e. the PC sees the **actual** (encoder) pose, not the command.

### Rung 10 — IK debug instance
`Calc_Inverse_Kinematics(X_cal, Y_cal, Z_cal)` → `Out_TestAngle10..30`. No motion.

### Rungs 11–12 — `Goto_Absolute`
Rung 11 runs `Goto_Abs` while `MC_Goto_Abs` is TRUE ([`motion-fbs.md`](motion-fbs.md) §3).
Rung 12 (`Main_Section2`) clears `MC_Goto_Abs` on `Goto_Abs.Done` (or `Goto_Abs_Error`,
never set) and writes `task_state := 1` if `task_doing = 2`.

### Rung 13 — Trajectory sequencer + pump
1. Calls `Traj` (`FB_ICV_Sequencer`) with `Execute := ICV_Start_NewTurn`,
   `Abort := ICV_Abort`, `ErrReset := ICV_Err_Reset`, `N_Points := ICV_n`, `Max_Index := 31`,
   `V_max/A_max/D_max := ICV_Vmax/ICV_Amax/ICV_Dmax`, arrays `ICV_Pos_X/Y/Z`, `T_Seg := ICV_t`,
   axes by reference.
2. Lowers `ICV_Start_NewTurn` immediately after the call (the FB latches the rising edge).
   A command 3 arriving while `Traj.Busy` is therefore consumed and lost — **P4**.
3. Clears `ICV_Err_Reset` once `Traj.Error` is FALSE.
4. Pump:
   * `Seq_LastSegReady := Traj.IsLastSeg AND Traj.T_SegNow > 0` (waits until the last
     segment's duration has been computed).
   * On that edge `Mem_T_LastSeg := T_SegNow`; `fb_PumpTimer(IN := Seq_LastSegReady,
     PT := 0.5 · Mem_T_LastSeg)`.
   * `Pump_Ext :=`
     * `FALSE` when `NOT Traj.Busy`;
     * `ICV_Pos_E[SegIdx] = 1 AND (fb_PumpTimer.Q OR NOT Seq_LastSegReady)` on the last
       segment → vacuum released at 50 % of the last segment's duration (`fb_PumpTimer` is
       declared `TP`);
     * `ICV_Pos_E[SegIdx] = 1` otherwise.

   Consequences: segment *i* (point *i* → *i+1*) uses `E[i]`; `E[N-1]` is never read; the
   pump is always off when no trajectory runs, so commands 5/6 cannot hold suction.

### Rung 14 — Outputs
`Pump_Out := Pump_Ext`; `Valve_Out := NOT Pump_Ext` (valve de-energised = vacuum held;
energised = vent).

### Rung 15 — `Main_Section4`: telemetry
`pos_angular[0..2] := MC_AxisN.Act.Pos`; `pos_EE[0..2] := Fwk_Calc_Out*`;
`Total_Time_Estimate[0..6] := ICV_t[0..6]` (per-segment modelled durations, filled as each
segment starts). `Total_Time_Sum`, `current_step`, `n_points_ack` are declared but never
written — **P6**.

### Rung 16 — Teaching mode
`button_teaching_active := NOT button_teaching_ex` (NC contact); calls `Teach_FB`
(`FB_TeachingMode`) every scan; on `teaching_done` appends the taught X/Y/Z and three joint
angles to `Teach_Array_*[Teach_Index]` and increments `Teach_Index` (sets
`Teach_Array_Full_E` when full). The call does not pass `start_teaching_mode` — **P9**.
