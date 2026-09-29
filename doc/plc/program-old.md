# Omron `Program0` — `Matching_Code_10` (deployed)

> This is the program running on the cell with the current Python. Source: the Sysmac
> project `OMRON/matching code/Matching_Code_10.smc2`, read from its XML (declarations, ST
> bodies, ladder rungs). The `.md` exports and screenshots in the same folder are a subset of
> it. Where they disagree, the project file wins. Defect IDs: `P*` are shared with the target
> program, `O*` are specific to this program. Both are defined in
> [`version-diff-and-defects.md`](version-diff-and-defects.md).
>
> The simulator core [`modules/plc_sim/omron_core.py`](../../modules/plc_sim/omron_core.py)
> is a scan-by-scan port of this file. Numbers quoted as "simulated" come from it with an
> ideal servo, i.e. setpoint = actual.

## 1. Project contents

| POU | Kind | Same as target? |
|---|---|---|
| `Program0 / Section0` | Ladder + inline ST, 22 rungs | no — this file |
| `Calc_Inverse_Kinematics`, `Calc_Angles_YZ`, `Calc_Forward_Kinematic` | ST functions | identical — [`kinematics.md`](kinematics.md) |
| `Goto_Absolute`, `MC_Home_Delta` | Ladder FBs | identical — [`motion-fbs.md`](motion-fbs.md) §3–4 |
| `MC_Inter_Curve_Vel` | ST FB | differs — §4 below |

**Data types.**
- The PC contract structs are `From_pc` and `From_plc` (old column of
  [`data-contract.md`](data-contract.md) §1).
- `InforRobot`, `Input_Position`, `Angle_Theta`, `Bool_Theta`, `Speed_Theta` and `Workspace`
  are declared but not used.

**Built-in I/O.**

| Signal | Point |
|---|---|
| `Home_Axis1..3` | `Input_Bit_00..02` |
| `Mc_On_Btn` (Servo-ON button) | `Input_Bit_03` |
| `Res_Err_Btn` | `Input_Bit_04` |
| `Home_Lamp1..3` | `Output_Bit_00..02` (not driven by any rung) |
| `Pump_Out` | `Output_Bit_03` |
| `Valve_Out` | `Output_Bit_04` |

**Interpolator limits.**
- `ICV_Vmax = 300`, `ICV_Amax = 1000`, `ICV_Dmax = 1000` (mm/s, mm/s²).
- These are initial values of `Program0` variables, and no rung writes them.

## 2. Rung map

| Rung | Kind | Purpose |
|---|---|---|
| 0 | Ladder | Servo power: `MC_Power` × 3 |
| 1 | Ladder | Home-switch stop: `MC_Stop` × 3, resets some motion triggers |
| 2 | Ladder | Servo-ON push-button self-hold |
| 3 | Ladder | Error reset: `MC_Reset` × 3, resets homing and servo flags |
| 4 | ST `Main_Section0` | PC command dispatcher |
| 5 | Ladder | Homing trigger: `MC_Home_Delta` |
| 6 | Ladder | Calibration move and re-zero |
| 7 | Ladder | `Home_Done` |
| 8 | ST `Main_Section1` | Homing status → `plc_package` |
| 9 | Ladder | Forward kinematics of `Act.Pos`, every scan |
| 10 | Ladder | IK debug instance (`X_cal/Y_cal/Z_cal`), no motion |
| 11 | Ladder | `Goto_Absolute` |
| 12 | ST `Main_Section2` | Goto status → `plc_package` |
| 13–18 | Ladder | Six chained `MC_Inter_Curve_Vel` instances |
| 19 | ST `Main_Section3` | Pump state machine |
| 20 | Ladder | `Pump_Out := Pump_Ext`, `Valve_Out := NOT Pump_Ext` |
| 21 | ST `Main_Section4` | Telemetry → `plc_package` |

## 3. Rungs

### Rungs 0–3 — power, limit stop, reset

**Rung 0 — servo power.** `Pw_Power_0..2` are enabled by `Pw_servo_on OR Mc_On_Btn_Flag`.
- `Pw_servo_on` is a local variable with initial value FALSE, and no rung writes it.
- Servo power therefore follows the panel button's self-hold (rung 2).
- There is one `MC_Power` per axis.

**Rung 1 — home-switch stop.** When (`Home_Axis1 OR Home_Axis2 OR Home_Axis3`) `AND NOT MC_Home_Ext`:
- `MC_Stop4..6` run on all three axes;
- `MC_Goto_Abs`, `ICV_Start_NewTurn`, `ICV_Blend_Done_0` and `ICV_Blend_Done_1` are reset.

Instances 2–5 of the chain are not reset. A segment that is running continues to call its
`MC_SyncMoveAbsolute`, and `task_state` stays 2 (**P10**).

**Rung 3 — reset.** `Res_Err_Btn` runs `MC_Reset` on the three axes and resets:
- `MC_Home_Ext`;
- `Mc_On_Btn_Flag`, so servo power drops until Servo-ON is pressed again;
- `Start_Home_Flag` and `Home_Done_Flag1..3`.

### Rung 4 — `Main_Section0`: dispatcher

This rung is the same as the target program ([`program-main.md`](program-main.md) rung 4),
except for command 3.

**What command 3 does.**
1. It sets `ICV_Start_NewTurn := TRUE`.
2. It copies `argument_x/y/z/e[0..6]` into `ICV_Pos_X/Y/Z/E[0..6]`.
3. It writes `task_doing := 3`, `task_state := 2`.

`argument_number` and `argument_time` are not read.

**It is accepted in any state.**
- If the chain is still running, its waypoint arrays change under it and instance 0
  restarts (**O2**).
- Nothing sets `task_state := 1` after a trajectory (**P5**).

**Commands 5 and 6** write `Pump_Ext`, but rung 19 overwrites it later in the same scan, so
they have no effect.

### Rungs 5–8 — homing

The logic is as in [`motion-fbs.md`](motion-fbs.md) §4 and `program-main.md` rungs 5–8.

**Timing.**
1. The axes search their switches at 12 °/s.
2. When the last switch is reached, `MC_Home_Delta` pulses `Done` through a 5 s `TP`.
3. Rung 6 moves to 28.9° / 27.5° / −27.7° at 10 °/s and 20 °/s² (≈ 3.4 s for the largest).
4. `MC_Home` then zeroes the axes.

`Home_Done` needs the three `Home_test_N.Done` together. A calibration angle whose move takes
longer than the 5 s window (above ≈ 45°) would leave homing unfinished (**O10**).
`Home_Error` is never written (**P13**).

### Rung 9 — FK

- `Calc_Forward_Kinematic(Act.Pos)` → `Fwk_Calc_OutX/Y/Z` → `pos_EE`.
- The function's error result is assigned to a misnamed local variable and never returned
  (**O9**).

### Rungs 11–12 — `Goto_Absolute`

- The FB computes IK of `Goto_abs_x/y/z` every scan.
- On the rising edge of `MC_Goto_Abs` it starts three independent `MC_MoveAbsolute`
  (15 °/s, 30 °/s²).
- `Done` is set when all three are done and reset on the falling edge of `MC_Goto_Abs`.
- Rung 12 then clears `MC_Goto_Abs` and writes `task_state := 1`.
- An unreachable target is not reported (`Goto_Abs_Error` is never written). A target that
  trips only the joint limit moves to the unwritten IK outputs (**O1**).
- If another motion instruction takes the axes mid-move, `Done` never comes and
  `MC_Goto_Abs` stays TRUE. Every later command 2 is then ignored (**O6**).

### Rungs 13–18 — the six-instance trajectory chain

Instance *k* (*k* = 0…5) is wired as follows.

| Pin | Value |
|---|---|
| A, B, C | `Pos[k]`, `Pos[k+1]`, `Pos[k+2]` (C unused for *k* = 5) |
| `V_Start_Req` | 0 for *k* = 0, else `ICV_Vend_{k−1}` |
| `Execute` (VAR_IN_OUT) | `ICV_Start_NewTurn` for *k* = 0, else `ICV_Blend_Done_{k−1}` |
| `Blend_mode` | TRUE for *k* = 0…4, FALSE for *k* = 5 |
| `Out_Done_Blend` → | `ICV_Blend_Done_k`, a one-scan pulse |
| `Out_V_End` → | `ICV_Vend_k` |
| `t_total_estimate` → | `ICV_t[k]` |
| `ICV_t5_out` → | `k_t` (instance 5 only) |

**How the handover works.**
- Every instance owns its own `MC_Sync_Axis1..3` on the same three axes.
- The rising `Execute` of instance *k+1*'s `MC_SyncMoveAbsolute` aborts instance *k*'s
  (multi-execution), and the aborted instance can no longer move the axes.

**Consequences.**
- **Junction stalls (P12).** At each junction the setpoint repeats for two to three scans.
  The simulator shows 6 zero-velocity setpoint intervals per 7-point trajectory.
- **Only instances 0–4 are dangerous during a new dispatch.** While any of them is busy, it
  will later raise `Blend_Done` and start the next instance on whatever the arrays contain
  by then (**O2**). An instance 5 that is still busy is harmless, because it ends without a
  pulse.
- **A zero-length segment stops the chain (O3).** Segment *k* with `L = 0` returns
  `Execute := FALSE` without `Out_Done_Blend`. The same happens if a blend corner reverses
  exactly (`V_end = 0` → State 3 instead of blend). The arm then stays at `Pos[k]`.
- **The rest-to-rest S-curve branch never runs in production.** It runs only when instance 5
  starts from rest, and in a 7-point chain `ICV_Vend_4 > 0`. Every production segment is a
  trapezoid.

### Rung 19 — `Main_Section3`: pump

**Step tracking.**
- `Current_Step := 0` while `ICV_Start_NewTurn` is TRUE, and `k + 1` on
  `ICV_Blend_Done_k`.
- It stays 5 after a trajectory, because instance 5 never pulses. Step 6 is unreachable.

**Pump output.**
- For steps 0–4: `Pump_Ext := (ICV_Pos_E[step] = 1)`. Segment *i* uses `E[i]`, and `E[6]`
  is never read.
- For step 5: `Pump_Ext := (E[5] = 1) AND fb_PumpTimer.Q`.

**The release timer.**
- `fb_PumpTimer` is declared `TP`.
- It starts when step 5 is entered, with `PT = 0.5 · Mem_ICV_t5`.
- `Mem_ICV_t5` latches `k_t`, which instance 5 computes one scan later, so `PT` comes from
  the previous trajectory (**O5**).

**Between trajectories** the pump holds the value of the last step.
- A command 3 with `Pos[0] = Pos[1] =` the current pose stops at segment 0 (O3) with
  `Current_Step = 0`.
- The pump then holds `E[0]` and the arm does not move.
- This is the only working "pump on/off in place" command on this program.

### Rung 21 — `Main_Section4`: telemetry

- `pos_angular[0..2] := Act.Pos`.
- `pos_EE[0..2] := Fwk_Calc_Out*`.
- `Total_Time_Estimate[0..6] := ICV_t[0..6]`. `ICV_t[6]` is never written.

## 4. `MC_Inter_Curve_Vel` — differences from the target version

The state machine and the profile formulas are those of [`motion-fbs.md`](motion-fbs.md)
§2, with these differences.

**Blend exit speed with corner look-ahead.** θ is the angle at B between AB and BC:

`V_end = min(V_max·sqrt((1 + cosθ)/2), sqrt(V_start² + 2·A·L), V_max)`.

There is still no backward pass, and the braking case still uses `+A` (**P7** faults 2–3).
In the simulator:
- The pick phase's final 5 mm descent is entered at ≈ 263 mm/s.
- The setpoint reaches the place point after ≈ 20 ms, and then holds for the rest of the
  planned 0.26 s.

**Time estimate.** `t_total_estimate = T_total + 0.08 s` when `V_Start_Req = 0`.

**State 3** leaves on `MC_Sync*.Busy = FALSE` only. The 0.0005° test sets
`Out_Done_Inter`, but nothing reads it, so **P3** does not exist here.

**IK errors are sticky.** State 99 exits only when `Out_Error_IK_OWS` is FALSE, and that
output is written FALSE only in State 0. The declaration comment reads "Err_IK must reset
manual". One IK error locks instance *k* until the PLC restarts (**P2**).

## 5. Declared but unused

These are dead declarations, safe to ignore when reading the project:
- `Move_abs_0..2`, `Moveto_Intermediary`, `Moveto_Midpoint*` (`MC_MoveAbsolute`);
- `MC_Stop1..3`, `Stop1` (`MC_Power`);
- `MC_Inter_Curve_Vel_6`, `ICV_Blend_Done_6`, `ICV_Vend_6`, `ICV_t0..t5`;
- `Exe_Inter1/2`, `Count1..4`, `when_it_turn2`, `another3/4`, `Goto_Abs2*`, `Check_IK`;
- `Axis*_test`, `Pos_calib1..3`, `MC_ICV_Err`, `Pump_Res`, `MC_Move_Abs`;
- the global `Out_Done_Inter`.
