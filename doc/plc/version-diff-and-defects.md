# Omron Program: Version Differences and Defect Review

> Review of both Omron programs against each other and against the Python side
> (`modules/comm/`, `modules/core/`, `modules/runtime/`):
> - `delta_paper_0_2` is the target and is **not yet validated on hardware**;
> - `Matching_Code_10` is deployed, and the current Python runs against it.
>
> **Sources.** Findings were checked against the Sysmac project files (`.smc2`
> declarations, ST bodies and ladder rungs). Items marked *verify* depend on a **run-time**
> behaviour of an Omron instruction and must be confirmed on the bench.
>
> **How the defects are grouped.** `P*` defects are described for the target program.
> §3a states which of them also exist in the deployed program and adds the `O*` defects
> specific to it. Behaviours marked "simulated" were reproduced with
> [`modules/plc_sim/omron_core.py`](../../modules/plc_sim/omron_core.py).
>
> Every open defect below is also registered in [`../open-issues.md`](../open-issues.md) §C.1.

## 1. What changed between the two versions

| Area | `Matching_Code_10` | `delta_paper_0_2` |
|---|---|---|
| Trajectory runner | 6 hard-wired `MC_Inter_Curve_Vel` instances (rungs 13–18), each chained by `ICV_Blend_Done_k`; fixed 7 points | One `MC_Inter_Curve_Vel` inside `FB_ICV_Sequencer`; N = 2…32 points; explicit Busy/Done/Error/SegIdx |
| Corner speed | Look-ahead: `V_end = min(V_max·sqrt((1+cosθ)/2), sqrt(V_start² + 2AL), V_max)` using point C | **None**: every blend segment targets `V_end = V_max` |
| Time estimate | `t_total_estimate` includes +0.08 s soft start when starting from rest | `Out_T_Segment` = profile time only |
| Axis references | FB used global `MC_Axis*` directly | Axes passed In/Out through every FB level |
| Pump control | `Current_Step` from `Blend_Done` flags; release at 50 % of segment 5 | From `SegIdx`/`IsLastSeg`; release at 50 % of the last segment; pump forced off when idle |
| `From_pc` | 7-element arrays, `argument_time` | `n_points` added, `argument_time` removed, 32-element arrays |
| `From_plc` | `Total_Time_Estimate[0..6]` | `[0..30]` + `Total_Time_Sum`, `current_step`, `n_points_ack` |
| Teaching | — | `FB_TeachingMode` + `Teach_Array_*` (rung 16) |
| Unchanged | Dispatcher (rung 4), homing, FK/IK, `Goto_Absolute`, outputs, telemetry section | |

## 2. Severity scale

* **Blocker** — the program cannot run the current production path correctly.
* **High** — wrong motion, lost commands, or a permanent lock-up in a reachable situation.
* **Medium** — degrades accuracy/timing or leaves a feature dead.
* **Low** — hygiene, dead code, misleading telemetry.

## 3. Defects

### P1 — Command 3 ignores `n_points` — **Blocker**
Section0 never copies `pc_package.n_points` into `ICV_n`; `ICV_n` keeps its initial value
**4**. The sequencer runs points 0–3 of Python's 7-point template and treats segment 2 as the
last one: the goto phase stops short of the park point, and in the pick phase the vacuum is
released halfway along P2→P3, over the belt instead of at the bin. Section0 also still copies
only indices 0–6.

### P1b — `argument_time` removed while Python writes it — **Blocker** (PC side)
`comm.omron.PLCGateway.send_package` writes `pc_package.argument_time[0..6]` in the same batch as
`bit_doing`. On the target program those tags do not exist; the batch write errors and the
gateway's retry/reconnect path runs on every dispatch. Fix on the PC
([`data-contract.md`](data-contract.md) §5).

### P2 — IK error locks trajectory motion permanently — **High**
Inside `MC_Inter_Curve_Vel`, State 99 exits only when `Out_Error_IK_OWS` is FALSE, but that
flag is cleared only in State 0 — which State 99 never reaches. Nothing outside can clear it
(the sequencer only reads it). After `ErrReset` the sequencer restarts, reads the still-TRUE
flag in State 1 and returns to Error. In addition `ICV_Err_Reset` is not driven by
`Res_Err_Btn` or by any PC command, so `Traj` stays in State 9 anyway. Result: one
unreachable waypoint (IK fails, including the −20° joint limit) disables command 3 until the
PLC is restarted. The PC does not pre-check IK ([`kinematics.md`](kinematics.md) §4).
`Out_Error_IK_OWS` is declared `VAR_OUTPUT` in both programs, so no caller can clear it.
The old declaration comment "Err_IK must reset manual" records the intent, but no rung
implements it.

### P3 — Sequencer can hang in Busy after the last segment — **High** (*verify*)
The sequencer leaves State 1 only on a rising edge of `Out_Done_Inter`. State 3 of the
interpolator raises it only if all three joints are within **0.0005°** of the last setpoint
*before* the `MC_Sync_Axis*` instances report not-Busy; when Busy drops it returns to State 0
regardless. If the servo has not settled inside 0.0005° at that instant (≈ 116 encoder
counts at this scaling), `Out_Done_Inter` is never raised, `Traj.Busy` stays TRUE forever and
every later command 3 is discarded (P4). The old program did not depend on this flag. *Verify*
how long `MC_Sync*` stays Busy after `Execute` falls.

### P4 — Command 3 while busy is silently dropped — **High**
Rung 13 lowers `ICV_Start_NewTurn` right after the call; the sequencer only accepts an edge in
State 0. A pick-phase packet that arrives while the goto phase is still running or settling is
lost, with `task_state` still showing 2 and no other feedback. The PC dispatches the pick
phase on its own positional gate and cannot tell. Needs an acknowledgement (`n_points_ack` /
a busy-reject state) and a PC-side check.

### P5 — No completion report for command 3 — **Medium**
`task_state` stays 2 for command 3 forever; `Traj.Done` / `Traj.Error` are not mapped to
`plc_package`. (Present in both versions.)

### P6 — New telemetry fields never written — **Low**
`Total_Time_Sum`, `current_step`, `n_points_ack` are declared, never assigned; Section4 still
copies only `ICV_t[0..6]`. (`ICV_t` is declared `ARRAY[0..30] OF LREAL`, so the sequencer's
zeroing of `T_Seg[0..30]` is in range.)

### P7 — Trajectory velocity discontinuities — **High** (accuracy and mechanics)
Three independent faults in the profile planner (§2.2 of [`motion-fbs.md`](motion-fbs.md)):

1. **Blend segments always end at `V_max`.** A blend segment too short to reach `V_max`
   still has its peak clamped to `V_end = V_max`; `S_acc` then exceeds L, `S` is clamped to L
   early and the setpoint **stops at B** for the rest of `T_total`; the next segment then
   starts with `V_Start_Req = V_max`. The TCP setpoint goes accelerate → stand still →
   jump to `V_max`. Python's template has short segments (25 mm corner chamfers).
2. **No backward pass.** Nothing limits a segment's exit speed to what the remaining path can
   absorb. Example with `V_max = 300`, `A = D = 1000`: entering a 12 mm final descent at
   300 mm/s, the stop distance is 45 mm.
3. **Wrong sign when braking.** In that case the planner computes a peak
   `V = sqrt((2ADL + D V_start²)/(A+D)) ≈ 239 mm/s < V_start`, then interpolates the first
   phase as `S = V_start t + A t²/2` — accelerating. `S` reaches L after ≈ 37 ms and is
   clamped: the setpoint arrives at the endpoint at > 300 mm/s and stops within one 4 ms scan.
   The servo absorbs this as following error — overshoot below the pick height or a lagging
   contact, which is exactly the uncertainty behind the pick-accuracy report §2.5.

Fault 2 exists in both versions; faults 1 and 3 are made worse in the target version by the
removal of the corner look-ahead (the old version capped corner speed at
`V_max·sqrt((1+cosθ)/2)`, e.g. 212 mm/s at 90°).

### P9 — Teaching FB defeats the servo-on interlock — **High** (safety)
* `Teach_FB` is called without `start_teaching_mode`, a `VAR_INPUT` with no initial value
  (FALSE), so teaching mode can never be entered.
* Outside teaching mode the FB sets `Pw_servo_on := TRUE` and runs its **own** `MC_Power`
  instances on all three axes every scan. Rung 0 has another `MC_Power` per axis, and rung 3's
  "reset forces servo off until Servo-ON is pressed" relies on rung 0 alone. With two
  instances the FB's always-TRUE enable wins, so the operator interlock no longer controls
  servo power. Two `MC_Power` instances on one axis is itself a configuration error to check
  in Sysmac (multi-execution of motion instructions).

### P10 — Limit stop / Abort leave the sequencer inconsistent — **Medium**
Rung 1 (home switch hit outside homing) stops the axes with `MC_Stop` and clears
`ICV_Start_NewTurn`, but does not assert `ICV_Abort`; the interpolator keeps advancing its
internal time and the sequencer stays Busy with the axes stopped. `ICV_Abort` itself is never
driven, and it only drops `Seq_Exec`: a segment already in State 2 runs to completion. There
is no working stop command from the PC (command 0 is commented out).

### P11 — State-10 soft start has no distance guard — **Medium**
Every trajectory begins with a fixed 80 ms linear joint ramp from the actual pose to
`Pos[0]`, whatever the distance. For the pick phase that is the 16 mm descent (≈ 200 mm/s
average, velocity steps at both ends). If `Pos[0]` is far from the actual pose (stale PC pose
after a failed pick) the arm is commanded to cover the whole distance in 80 ms. Reject
trajectories whose `Pos[0]` is more than a few mm from FK(`Act.Pos`), or give the ramp a
distance-dependent duration.

### P12 — Segment junction setpoint stall — **Low**
At each blend junction the interpolator spends one scan finishing State 2 (no setpoint
update), one in State 0 and one in State 1 before interpolating again: ≈ 8–12 ms without a new
setpoint while the plan assumes continuous motion (≈ 2.4–3.6 mm lag at 300 mm/s, recovered as
a velocity spike). Also, State 2 stops sampling at the last `t ≤ T_total`, so the final
setpoint of a blend segment can fall up to `V·4 ms` short of B before the next segment snaps
to IK(B). Present in both versions.

### P13 — Dead or unreported paths — **Low**
`Goto_Abs_Error` and `Home_Error` are never set (errors appear as "never done");
`Goto_Absolute` moves joints independently (curved TCP path) and would conflict with a
running `MC_Sync` trajectory if command 2 were sent mid-trajectory; commands 0, 1, 5, 6, 10
are no-ops; `argument_number` is unused.

## 3a. The deployed program `Matching_Code_10`

### Which `P*` defects it shares

| ID | In `Matching_Code_10` |
|---|---|
| P1, P1b | Absent: 7 fixed points; `argument_time` exists. Reading the non-existent `end_effector` is tolerated, because the status read accepts partial success. |
| P2 | **Present**: identical `MC_Inter_Curve_Vel` State 99. |
| P3 | Absent: State 3 leaves on `MC_Sync*.Busy` only. |
| P4 | **Different, worse**: a command 3 while busy is executed on top of the running chain (O2). |
| P5 | **Present**. |
| P6 | N/A. `Total_Time_Estimate[6]` is always 0. |
| P7 | Fault 1 absent (corner look-ahead). Faults 2–3 **present**: the pick phase's 5 mm final descent is entered at ≈ 263 mm/s and covered in ≈ 20 ms (simulated). |
| P9 | N/A (no teaching FB). |
| P10 | **Present**: rung 1 resets only `ICV_Start_NewTurn` and `ICV_Blend_Done_0/1`; instances 2–5 keep running against `MC_Stop`. |
| P11, P12, P13 | **Present**: 6 zero-velocity setpoint intervals per 7-point trajectory (simulated). |

### O1 — Joint-limit violation reported as success — **High** (safety, both programs)

In `Calc_Inverse_Kinematics`, the test
`IF Calc_Inverse_Kinematics = TRUE OR tmp_ThetaN < -20 THEN Out_Error_IK := TRUE; RETURN;`
leaves the return value at the FALSE that `Calc_Angles_YZ` just produced. `Out_Error_IK` is
a local variable.

A point that only trips the −20° limit therefore comes back as "no error", and every joint
output from the failing arm onwards is unwritten. The expected value of an unwritten IEC
function output is its default, 0.0 (bench check B1).

**Where this bites.**
- **The interpolator** streams those angles to `MC_SyncMoveAbsolute`. The simulator shows a
  jump of 4 800 °/s on axis 1.
- **`Goto_Absolute`** moves to them.

The limit binds at z ≥ −265 mm (see [`config-review.md`](config-review.md) §1). The PC
checks only the XY radius.

**Fix.**
- PLC: `Calc_Inverse_Kinematics := TRUE;` before each `RETURN`.
- PC: an IK pre-check of every waypoint.

### O2 — Command 3 on top of a running chain — **High** (safety)

**What happens.** Section0 accepts command 3 in any state:
1. The waypoint arrays are overwritten.
2. `ICV_Start_NewTurn` restarts instance 0, which takes the axes with its State-10 ramp.
3. Any instance *k* ≤ 4 that is still running finishes on the **new** arrays and pulses
   `ICV_Blend_Done_k`.
4. Instance *k+1* starts in blend mode with `Theta := IK(new Pos[k+1])` and seizes the axes.

The result is a one-scan setpoint jump between the old and the new trajectory. The simulator
shows a jump from the pick area to the bin.

**The exposure is real.** Python dispatches the next phase as soon as `pos_EE` is within
`pick_arrival_tolerance_mm` (15 mm, ramping to 50 mm at ≥ 100 mm/s) of the last waypoint.
This applies both to goto → pick and to pick → the next goto. After arrival is accepted,
instances 0–4 stay busy for:

| Tolerance | goto → pick | pick → next goto |
|---|---|---|
| 15 mm | 12 ms | 36 ms |
| 50 mm | 160 ms | 172 ms |

The pylogix write of a packet (36 per-element tag writes) plus the 50 ms status poll covers
the 15 mm window, but not the 50 mm one.

**Fix on the PC, no PLC change.** Accept arrival only once the pose is inside the final
vertical segment, i.e. below `slope_transition_height` at the target XY. From then on only
instance 5 can be busy, and it ends without a pulse.

**Fix on the PLC.** Reject command 3 while any instance is not in State 0.

### O3 — A zero-length segment stalls the chain — **Medium**

`MC_Inter_Curve_Vel` State 1 with `L = 0` returns `Execute := FALSE` without
`Out_Done_Blend`, and an exact reversal (`V_end = 0`) ends in State 3 instead of a blend.
In both cases the remaining instances never start. The arm stays at `Pos[k]`, `task_state`
stays 2, and Python's arrival wait times out.

Python's goto template produces four identical points whenever the start XY equals the pick
XY. The simulator shows the arm stopping at clearance height.

### O4 — CLI `pick` / `release` send a 1-point trajectory padded with (0, 0, 0) — **High** (PC side)

**What happens.** Segment 0 runs from the current pose towards the base origin at up to
`V_max`. On the way it trips O1 repeatedly (27 times in the simulator, including one
4 800 °/s jump) and ends in the sticky IK error (P2). Command 3 is then dead until the PLC
restarts.

**Fix.** Pad with the current pose instead. Segment 0 then has `L = 0` and stops the chain
by design (O3), with the pump holding `E[0]`. This is a working "pump in place" command on
this program.

### O5 — Pump release uses the previous trajectory's timing — **Low**

The `TP` starts when step 5 is entered, with `PT = 0.5 · Mem_ICV_t5`. `Mem_ICV_t5` is updated
from instance 5 one scan later, so `PT` belongs to the previous trajectory.

**Effect.**
- The first trajectory after power-up (`Mem_ICV_t5 = 0`) releases the vacuum at the start of
  the final 5 mm descent.
- Later picks release ≈ 0.13 s after it starts, which is after the motion has stopped
  (P7-3). The bin is reached first, so this is benign.

### O6 — An aborted goto blocks every later goto — **Medium** (both programs)

**The mechanism.**
- `Goto_Absolute` starts its three `MC_MoveAbsolute` on the rising edge of `MC_Goto_Abs`.
- Section2 clears `MC_Goto_Abs` only on `Goto_Abs.Done`.

**How it breaks.**
1. A command 3, or any other motion instruction, takes the axes while a goto is running.
2. The moves are aborted, so `Done` never comes and `MC_Goto_Abs` stays TRUE.
3. Every later command 2 sets it TRUE again without a rising edge, so it is ignored, and
   `task_state` stays 2.

Only rung 1 (a home switch) clears the flag. A command 2 sent while an earlier one is still
moving is ignored the same way: the running move keeps its first target.

The simulator reproduces this (`tests/test_plc_sim.py`). The operator console
refuses manual commands while a goto is running.

### O7 — CLI `grab` / `place` send their moves without waiting — **Medium** (PC side)

`cli.py` dispatches the four packets of `grab`/`place` back to back:
- **Gotos:** the three command 2s go out while the first is still moving, so only the
  first executes (O6).
- **Pump:** the command 5/6 has no effect on the PLC.

The sequence therefore stops at clearance height above the target and never grips.

### O9 — FK error never reported — **Low** (both programs)

`Calc_Forward_Kinematic` assigns its error flag to a local variable called
`Calc_Forward_Kinematics` (with a trailing "s"), not to the function's return value. On a
degenerate pose the outputs stay unwritten, so `pos_EE` reads (0, 0, 0) with no error flag.

### O10 — Homing completion depends on a 5 s window — **Low**

Rung 6 (calibration move + re-zero) runs only while `MC_Home_Delta.Done` — a 5 s `TP`
pulse — is TRUE. It currently needs ≈ 3.4 s. A calibration angle above ≈ 45° would never
reach `Home_Done`.

### Impact on the planned work

| Planned change | Defects it depends on |
|---|---|
| New scheduler + speed controller (sandbox KIM / `predictive_rank`) | P1, P4, P5 — the sandbox assumes two dispatches per pick that are always executed; P7 changes the per-pick time model it plans with |
| Pick-accuracy fixes (`bao-cao-do-chinh-xac-pick.md`) | P11 settles report §2.3: the descent is **commanded** in 80 ms (State 10), so `t_d ≈ 0.08 s` plus servo following lag, not the 0.31 s S-curve model; P7 is the physical cause behind report §2.5 |
| Oblique (belt-tracking) descent | The descent is not a profiled segment — it is the State-10 ramp to `Pos[0]`. An oblique descent needs the contact point to be a profiled segment (e.g. pick phase `Pos[0]` = park point, `Pos[1]` = moving contact point) and P7 fixed; otherwise the slant is executed as an 80 ms joint ramp |

## 4. Proposed ST patches

Patches are proposals for the user to apply in Sysmac Studio; none has been compiled.

**P1 — dispatcher, command 3**
```iecst
3:
    ICV_n := pc_package.n_points;
    IF ICV_n > 32 THEN ICV_n := 32; END_IF;
    FOR i1 := 0 TO 31 DO
        ICV_Pos_X[i1] := REAL_TO_LREAL(pc_package.argument_x[i1]);
        ICV_Pos_Y[i1] := REAL_TO_LREAL(pc_package.argument_y[i1]);
        ICV_Pos_Z[i1] := REAL_TO_LREAL(pc_package.argument_z[i1]);
        ICV_Pos_E[i1] := pc_package.argument_e[i1];
    END_FOR;
    IF Traj.Busy THEN
        plc_package.task_state := 4;          // 4: rejected, busy (P4)
    ELSE
        ICV_Start_NewTurn := TRUE;
        plc_package.n_points_ack := ICV_n;
        plc_package.task_state := 2;
    END_IF;
    plc_package.task_doing := 3;
    pc_package.commandID := -1;
```
(Copy the arrays only when accepted if a running trajectory must not see its waypoints
change — the sequencer reads `Pos[SegIdx+1]` at each segment load.)

**P5/P6 — completion and telemetry** (after the `Traj(...)` call)
```iecst
IF plc_package.task_doing = 3 THEN
    IF Traj.Done THEN plc_package.task_state := 1;
    ELSIF Traj.Error THEN plc_package.task_state := 3; END_IF;
END_IF;
plc_package.current_step   := Traj.SegIdx;
plc_package.Total_Time_Sum := LREAL_TO_REAL(Traj.T_Total);
// Section4: FOR i_t := 0 TO 30 DO ... END_FOR;   (ICV_t must be ARRAY[0..30] or larger)
```

**P2 — IK error recovery**: in `MC_Inter_Curve_Vel` State 99, clear the flag when `Execute`
is FALSE (`Out_Error_IK_OWS := FALSE; State := 0;`) so the next command can run; drive
`ICV_Err_Reset` from `Res_Err_Btn` and automatically on a new command 3.

**P3 — settle**: in State 3 raise `Out_Done_Inter` when all `MC_Sync*` are not Busy, with the
0.0005° test kept only as a diagnostic flag; or add a settle timeout (e.g. 200 ms) after which
the sequencer reports Done (and an "unsettled" flag).

**P7 — profile planner**:
* restore a corner speed limit (the old cosine law) instead of `V_end = V_max`;
* cap every exit speed by the stop distance of the rest of the path
  (`V_end ≤ sqrt(2·D·L_remaining)`), i.e. a backward pass over `Pos[SegIdx+1..N-1]` in the
  sequencer before starting;
* when `V < V_start`, use `−A` in the first phase (braking trapezoid).
The PC time model must be updated to the same law in the same change.

**P9 — teaching**: pass `start_teaching_mode` explicitly; remove the FB's `MC_Power`
instances and let it only request servo-off through a flag that rung 0 combines with the
operator interlock.

**P10 — stop**: implement command 0 as `ICV_Abort := TRUE` + `MC_Stop` on all axes, and assert
the same abort from rung 1.

**P11 — soft start guard**: in State 0, reject (ErrorID 3) when
`|FK(Act.Pos) − A| > 2 mm`.

## 5. Bench checks before first production run

1. *(Declarations are settled from the project files: `ICV_t` is `[0..30]`,
   `fb_PumpTimer` is `TP`, `Out_Error_IK_OWS` is `VAR_OUTPUT`, and
   `ICV_Vmax/Amax/Dmax = 300/1000/1000`.)*

   **B1**: command a point that trips only the −20° limit, e.g. X = 0, Y = −150,
   Z = −255, **with the arm clear and servo speed limited**, or read the IK outputs online
   without motion via rung 10 (`X_cal/Y_cal/Z_cal`). This confirms the unwritten-output
   value behind O1.
2. Send one 7-point command 3 with the arm in free space; confirm all 7 points are visited and
   `task_state`/`n_points_ack` behave as patched.
3. Send a second command 3 during the first; confirm it is reported rejected, not lost.
4. Command an unreachable point; confirm the error is reported and the next valid command runs.
5. Log `MC_Axis*.Act.Pos` at 4 ms (Sysmac data trace) through a pick phase: measures the real
   descent time and the following error at the end of the last segment (P7, P11).
6. Deployed program, O2: in a data trace of production at belt ≥ 100 mm/s, look for
   `MC_Sync` `CommandAborted` or following-error alarms at phase boundaries.
