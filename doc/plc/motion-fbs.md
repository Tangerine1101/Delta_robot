# Omron Motion Function Blocks — `delta_paper_0_2`

> **Source.** The ST bodies and declarations in `OMRON/delta_paper_0_2/delta_paper_0_2.smc2`.
> Defect IDs (`P*`) refer to [`version-diff-and-defects.md`](version-diff-and-defects.md).
>
> **Deployed program.** Its `MC_Inter_Curve_Vel` differs in the points listed in
> [`program-old.md`](program-old.md) §4.

## 1. `FB_ICV_Sequencer` — N-point trajectory runner

Runs an N-point polyline (`N_Points` ≥ 2, ≤ `Max_Index + 1` = 32) through **one**
`MC_Inter_Curve_Vel` instance, one segment at a time. Segment *i* is `Pos[i] → Pos[i+1]`,
*i* = 0 … N−2. It does not touch the pump; the caller uses `SegIdx`, `IsLastSeg`, `T_SegNow`.

**Interface**

| Kind | Name | Meaning |
|---|---|---|
| IN | `Execute` | rising edge starts a trajectory (ignored unless idle) |
| IN | `Abort`, `ErrReset` | abort to idle; clear error |
| IN | `N_Points`, `Max_Index` | point count; highest index of the position arrays |
| IN | `V_max`, `A_max`, `D_max` | interpolator limits (mm/s, mm/s²) |
| IN_OUT | `Pos_X/Y/Z : ARRAY[*] OF LREAL`, `T_Seg : ARRAY[*] OF LREAL` | waypoints; per-segment durations written back |
| IN_OUT | `Int_Axis1..3` | axis references (must be In/Out at every nesting level) |
| OUT | `Busy`, `Done` (1-scan pulse), `Error`, `ErrorID` (1 = bad `N_Points`, 2 = IK) | status |
| OUT | `SegIdx`, `IsLastSeg`, `T_Total`, `T_SegNow` | progress for the caller |

**Per-scan order** (fixed, the code depends on it): (1) clear `Done` → (2) call the
interpolator → (3) edge-detect `Out_Done_Blend`, `Out_Done_Inter`, `Execute` →
(4) `Abort` → (5) state machine → (6) load the next segment → (7) outputs.
Because (2) runs before (5)/(6), the next segment is loaded in the same scan the previous one
reports done.

**States**

| State | Behaviour |
|---|---|
| 0 idle | On `Execute` edge: reject `N_Points < 2` or `N_Points − 1 > Max_Index` (→ 9, ErrorID 1); else latch `Last_Idx := N_Points − 2`, zero `T_Seg[0..Max_Index−1]`, `SegIdx := 0`, `V_start := 0`, request load, → 1 |
| 1 running | IK error → 9 (ErrorID 2). `Out_Done_Blend` edge → store `T_Seg[SegIdx]`, `V_start := Out_V_End`, `SegIdx += 1`, load next. `Out_Done_Inter` edge (last segment settled) → store time, → 2 |
| 2 done | `Done` pulse, → 0 |
| 9 error | wait for `ErrReset` → 0 |

Loading (6) sets A = `Pos[SegIdx]`, B = `Pos[SegIdx+1]`, `Blend := SegIdx < Last_Idx`,
`Seq_Exec := TRUE`. `Abort` only drops `Seq_Exec`; a segment already in State 2 of the
interpolator finishes (**P10**). Leaving State 1 needs the interpolator's settle flag
(**P3**); an IK error is sticky inside the interpolator (**P2**).

## 2. `MC_Inter_Curve_Vel` — one-segment Cartesian interpolator

Moves the TCP along the straight line A → B, computing IK every scan and streaming joint
setpoints through `MC_SyncMoveAbsolute`-style instances `MC_Sync_Axis1..3`
(`Execute := Enable_Motion`, `Position := ThetaN`), called at the end of every scan.

### 2.1. State machine

| State | Behaviour |
|---|---|
| 0 | `Execute` TRUE: clear flags, IK(A) → error → 99. Otherwise `Out_T_Segment := 0`; if `V_Start_Req > 0` (blend continuation) set `Theta := IK(A)` → 1; else latch `Start_Theta := Act.Pos` → 10. `Execute` FALSE: `Out_Done_Blend := FALSE`, `Enable_Motion := FALSE`, `Theta := Act.Pos` |
| 10 | **Soft start**: 20 scans, `Theta := Start + (IK(A) − Start)·k/20` — a linear **joint-space** ramp from the actual pose to point A over 80 ms, whatever the distance → 1 |
| 1 | Profile planning (§2.2), `Out_T_Segment := T_total`, `t := 0` → 2 |
| 2 | While `t ≤ T_total`: S(t) from the profile, P(t) = A + (S/L)(B − A), IK(P) → `Theta` (IK error → 99), `t += 0.004`. At the end: blend with `Out_V_End > 0` → `Out_Done_Blend := TRUE`, `Execute := FALSE`, → 0; else → 3 |
| 3 | Settle: `Enable_Motion := FALSE`; `Out_Done_Inter := |Theta − Act.Pos| ≤ 0.0005°` on all axes; when all `MC_Sync_Axis*.Busy` are FALSE → `Execute := FALSE`, → 0 |
| 99 | `Execute := FALSE`, `Enable_Motion := FALSE`; leaves only when `Out_Error_IK_OWS` is FALSE (**P2**) |

Point A of every new trajectory is therefore reached by the State-10 ramp, not by the
profile. In the pick phase Python puts the **contact point** in `Pos[0]`, so the whole
pre-pick → contact descent is commanded as this 80 ms linear joint ramp (velocity step at
both ends), followed by the planned motion from the contact point onwards.

### 2.2. Profile planning (State 1)

`L = |B − A|`. Two profile families:

**(a) Rest-to-rest polynomial S-curve** — last segment (`Blend = FALSE`) with
`V_Start_Req = 0`:

* Peak speed `V = V_max`, reduced to `V = sqrt(L / (0.75 (1/A + 1/D)))` when
  `L < 0.75 V² (1/A + 1/D)`.
* `t_acc = 1.5 V/A`, `t_dec = 1.5 V/D`, `S_acc = V t_acc/2`, `S_dec = V t_dec/2`,
  `t_run = (L − S_acc − S_dec)/V`.
* Acceleration: `S(t) = V t_acc (τ³ − τ⁴/2)`, τ = t/t_acc (zero velocity and acceleration
  at both ends of the phase; peak acceleration 1.5·V/t_acc·… = `A_max`).
* Deceleration: `S = S_acc + S_run + V t_dec (τ − τ³ + τ⁴/2)`.

The factor 1.5 is what makes the polynomial's peak acceleration equal `A_max`.

**(b) Trapezoid** — every other case (any segment with non-zero start or end speed):

* End speed `V_end`: 0 for the last segment; **`V_max` for every blend segment** (no corner
  look-ahead).
* Peak `V = V_max`; if `S_limit = |V² − V_start²|/2A + |V² − V_end²|/2D > L`, then
  `V = sqrt((2ADL + D V_start² + A V_end²)/(A + D))`; for blends V is clamped to
  ≥ `V_start` and ≥ `V_end`.
* `t_acc = |V − V_start|/A`, `t_dec = |V − V_end|/D`, `S_run = max(0, L − S_acc − S_dec)`.
* Interpolation: `S = V_start t + A t²/2` (acc), linear (run), `S_acc + S_run + V t' − D t'²/2`
  (dec), then **`S := min(S, L)`**.

The trapezoid always uses `+A` in the first phase, even when `V < V_start` (a braking
segment) — **P7**. There is no backward pass: nothing limits a blend segment's exit speed so
that the following segments can stop in time.

`Out_T_Segment = T_total = t_acc + t_run + t_dec`. It excludes the 80 ms soft start and
the settle time.

## 3. `Goto_Absolute` — single-point joint move

IK(`X_0`, `Y_0`, `Z_neg300`) → three independent `MC_MoveAbsolute` (15 °/s, 30 °/s²) on the
rising edge of `Goto_Abs`; `Done` is set when all three are done and reset on the falling edge
of `Goto_Abs`. The TCP path is **not** a straight line (each joint moves on its own profile).
IK failure is not reported (`Goto_Abs_Error` is never written). Used for manual positioning
only; production uses command 3.

## 4. `MC_Home_Delta` — homing

On the rising edge of `Start_Home_Ext`: set `Start_Home_Flag`, clear `Home_Done_Flag1..3`.
Each axis runs `MC_MoveVelocity` at 12 °/s (24 °/s²) towards its switch (axes 1, 2 negative;
axis 3 positive) while its home switch is not active; on the switch's rising edge `MC_Stop`
then `MC_Home` (current position := 0) and `Home_Done_FlagN` is set. When all three are set,
a 5 s `TP` pulse drives `Done` and resets `Start_Home_Flag`. `Program0` rung 6 then moves to
the calibration angles and re-zeros there.

## 5. `FB_TeachingMode` — hand-guided teaching

While `start_teaching_mode` is TRUE: holding the teach button disables servo power (arm can be
moved by hand, brake released); on release the servo is re-enabled and the FB reads the three
`Act.Pos`, runs FK and pulses `teaching_done` (or `teaching_err` on FK failure) with
`pos_teaching1..3` / `theta_teaching1..3`. Outside teaching mode it forces `Pw_servo_on :=
TRUE`. It calls its own `MC_Power` instances on the three axes every scan — **P9**.
