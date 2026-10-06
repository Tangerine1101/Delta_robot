# PC ↔ PLC Data Contract

> Authoritative description of every value exchanged between the PC and the two PLCs.
> §1–§2 Omron (both program versions, because the Python side currently matches the old one),
> §3 Siemens, §4 command IDs, §5 what Python must change to run the target Omron program.
> Byte order and connection prerequisites: [`../basis-programming.md`](../basis-programming.md) §3.1.

## 1. Omron NX1P2 — global tags

Access is by **tag name** via `pylogix` (EtherNet/IP, CIP). `pc_package` and `plc_package`
are published ("Publish Only"). Python writes and reads **individual elements**
(`pc_package.argument_x[3]`, …), never the whole struct, so struct layout/offsets do not
matter — but **every tag name Python touches must exist**, or the write fails.

### 1.1. `pc_package : From_pc` (PC → PLC)

| Field | Old (`Matching_Code_10`) | Target (`delta_paper_0_2`) | Written by Python today |
|---|---|---|---|
| `n_points` | — | INT | **no** |
| `commandID` | INT | INT | yes |
| `argument_number` | INT | INT | yes (not read by either PLC program) |
| `argument_x/y/z` | ARRAY[0..6] OF REAL | ARRAY[0..31] OF REAL | yes, `interpolar_points` (= 7) elements |
| `argument_time` | ARRAY[0..6] OF REAL | **removed** | **yes → fails on target** |
| `argument_e` | ARRAY[0..6] OF INT | ARRAY[0..31] OF INT | yes, 0/1 |
| `bit_doing` | INT | INT | yes, `1`, written **last** |
| `conveyor_speed` | REAL (added with `Section_Conveyor`) | — | yes, command 8 only (mm/s ≥ 0) |

### 1.2. `plc_package : From_plc` (PLC → PC)

| Field | Old | Target | Written by PLC | Read by Python |
|---|---|---|---|---|
| `pos_angular` | [0..2] REAL | [0..2] REAL | actual joint angles, every scan | yes |
| `pos_EE` | [0..6] REAL | [0..6] REAL | [0..2] = FK of actual angles | yes, [0..2] |
| `task_doing` | INT | INT | last dispatched command ID | yes |
| `task_state` | INT | INT | 1 done / 2 busy / 3 error (see §2) | yes |
| `Total_Time_Estimate` | [0..6] REAL | [0..30] REAL | [0..6] = per-segment modelled time | no |
| `Total_Time_Sum` | — | REAL | **never written** | no |
| `current_step` | — | INT | **never written** | no |
| `n_points_ack` | — | INT | **never written** | no |
| `end_effector` | — | — | does not exist in either struct | **yes** — that element fails; the read still succeeds because `_response_has_success` accepts any successful element |
| `conveyor_velocity` | REAL (added with `Section_Conveyor`) | — | `-MC_Conveyor.Act.Vel`, mm/s, positive along the belt | yes, every poll |
| `conveyor_position` | REAL (added) | — | `-MC_Conveyor.Act.Pos`, mm, increasing along the belt | yes, every poll |
| `conveyor_state` | INT (added) | — | 0 stopped, 1 ramping, 2 at speed, 3 error, 4 servo off | yes, every poll |

The belt members were added at the end of both structs of the deployed program, together with
`Section_Conveyor` (axis `MC_Conveyor`, EtherCAT servo, unit mm); the `.smc2` in `OMRON/matching
code/` predates them. The target program has none of them yet.

## 2. Omron handshake semantics

1. PC writes all arguments, then `bit_doing := 1`.
2. In the next scan with `bit_doing <> 0` the PLC dispatches `commandID`, writes
   `commandID := -1`, `bit_doing := 0`. Dispatch is one scan; motion runs afterwards.
3. Progress:
   * command 2 → `task_state` 2 then 1 when `Goto_Abs.Done`;
   * command 4 → 2 then 1 on `Home_Done`;
   * command 3 → 2 and **never changes** (no rung maps sequencer completion);
   * commands 5/6 → 1 immediately (and have no physical effect, `program-main.md` rung 13);
   * command 8 (belt speed) → `task_doing` / `task_state` untouched, so a belt command never
     masks the state of a running arm command; the belt reports through `conveyor_state`.
4. Python reads `bit_doing` back only after a command 8 (≤ 0.1 s, so the next command cannot
   overwrite `commandID` before the PLC scanned it) and does not use `task_state` for command 3. It
   detects arrival from `pos_EE` (must first leave, then re-enter a tolerance around the last
   waypoint) with a deadline of `Σ argument_time + execution_margin_s`
   (`RealtimePickExecutor._wait_for_arm_arrival`).
5. **A command 3 while a trajectory is running** behaves differently in the two programs:
   * **Target program**: it is discarded without any feedback (P4). This includes the
     sequencer's settle phase.
   * **Deployed program**: it is executed on top of the running chain (O2). The PC must not
     dispatch until the arm is inside the final segment of the previous trajectory.

## 3. Siemens S7-1200 — DB contract

The Siemens program is unchanged; the PC uses it for the cup rotation only (commands 7, 9).
Its DB1 (12 bytes, PC → PLC: `CommandID`, `rotate`, `speed`) and DB2 (20 bytes, PLC → PC:
`rotate_current`, `speed_current`, `task_doing`, `task_state`, `conveyor_position`) offset
tables are kept in one place only: [`../basis-programming.md`](../basis-programming.md) §3.2.
The belt fields of both DBs are no longer used by the PC; the layout is kept.

Each send is `db_write(DB1)` followed by `db_read(DB2)`. DB1 has no handshake bit
(`open-issues.md` L5); DB2 `task_state` is unusable (L3).

## 4. Command IDs (`modules/comm/packets.py` › `COMMAND_ID`)

| ID | Name | PLC | Effect on target Omron program |
|---|---|---|---|
| 0 | stop | Omron | none (branch commented out) |
| 1 | goto_relative | Omron | none (not implemented) |
| 2 | goto_absolute | Omron | joint-space move to `argument_*[0]` |
| 3 | go_trajectory | Omron | N-point trajectory (production path) |
| 4 | calibrate | Omron | homing + calibration |
| 5 / 6 | pick / release | Omron | none (pump overwritten each scan) |
| 7 | rotate_absolute | Siemens | cup rotation |
| 8 | change_speed | Omron (deployed program) | belt speed, `pc_package.conveyor_speed` |
| 9 | plan_siemen | Siemens | rotation (the CLI `plan_siemen` sends 7 + 8) |
| 10 | enable | Omron | none |

Suction is controlled only through `argument_e` of a command 3.

## 5. Changes required on the PC before running `delta_paper_0_2`

Assuming the PLC patches in [`version-diff-and-defects.md`](version-diff-and-defects.md) §4
are applied:

| # | Change in `modules/comm/` / callers | Why |
|---|---|---|
| 1 | Drop `argument_time` from `ARRAY_FIELDS` / `_zero_package` for the Omron (keep the PC-side times only for the arrival deadline and logs) | tag no longer exists |
| 2 | Write `pc_package.n_points` = number of meaningful points (≥ 2) | sequencer length |
| 3 | Stop padding with `(0, 0, 0)` beyond `n_points` being executed; for the CLI `pick`/`release` (currently a 1-point command 3) send a 2-point hold trajectory or drop the feature | `N_Points < 2` is rejected; a padded point at Z = 0 is an IK error |
| 4 | Read `n_points_ack`, `current_step`, `task_state` (once patched) and only dispatch the pick phase after the goto phase has been **accepted and completed** by the PLC | otherwise the pick phase can be silently discarded (**P4**) |
| 5 | Remove `end_effector` from the status tags (or add it to `From_plc`) | read of a non-existent member |
| 6 | Re-derive the PC trajectory-time model (`core/motion`: `corner_v_end`, `segment_profile_time`, `trajectory_time`) from the PLC as patched — the target FB has no cosine corner law | gate lead / ETAs |

On the PLC side the target program also needs the belt section of the deployed one
(`open-issues.md` **P14**); the PC side of the belt contract is unchanged by the switch.
