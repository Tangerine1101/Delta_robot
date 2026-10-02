# Open Issues — Delta Robot

> **This file is the single register of everything still unresolved in this repository.**
> If a problem is real and not yet fixed, it is here — not scattered through prose in other
> documents. If it is not here, it is either done or it never existed.
>
> **AI assistants may read *and* update this file.** When you close an item, delete the row
> and record the resolution in [`decision-log.md`](decision-log.md); do not leave `[FIXED]`
> entries behind — that is exactly the changelog-rot this file replaces.
> Historical/superseded material belongs in `decision-log.md`, never here.

**Roadmap tags** (the project's declared phases):

| Tag | Phase |
|---|---|
| **NEAR** | Redesign of the pick scheduler and the conveyor speed controller |
| **MID** | Rework of the packet layout and the PLC ↔ IPC communication method |
| **FAR** | Vision-model fine-tuning, configuration slimming |
| **DOC** | Documentation / repository hygiene |

Code references name the module and function (2026-09-25 layout); treat them as hints, not
addresses.

---

## A. Pending hardware calibration

| ID | Issue | Where | Action to close | Tag |
|---|---|---|---|---|
| **C1** | `robot.rotation.sign = 1` has never been verified against the physical axis. If wrong, every rotation turns the wrong way. | `config.yaml` › `robot.rotation.sign` | `python3 -m modules.tools.test_rotate` — a commanded +90° must turn the cup CCW seen from above; otherwise set `-1`. | — |
| **C3** | `oblique_descent_enabled = false`. The belt-tracking slanted descent is implemented but disabled: an earlier simulation (E6: −65 mm error, 9% hit rate) was run with the biased descent model (T2, now fixed) and has not been repeated. | `config.yaml` › `pick_gate.oblique_descent_enabled` | Close C5 and T3, re-run E6 on the PLC simulator, then re-test on hardware. | NEAR |
| **C4** | `robot.rotation.offset_deg = -62.0` and `object_types.<type>.heading_offset_deg` (both `0.0`) are unconfirmed; they must be re-checked after any change to the suction-cup marker or mounting. | `config.yaml` | Hardware run reading the per-pick `[ROTATE]` log (`vision_angle / board_heading / rotate_cmd / rotate_at_gate / rotate_at_end`). | — |
| **C5** | **`robot.interpolator.{v_max, a_max, d_max}` have never been validated against the real arm.** This block is the trajectory-time model — i.e. the scheduler's estimate of processing time per job. Every pick-time prediction, gate lead, descent-time model and feasibility test is built on it. **First hardware evidence (2026-10-02, 45 trajectories from the pose stream):** send → arrival is 1.040–1.071 × the model for gotos (median 1.028 s vs 0.974 s; the 0.054 s gap ≈ the 0.046 s command write the model leaves out) and 1.012–1.027 × for pick phases. | `config.yaml` › `robot.interpolator`, `basis-theory.md` §3 | `speed_tuning` in the CLI runs a tilted heptagon and reports measured-vs-modelled ratio plus a first-order `v_max` suggestion (report-only, never writes config). Then apply by hand. | **NEAR — blocking** |
| **C7** | `scheduling.arm_cycle.occupancy_worst_s = 2.779` / `grab_worst_s = 1.551` and `scheduling.setup_time_s = 0.064` are measured (135 hardware pick cycles of 2026-10-02, `modules/tools/pick_timing`), but only over the picks those runs made (u = 188–359 mm). The model sweep (`python3 -m modules.tools.derive_rank_bounds`) puts the worst case at the far corner u = 390, v = 0: 1.909 / 3.458 s, i.e. v_max 35 instead of 43.9 mm/s. `queue_depth_k = 1` still makes the queue bound, not the speed band, the ceiling of `predictive_rank`. | `config.yaml` › `scheduling.arm_cycle`, `speed.laws.predictive_rank` | Re-measure after runs that pick near u_max; decide k (0 gives L / g ≈ 122 mm/s, then capped at the band). | NEAR |
| **C8** | The belt scale is now the Omron axis unit conversion of `MC_Conveyor`: *work travel distance per motor rotation* $L$ (mm/rev), to be measured by moving the belt a known number of motor turns and taping the distance. Whether $L$ has been measured and entered in Sysmac is unrecorded, so `conveyor_position` / `conveyor_velocity` are only as true as that number. Belt slip on the pulley is invisible to the servo encoder. `conveyor.position_scale_mm` stays 1.0 (a second correction on top of $L$ would double-count). Every tracked object's position is dead-reckoned from this feedback. **Resolved mismatch (2026-10-02):** `vision.pixels_per_mm` 5.7256 → 6.1 brought the camera / encoder travel ratio from 1.070 to 1.005 (106 tracks); what remains open is the absolute check by tape. Before that change: with `vision.pixels_per_mm = 5.7256` the camera sees parts travel 1.070 × the encoder's displacement (sd 0.008, 24 tracks, `modules/tools/pick_timing`). If the camera is right, every pick lands ≈ 0.07 × (u_pick − camera exit u) ≈ 7 mm late at u_pick = 188, the largest term of the along-belt pick error measured that day. | Sysmac › `MC_Conveyor` unit conversion; `config.yaml` › `conveyor.position_scale_mm` | Mark the belt, run `setspeed 50`, and check 500 mm takes 10 s; or tape ≥ 500 mm of travel against the change of `conveyor_position_raw` in `status.csv`; repeat at two speeds. A mismatch is corrected in $L$, not in the config. | NEAR |
| **C9** | `vision.latency_offset_s = 0`: the capture latency beyond the uvc start-of-exposure stamp has never been measured on the cell. A late stamp shifts every tracked part upstream by v·Δ, so picks land late in proportion to belt speed. | `config.yaml` › `vision.latency_offset_s` | `python3 main.py --scheduler --scenario test_camera_latency --set speed.static_mm_s=12` with 3–5 boards at the upstream edge of the camera window (repeat a few times); set the suggested value. | NEAR |
| **C10** | `pick_gate.gate_offset_mm = 10` makes every pick fire 10 mm early. It is a hand-set correction of the ≈ 10–15 mm late picks of 2026-10-02, not a measurement: the probable cause is the belt scale (C8), whose error grows with how far a part is dead-reckoned, so one fixed offset is only right near u_pick ≈ 188 mm. | `config.yaml` › `pick_gate.gate_offset_mm` | Close C8 and C9, then re-measure the along-belt error and bring the offset back toward 0. | NEAR |
| **C6** | The physical-calibration stage of the config checker is a documented stub: `conveyor.frame` (θ, origin), `pick_gate.robot_movement_delay_s`, and `conveyor.position_scale_mm` have no automated procedure. | `calibrate_everything.py` › `_todo_physical_calibration()` | Implement, driven by `test_vision_only` and the stationary-belt `[GATE]` measurement. | FAR |

---

## B. Configuration inconsistencies

| ID | Issue | Where | Tag |
|---|---|---|---|
| **G1** | `speed.static_mm_s = 10` lies **outside** the band `speed.band = [30, 100]` every adaptive law is clamped to. Startup seeds the belt there, so the first adaptive commit always steps it into the band (under `constant` the belt simply runs at 10). Decide whether the static seed is meant to be exempt from the band or whether one of the values is wrong. | `config.yaml` › `speed` | NEAR |
| **G2** | `scheduling.arm_cycle.cycle_s = 3.0` implies ~20 picks/min, while `doc/proposal/abstract-en.tex` claims a nominal **30–60 picks/min**. One of the two is wrong, and the discrepancy will be visible to reviewers. Note `cycle_s` is the *calibrated arm cycle* $t_\text{pick}$ feeding $\mu_\max = 1/t_\text{pick}$, so this also shifts the whole rate target $\lambda_\text{nom}$. | `config.yaml` vs the abstract | **NEAR — publication-blocking** |
| **G3** | `speed.laws.inverse_density.density_length_mm = 0.0` makes the density region derive to $L_\text{meas} = u_\max = 363$ mm, which starts upstream of the workspace ($u_\min = 188$). Density is therefore measured over a longer region than the one being regulated. This may be intentional (the camera previews parts before they arrive) but it is undocumented and it changes the meaning of $\rho$. | `config.yaml`, `basis-theory.md` §6.1–6.2 | NEAR |
| **G4** | `robot.rotation.home_tolerance_deg = 0.0` keeps the "axis not yet home at grip" check strict, whereas the documented intent of a positive value is a warn-only degradation. Confirm which behaviour production wants. | `config.yaml`, `basis-theory.md` §5.2 | — |
| **G6** | `pick_gate.arrival_tolerance_mm / _max_mm` were widened to 15/50 mm to make picks land; that is a symptom treatment. The gate latency was calibrated on 2026-10-02 and the belt scale (C8) is still open, so the band cannot be tightened back until picks are re-measured. The wide band also opens the O2 hazard window (§C.1c) on the deployed PLC. | `config.yaml`, `basis-theory.md` §4.4 | NEAR |
| **G7** | `robot.heights.clearance` (−265) lies where the −20° joint limit binds (z ≥ −265). The worst-heading XY reach there is 140 mm, not `robot.limits.radius_xy_mm = 180`, and a workspace corner already sits at θ1 = −18°. A clearance waypoint outside that envelope trips O1 silently (the realtime planner rejects such plans; see O1). | `config.yaml` › `robot.heights.clearance`, `robot.limits.radius_xy_mm`; [`plc/config-review.md`](plc/config-review.md) | NEAR |
| **G8** | The PC trajectory-time model (`core/motion.trajectory_time`) books the full planned time of the last segment, which the PLC plans but covers in ≈ 20 ms (P7 fault 3). Motion ends 0.15–0.18 s before the model; the chain stays busy ≈ 0.07 s after it. The sandbox's plant uses the model's times, so it does not reproduce this either. | `modules/core/motion.py`; [`plc/config-review.md`](plc/config-review.md) §3 | NEAR |
| **G10** | `scheduling.planner` and `speed.laws.predictive_rank.planner` are `drop_longest`: the algorithm the cell ran as `kim` until 2026-09 (add jobs in belt order, drop the longest while late — Moore-Hodgson style). The name `kim` now means Kise-Ibaraki-Mine's single-removal algorithm, as in the research repo. The config keeps the old behaviour until someone chooses. | `config.yaml` › `scheduling.planner`; compare with `sandbox/experiments/planners_under_load.yaml` | NEAR |

---

## C. Architectural and algorithmic limitations

These are real properties of the system as built. They are **not** bugs to be quietly
deleted from the docs — several of them constrain what the new scheduling algorithm can
assume.

### C.1 PLC / firmware constraints

| ID | Limitation | Consequence | Tag |
|---|---|---|---|
| **L1** | The Omron firmware **ignores `argument_time`**; the arm always runs at the interpolator's fixed limits. | The PC cannot modulate how fast a pick executes. Processing time $p_j$ is an *observable*, not a decision variable — any scheduling formulation that assumes controllable $p_j$ is invalid on this hardware. | MID |
| **L2** | `goto` and `pick` must be dispatched as two separate phases, because the PLC exposes no trajectory-time planner. | Pick timing is controlled by *when* the second phase is sent, not by the trajectory itself. This is what makes the positional gate necessary. | MID |
| **L3** | `task_state` (DB2 offset 12) reports inconsistent/legacy values; the real handshake is Omron's `bit_doing`. | Half of a status word is unusable. Candidate for removal in the packet rework. | MID |
| **L4** | `goto_relative` (command ID 1) is not implemented on the Omron program. | The ID is reserved but dead. | MID |
| **L5** | **Siemens DB1 carries no handshake bit.** It is unverified whether the ST program edge-triggers on a `CommandID` change. The belt speed now goes to the Omron, so nothing is written to DB1 between the two rotations of a pick (home, then post-grip — both `CommandID = 7`). | If the program edge-triggers, every post-grip rotation is silently dropped. Run the retrigger test of `python3 -m modules.tools.test_rotate`, or check in TIA Portal, and record the finding in `doc/plc/data-contract.md`. | **MID — highest risk** |

| **L13** | The UDP pose stream's destination (`192.168.250.101:9001`) is hard-coded in `Program_UDP`; the PC cannot choose it. | A PC with any other IP silently gets no stream and falls back to EtherNet/IP polling (logged once as `[WARN] no UDP pose datagram in 2 s`). Changing the PC means editing the PLC program, or switching it to the subnet broadcast. | MID |
| **L14** | While the UDP stream is fresh, the Omron status block (`task_state`, `task_doing`, `bit_doing`, `end_effector`) is served from a cache up to `pose_stream.omron_status_period_s` (0.2 s) old. | A consumer that waits for a handshake transition — the console's `arm_idle` (goto still running) — can see the pre-command value for up to one period. The console's 0.5 s post-dispatch busy window covers it. Lower the period or set it to 0 if a transition is missed. | MID |
| **L15** | `pos_EE_t` is the PC receive time of the UDP sample, not the PLC's sampling instant; the PLC-side delay between reading the axes and sending (≤ one 12 ms send period plus the socket call) is unmeasured. | Pose-vs-command timing from `pose.csv` carries that unknown, constant-ish offset. Measure by moving the arm and cross-correlating `pose.csv` with `status.csv` EtherNet/IP reads, or timestamp the sample in the PLC payload. | NEAR |

### C.1b Target Omron program `delta_paper_0_2` — defects before deployment

Full analysis and proposed ST patches: [`plc/version-diff-and-defects.md`](plc/version-diff-and-defects.md).
Blockers must be fixed before the target program is run with the current Python.

| ID | Defect | Consequence | Tag |
|---|---|---|---|
| **P1** | Command 3 never copies `pc_package.n_points` into `ICV_n` (init 4); copies only indices 0–6. | Only 4 of 7 points run; pick-phase vacuum released over the belt. | **MID — blocker** |
| **P1b** | `argument_time` removed from `From_pc`, but `comm.omron.PLCGateway.send_package` still writes it; `n_points` never written; `end_effector` read from a member that does not exist. | Every dispatch fails / reconnects. PC-side change list: `plc/data-contract.md` §5. | **MID — blocker** |
| **P2** | IK error in `MC_Inter_Curve_Vel` is sticky (State 99 never exits) and `ICV_Err_Reset` is not driven. | One unreachable waypoint disables trajectories until PLC restart. | MID |
| **P3** | Sequencer leaves Busy only on the 0.0005° settle flag. | Possible permanent Busy → all later trajectories dropped. Verify on bench. | MID |
| **P4** | Command 3 received while the sequencer is busy is discarded without feedback. | Pick phase silently lost if sent before goto settles. | MID |
| **P5/P6** | No completion report for command 3; new telemetry fields never written. | PC cannot know a trajectory finished or was accepted. | MID |
| **P7** | Blend exit speed forced to `V_max`, no backward pass, braking phase uses `+A`. | Setpoint velocity jumps; the final segment arrives at >300 mm/s and stops in one scan. Also invalidates the PC time model. | **NEAR/MID — accuracy** |
| **P9** | `FB_TeachingMode` not given `start_teaching_mode`; owns a second `MC_Power` per axis forcing servo on. | Teaching unusable; operator servo-on interlock defeated (safety). | MID |
| **P10** | Home-switch stop does not abort the sequencer; `ICV_Abort` undriven; command 0 commented out. | No working stop from the PC; inconsistent state after a limit stop. | MID |
| **P14** | The target program has no belt section: `Section_Conveyor`, command 8 and the `conveyor_speed` / `conveyor_velocity` / `conveyor_position` / `conveyor_state` members exist only in the deployed program. | Deploying `delta_paper_0_2` as is leaves the belt without a speed command or feedback (every status read of the belt members fails). Port the section and the struct members first. | **MID — blocker** |
| **P11** | State-10 soft start: fixed 80 ms joint ramp to `Pos[0]` with no distance guard. | Pick-phase descent is commanded in 80 ms (settles the descent-time question of the 2026-08-28 pick-accuracy report, §2.3, for the command; servo lag unmeasured); a stale `Pos[0]` is a violent move. | NEAR |

### C.1c Deployed Omron program `Matching_Code_10` — defects

The deployed program also has P2, P5, P7 (faults 2–3), P10, P11, P12 and P13; see
[`plc/version-diff-and-defects.md`](plc/version-diff-and-defects.md) §3a. Its own defects:

| ID | Defect | Consequence | Tag |
|---|---|---|---|
| **O1** | `Calc_Inverse_Kinematics` returns "no error" when only the −20° joint limit trips, and leaves the remaining joint outputs unwritten (both program versions). | The interpolator and `Goto_Absolute` command those axes to the unwritten value (0° expected): a setpoint jump. The realtime planner rejects any plan with a waypoint failing the PLC-faithful IK port (`core/kinematics.ik_reachable`); CLI presets, `grab`/`place` and the console goto are **not** pre-checked. | **MID — safety** |
| **O2** | Command 3 is accepted while the six-instance chain is running: the arrays are overwritten, instance 0 restarts, and a still-running instance *k* ≤ 4 later hands the axes to instance *k+1* on the **new** geometry. | One-scan setpoint jump between trajectories (simulated). Mitigated on the PC for the realtime loop: arrival is accepted only inside the final vertical segment (`runtime/pick_gate.in_final_segment`). The PLC itself still accepts the command; other senders (CLI, console) are not guarded. | **MID — safety** |
| **O3** | A zero-length segment (or an exact reversal) ends the chain without `Blend_Done` or an error. | Arm parks mid-trajectory, `task_state` stays 2, Python times out. Triggered by a goto whose start XY equals the pick XY. | MID |
| **O5** | Pump-release `TP` uses the previous trajectory's last-segment time. | First pick after power-up releases at the start of the final 5 mm descent. | — |
| **O6** | A goto (command 2) whose moves are aborted by another motion command never reaches `Done`, so `MC_Goto_Abs` stays TRUE (both versions). | Every later command 2 is ignored (no rising edge) and `task_state` stays 2 until a home switch trips; a goto sent while another is moving is ignored too. | MID |
| **O7** | CLI `grab` / `place` dispatch their three gotos and the pump command back to back. | Only the first goto runs (O6) and commands 5/6 do nothing: the sequence never grips. | — |
| **O9** | FK error flag assigned to a misnamed local, never returned (both versions). | On FK failure `pos_EE` reads the unwritten value (0, 0, 0) with no error. | — |

### C.1d Pick-gate timing defects — investigated 2026-08-28, re-checked on the PLC simulator 2026-09-16

Full evidence, formulas and fix roadmap: [`pick-accuracy-findings.md`](pick-accuracy-findings.md).
T2 and T6–T9 are fixed (see `decision-log.md`); the rows below are what remains. The
tests `tests/test_runtime.py` and `tests/test_pick_gate.py` assert the fixed behaviour. The PLC-simulator
runs cited below use `main.py --scheduler --scenario production --sim` with a static belt.

| ID | Defect | Consequence | Tag |
|---|---|---|---|
| **T3** | The pick descent does not reach `pickup_height`: over 25 hardware picks (2026-10-02, pose stream) the lowest z was −298.1 mm against a commanded −300 (1.9 mm short), reached 0.224 ± 0.006 s after the command-3 send. Servo lag against the PLC's 80 ms State-10 ramp (P11) is the likely cause; in the PLC simulator a first-order lag of τ = 50 ms keeps the cup from reaching `pickup_height` at all. | Shallow grips on thick or warped boards may be a descent-depth problem, not a timing one. The timing half of this item is calibrated (`decision-log.md`). | **NEAR** — decide whether to command the pickup lower by the measured shortfall, or patch P11 |
| **T4** | The realtime loop only plans while idle; above ≈120 mm/s an object must be committed to before it reaches the workspace. A plan also needs the arm parked a full gate lead before the intercept (≈ 0.22 s), and the gate aborts when fired more than `pick_gate.late_abort_mm` late. | A feasibility cliff, not a timing error — explains why 120 mm/s is a hard wall. The sequence planners look ahead over the whole queue but still commit one pick at a time from idle, so the cliff remains; a speed ceiling (predictive-rank's $v_\max$) is the available mitigation. | NEAR |
| **T5** | `core/motion.trajectory_time` chains segment speeds forward only (no backward pass); the deployed PLC's own `MC_Inter_Curve_Vel` has the same gap (`plc/version-diff-and-defects.md` **P7**). | A physically correct two-pass model would be *longer*, while the deployed PLC already ends motion 0.15–0.18 s *before* the forward-only model (G8). Do **not** port the research repo's backward pass until P7 is patched on the PLC; until then the forward model is the conservative estimate. | NEAR — blocked on P7 |

### C.2 Control / measurement gaps

| ID | Limitation | Consequence | Tag |
|---|---|---|---|
| **L6** | **No suction verification.** A pick is booked as successful when the arm's *motion* completes; the vacuum is never checked, and a dispatched grip is never retried. | The controller's own success count over-states reality. Any $\sum U_j$ reported from software is a lower bound on true losses unless picks are also counted by an independent means (bin count, downstream camera). | NEAR |
| **L9** | The accelerating-belt forms of the gate offset are derived but deliberately **not implemented**; the single-term $v \cdot T_\text{delay}$ is only correct while the belt is steady, which the speed controller guarantees by taking no decision at all from goto dispatch to cup contact (`RealtimeState.pick_committed`), whatever the law or its gate. A commit issued while the arm is free may still be ramping when the next gate fires (≤ `speed.commit.max_step_mm_s / conveyor.accel_mm_s2` = 20 / 500 = 0.04 s with the belt servo's ramp): the plan's forecast includes the ramp, the gate offset does not. | A gate that opens right before a goto (every gate does, at `arm_free`) leaves this residual; the ramp-aware offset is the fix if it matters. `tests/test_runtime.py` pins the freeze. | NEAR |
| **L10** | **The default speed law is open-loop with respect to misses.** The configured law is `constant`; `inverse_density` reacts to density and spacing only. | The outer loop described in `doc/proposal/abstract-en.tex` — predicting how many parts will miss their deadline and adjusting belt speed from that prediction — exists as the opt-in `predictive_rank` speed law (`basis-theory.md` §7.3), but it has only run on the PLC simulator and the sandbox; its band is currently pinned by C7. | **NEAR — core** |
| **L11** | The `inverse_density` spacing cap inspects only the leading `speed.laws.inverse_density.spacing_lead_objects` (default 4) parts. | A cluster further upstream is invisible to the speed law until it reaches the front. | NEAR |
| **L12** | The operator console (`main.py --interface`) binds `0.0.0.0` with **no authentication**: anyone on the cell's LAN can move the arm, run the belt or start a scenario. Accepted by the user for the lab network (2026-09-16). | Do not expose the PC beyond the cell's network. Revisit if the cell moves to a shared network (a token on `POST /api/*` is the minimal fix). | MID |

### C.3 Experiment / reproducibility gaps

| ID | Limitation | Consequence | Tag |
|---|---|---|---|
| **L16** | **The sandbox has not been validated against a reference.** `sandbox/` runs the robot's plugins on the robot's delta model, so its absolute numbers differ from the research repo's published ones (backward pass, motion parameters, per-segment formula). The paper's headline comparison has not been re-run in the sandbox to record which conclusions carry over, and the sandbox's plant finishes motion at the model's time (G8) and ignores camera latency. | Treat sandbox results as rankings and trends until that comparison is recorded in `decision-log.md`; confirm a finding on the PLC simulator before hardware. | NEAR |
| **L17** | **The part record's grip judgement is geometric, and virtual parts are idealised.** `runtime/outcomes.py` books `picked` / `miss_grip` by comparing the part's *tracked* centre with the cup at the first pose sample inside the contact band, against `runtime.grip_tolerance_mm` (12.7 mm, taken from the simulator and the paper, never measured on this cup and these boards). For real parts the tracked centre carries the tracking and latency error (C8, T3); the pose and belt samples are up to one perception period (25 ms) apart. `simulate_feeder` parts never overlap-check, never slip and are seen with no detection latency or noise unless `feeder.detection_latency_s` / `position_noise_mm` are set. | Real-part grip counts are an estimate until checked against a bin count (L6); virtual-part runs measure scheduling, belt and arm timing on the real cell, not suction. Report them as such. Measuring the real capture tolerance (off-centre picks at known offsets) closes the first half. | NEAR |

---

## D. Repository hygiene

| ID | Issue | Status |
|---|---|---|
| **D8** | `doc/proposal/` is untracked. LaTeX build artefacts are already ignored, so committing `abstract-en.tex` is safe — but it names the authors, and author names were deliberately purged from git history once already (see `decision-log.md`). Decide before committing. | **Open — user's call** |
| **D10** | The belt section of the deployed Omron program (`Section_Conveyor`, command 8, the new `pc_package` / `plc_package` members, the `MC_Conveyor` axis settings) is not in the repository: `OMRON/matching code/Matching_Code_10.smc2` predates it, and its only description is the untracked Vietnamese note `conveyor_velocity_control.md` at the repo root. | **Open — export the Sysmac project and update `doc/plc/program-old.md`** |
| **D9** | The purged git history has still not been pushed to GitHub (no credentials available in the agent environment). Until it is, the remote — if any — carries the pre-purge history. | **Open — user action** |

---

## How to use this file

* **Before proposing work**, read §A–§C: several plausible-looking improvements are already
  blocked on a calibration or a hardware fact listed here.
* **Before trusting a number** in `modules/config.yaml`, check §A and §B — a value being
  present in the config does not mean it was measured.
* **Before designing a planner or a speed law**, read L1, L6, L9, L10, L16 and §C.1c together:
  they define what the hardware permits, what can be measured, and which PLC behaviours the
  simulator (`plc/simulator.md`) and the sandbox reproduce.
