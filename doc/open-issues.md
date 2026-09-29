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
| **C2** | `pick_gate.robot_movement_delay_s = 0.17` and `pick_gate.ethernet_delay_s = 0.016` are estimates. Their sum (plus `pick_descent_time_s`, default 0) *is* the pick gate's lead offset (`basis-theory.md` §4.4), so an error here lands every pick off-centre by `v_belt · Δ`. The `[GATE]` log bias (T2) is fixed, so this is now measurable; `sandbox/experiments/model_error.yaml` shows the cost of a wrong value (at 120 mm/s a 0.1 s error misses every grip). | `config.yaml` › `pick_gate.*` | Run the stationary-belt measurement of [`pick-accuracy-findings.md`](pick-accuracy-findings.md) §3.1, then calibrate per §3.3. | NEAR |
| **C3** | `oblique_descent_enabled = false`. The belt-tracking slanted descent is implemented but disabled: an earlier simulation (E6: −65 mm error, 9% hit rate) was run with the biased descent model (T2, now fixed) and has not been repeated. | `config.yaml` › `pick_gate.oblique_descent_enabled` | Close C5 and T3, re-run E6 on the PLC simulator, then re-test on hardware. | NEAR |
| **C4** | `robot.rotation.offset_deg = -62.0` and `object_types.<type>.heading_offset_deg` (both `0.0`) are unconfirmed; they must be re-checked after any change to the suction-cup marker or mounting. | `config.yaml` | Hardware run reading the per-pick `[ROTATE]` log (`vision_angle / board_heading / rotate_cmd / rotate_at_gate / rotate_at_end`). | — |
| **C5** | **`robot.interpolator.{v_max, a_max, d_max}` have never been validated against the real arm.** This block is the trajectory-time model — i.e. the scheduler's estimate of processing time per job. Every pick-time prediction, gate lead, descent-time model and feasibility test is built on it. | `config.yaml` › `robot.interpolator`, `basis-theory.md` §3 | `speed_tuning` in the CLI runs a tilted heptagon and reports measured-vs-modelled ratio plus a first-order `v_max` suggestion (report-only, never writes config). Then apply by hand. | **NEAR — blocking** |
| **C7** | `scheduling.arm_cycle.occupancy_worst_s = 3.282` / `grab_worst_s = 1.817` are the trajectory model's worst cases, not measurements, and they are stale: `python3 -m modules.tools.derive_rank_bounds` gives 3.220 / 1.792 with the current config (the pre-refactor script gives the same). `scheduling.setup_time_s = 0` ignores executor overhead (rotation commands, handshakes). With `queue_depth_k = 1` they cap the queue-bounded laws' band at $175/(3.28+1.82) ≈ 34$ mm/s against the 30 mm/s floor, so `predictive_rank` can barely move the belt. | `config.yaml` › `scheduling.arm_cycle`, `speed.laws.predictive_rank` | Close C5, measure per-pick grab and occupancy from `[GATE]`/`[PLAN]` logs on hardware (or the simulator with measured latencies), then decide $k$ (0 gives $L/g ≈ 96$ mm/s). | NEAR |
| **C8** | The belt scale rests on one 4 s stopwatch reading (commanded 40 mm/s, belt travelled 165 mm, PLC reported ≈ 81 mm/s). The Siemens position/speed feedback is ≈ 2× the true belt motion; the cause inside the PLC scaling is unknown, and a ±0.2 s timing error is ±5 % on the scale. Every tracked object's position is dead-reckoned from this value. Before 2026-09, `setspeed 20` measured 62–71 mm/s with scale 1.0, so the encoder scaling may have changed with the hardware. **The file currently holds `conveyor.position_scale_mm = 1.0` while its comment describes the 0.5 measurement** — confirm which is intended. | `config.yaml` › `conveyor.position_scale_mm` | Mark the belt, move it ≥ 500 mm, and divide the tape-measured distance by the change of `conveyor_position_raw` in `status.csv`; repeat at two speeds. | NEAR |
| **C6** | The physical-calibration stage of the config checker is a documented stub: `conveyor.frame` (θ, origin), `pick_gate.robot_movement_delay_s`, and `conveyor.position_scale_mm` have no automated procedure. | `calibrate_everything.py` › `_todo_physical_calibration()` | Implement, driven by `test_vision_only` and the stationary-belt `[GATE]` measurement. | FAR |

---

## B. Configuration inconsistencies

| ID | Issue | Where | Tag |
|---|---|---|---|
| **G1** | `speed.static_mm_s = 10` lies **outside** the band `speed.band = [30, 100]` every adaptive law is clamped to. Startup seeds the belt there, so the first adaptive commit always steps it into the band (under `constant` the belt simply runs at 10). Decide whether the static seed is meant to be exempt from the band or whether one of the values is wrong. | `config.yaml` › `speed` | NEAR |
| **G2** | `scheduling.arm_cycle.cycle_s = 3.0` implies ~20 picks/min, while `doc/proposal/abstract-en.tex` claims a nominal **30–60 picks/min**. One of the two is wrong, and the discrepancy will be visible to reviewers. Note `cycle_s` is the *calibrated arm cycle* $t_\text{pick}$ feeding $\mu_\max = 1/t_\text{pick}$, so this also shifts the whole rate target $\lambda_\text{nom}$. | `config.yaml` vs the abstract | **NEAR — publication-blocking** |
| **G3** | `speed.laws.inverse_density.density_length_mm = 0.0` makes the density region derive to $L_\text{meas} = u_\max = 363$ mm, which starts upstream of the workspace ($u_\min = 188$). Density is therefore measured over a longer region than the one being regulated. This may be intentional (the camera previews parts before they arrive) but it is undocumented and it changes the meaning of $\rho$. | `config.yaml`, `basis-theory.md` §6.1–6.2 | NEAR |
| **G4** | `robot.rotation.home_tolerance_deg = 0.0` keeps the "axis not yet home at grip" check strict, whereas the documented intent of a positive value is a warn-only degradation. Confirm which behaviour production wants. | `config.yaml`, `basis-theory.md` §5.2 | — |
| **G6** | `pick_gate.arrival_tolerance_mm / _max_mm` were widened to 15/50 mm to make picks land; that is a symptom treatment. The suspected root causes (C2 latency calibration, C3 vertical-vs-oblique descent) are still open, so the band cannot be tightened back yet. The wide band also opens the O2 hazard window (§C.1c) on the deployed PLC. | `config.yaml`, `basis-theory.md` §4.4 | NEAR |
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
| **L5** | **Siemens DB1 carries no handshake bit.** It is unverified whether the ST program edge-triggers on a `CommandID` change; if it does, a `rotate_absolute` sent back-to-back with a `change_speed` can be silently dropped. | Silent command loss between the two most timing-sensitive Siemens commands — rotation and belt speed. Verify in TIA Portal and record the finding in `doc/plc/data-contract.md`. | **MID — highest risk** |

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
| **T1** | The pick gate's lead offset does not model the pick descent separately. The PLC simulator (scan-accurate `Matching_Code_10`, no servo lag) shows the existing lead lands centred up to 120 mm/s (median along-belt error −0.3…−2.6 mm, every dispatched pick gripped) because the empirical `robot_movement_delay_s` already contains the 80 ms State-10 descent; adding the 0.39 s S-curve model misses every pick (−24 mm at 60 mm/s). The 47 mm error originally attributed to T1 came from a plant model that uses the S-curve descent. | The reported hardware accuracy loss is **not reproduced** by an ideal PLC, but only because the simulator's default tag latency (2 ms) makes gate→dispatch + dispatch→contact (0.186 s) match the configured lead. Shifting the simulated latency by ±0.1 s reproduces the hardware signature exactly (error ∝ v, 13/23 grips at 120 mm/s): the cause is the unmeasured real latency (C2, T3), not scheduler logic (2026-09-16 simulator report, archived). The explicit `pick_descent_time_s` term exists for when a measurement requires it. | **NEAR — needs T3 measurement** |
| **T3** | The real dispatch→contact time is unmeasured. Models disagree (0.08 s PLC State-10 ramp vs 0.39 s S-curve), and servo lag decides between them: in the PLC simulator a first-order servo lag of τ = 50 ms keeps the cup from ever reaching `pickup_height` (0 contacts in 23 picks, at 60 and 120 mm/s), because the 80 ms ramp ends and the lift starts before the axes catch up. | Shallow or missed grips on hardware may be a descent-depth problem (P11), not a timing one. | **NEAR — bench measurement §3.1** (stationary belt: `dispatch_to_contact_s`, and the minimum z reached vs `pickup_height`) |
| **T4** | The realtime loop only plans while idle; above ≈120 mm/s an object must be committed to before it reaches the workspace. A plan also needs the arm parked a full gate lead before the intercept (≈ 0.22 s), and the gate aborts when fired more than `pick_gate.late_abort_mm` late. | A feasibility cliff, not a timing error — explains why 120 mm/s is a hard wall. The sequence planners look ahead over the whole queue but still commit one pick at a time from idle, so the cliff remains; a speed ceiling (predictive-rank's $v_\max$) is the available mitigation. | NEAR |
| **T5** | `core/motion.trajectory_time` chains segment speeds forward only (no backward pass); the deployed PLC's own `MC_Inter_Curve_Vel` has the same gap (`plc/version-diff-and-defects.md` **P7**). | A physically correct two-pass model would be *longer*, while the deployed PLC already ends motion 0.15–0.18 s *before* the forward-only model (G8). Do **not** port the research repo's backward pass until P7 is patched on the PLC; until then the forward model is the conservative estimate. | NEAR — blocked on P7 |

### C.2 Control / measurement gaps

| ID | Limitation | Consequence | Tag |
|---|---|---|---|
| **L6** | **No suction verification.** A pick is booked as successful when the arm's *motion* completes; the vacuum is never checked, and a dispatched grip is never retried. | The controller's own success count over-states reality. Any $\sum U_j$ reported from software is a lower bound on true losses unless picks are also counted by an independent means (bin count, downstream camera). | NEAR |
| **L9** | The accelerating-belt forms of the gate offset are derived but deliberately **not implemented**; the single-term $v \cdot T_\text{delay}$ is only correct while the belt is steady, which the speed controller guarantees by taking no decision at all from goto dispatch to cup contact (`RealtimeState.pick_committed`), whatever the law or its gate. A commit issued while the arm is free may still be ramping when the next gate fires (≤ `speed.commit.max_step_mm_s / conveyor.accel_mm_s2` ≈ 0.9 s): the plan's forecast includes the ramp, the gate offset does not. | A gate that opens right before a goto (every gate does, at `arm_free`) leaves this residual; the ramp-aware offset is the fix if it matters. `tests/test_runtime.py` pins the freeze. | NEAR |
| **L10** | **The default speed law is open-loop with respect to misses.** The configured law is `constant`; `inverse_density` reacts to density and spacing only. | The outer loop described in `doc/proposal/abstract-en.tex` — predicting how many parts will miss their deadline and adjusting belt speed from that prediction — exists as the opt-in `predictive_rank` speed law (`basis-theory.md` §7.3), but it has only run on the PLC simulator and the sandbox; its band is currently pinned by C7. | **NEAR — core** |
| **L11** | The `inverse_density` spacing cap inspects only the leading `speed.laws.inverse_density.spacing_lead_objects` (default 4) parts. | A cluster further upstream is invisible to the speed law until it reaches the front. | NEAR |
| **L12** | The operator console (`main.py --interface`) binds `0.0.0.0` with **no authentication**: anyone on the cell's LAN can move the arm, run the belt or start a scenario. Accepted by the user for the lab network (2026-09-16). | Do not expose the PC beyond the cell's network. Revisit if the cell moves to a shared network (a token on `POST /api/*` is the minimal fix). | MID |

### C.3 Experiment / reproducibility gaps

| ID | Limitation | Consequence | Tag |
|---|---|---|---|
| **L16** | **The sandbox has not been validated against a reference.** `sandbox/` runs the robot's plugins on the robot's delta model, so its absolute numbers differ from the research repo's published ones (backward pass, motion parameters, per-segment formula). The paper's headline comparison has not been re-run in the sandbox to record which conclusions carry over, and the sandbox's plant finishes motion at the model's time (G8) and ignores camera latency. | Treat sandbox results as rankings and trends until that comparison is recorded in `decision-log.md`; confirm a finding on the PLC simulator before hardware. | NEAR |
| **L17** | **The part record's grip judgement is geometric, and virtual parts are idealised.** `runtime/outcomes.py` books `picked` / `miss_grip` by comparing the part's *tracked* centre with the cup at the first pose sample inside the contact band, against `runtime.grip_tolerance_mm` (12.7 mm, taken from the simulator and the paper, never measured on this cup and these boards). For real parts the tracked centre carries the tracking and latency error (C2, T3); the pose and belt samples are up to one perception period (25 ms) apart. `simulate_feeder` parts never overlap-check, never slip and are seen with no detection latency or noise unless `feeder.detection_latency_s` / `position_noise_mm` are set. | Real-part grip counts are an estimate until checked against a bin count (L6); virtual-part runs measure scheduling, belt and arm timing on the real cell, not suction. Report them as such. Measuring the real capture tolerance (off-centre picks at known offsets) closes the first half. | NEAR |

---

## D. Repository hygiene

| ID | Issue | Status |
|---|---|---|
| **D8** | `doc/proposal/` is untracked. LaTeX build artefacts are already ignored, so committing `abstract-en.tex` is safe — but it names the authors, and author names were deliberately purged from git history once already (see `decision-log.md`). Decide before committing. | **Open — user's call** |
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
