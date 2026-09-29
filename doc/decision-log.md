# Decision Log — Delta Robot

> ## ⚠ HISTORY ONLY — nothing in this file describes how the system works today.
>
> Every entry below is a design that was **tried and replaced**, a calibration that has
> **already been applied**, or an idea that is **parked**. Read it to understand *why* the
> current design is shaped the way it is — never to learn what the code does. For current
> behaviour read [`basis-theory.md`](basis-theory.md) and
> [`basis-programming.md`](basis-programming.md); for what is still unresolved read
> [`open-issues.md`](open-issues.md).
>
> Its purpose is to preserve the *why-not* reasoning — the material a paper needs when
> justifying a design choice against the obvious alternatives — without letting that
> reasoning contaminate the descriptive documents.

---

## 1. Superseded algorithms and design choices

### 1.1. Belt speed proportional to load — rejected

**Rejected model**: $v = A \cdot N + v_{\min}$ ("run the belt fast when the cell is busy").

**Why rejected**: belt speed does not set throughput; the serial arm's pick cycle does.
Speeding the belt up under high density only shortens each part's transit through the
workspace and pushes parts past $u_{\max}$ unpicked. Replaced by the **inverse** density law
(`basis-theory.md` §6.2), where higher density means a *slower* belt.

### 1.2. Speed commits at the grip instant only — replaced

The original controller committed a `change_speed` only at the moment of grip: at most one
commit per pick cycle, i.e. one per 2–10 s. Density changes therefore sat uncommitted for
seconds, and on hardware the belt felt laggy and uneven. Quantitatively, the large speed
jumps that policy allowed need acceleration ramps longer than the window it assumed was
free.

Replaced by the inverted policy (`basis-theory.md` §6.5): commits are *opportunistic* —
allowed from the executor wait loops and the idle loop, throttled ≥ 0.75 s apart — and
suppressed **only** inside the gate-critical window.

### 1.3. Fixed wall-clock gate-abort deadline — replaced

A pick that had not fired within a fixed wall-clock deadline was aborted. This fired
spuriously whenever the belt slowed after plan-build, since the deadline had been computed
against the earlier, faster belt. Replaced by a **progress-based stall check**: abort when
the tracked object's $u$ advances less than 0.5 mm over several seconds, or when the track
is lost.

### 1.4. Blanket object removal on a failed pick — replaced

Any failed pick used to remove the object from the tracker. That dropped objects which were
still perfectly pickable, and it also deflated the density count $N$, which sped the belt up
immediately after a failure — precisely the wrong response. Replaced by the exclusivity rule
of `basis-theory.md` §4.7: removal keys off whether the **grip was dispatched**, and a
pre-grip abort only unclaims.

### 1.5. Parking the arm upstream by $v \cdot t_d$ for the oblique descent — replaced

The first version of the belt-tracking slanted descent shifted the whole *park* position
upstream by the board's travel during the descent. At operating belt speed this placed the
arm outside the workspace. Corrected so that only the pick-phase **contact** point shifts
downstream; the goto/park target is unchanged (`basis-theory.md` §4.5).

### 1.6. Wrapping the wire angle to $[-180°, 180°)$ — replaced

The Siemens rotation command encodes **spin direction as well as position**, so wrapping the
commanded angle at the IPC boundary flipped the direction of travel: a $179° \to 180°$ step
became $179° \to -180°$ and drove the axis nearly a full turn the wrong way. This was the
"random over-rotation" fault. Fixed by making the wire layer a verbatim
radians→degrees identity clamped to $\pm 359$, with the single minimal-turn wrap moved up to
the algorithm layer (`basis-theory.md` §5.2, three-layer convention).

### 1.7. Time-based pick dispatch — replaced

The predictor originally converged on a pick *time* and fired the pick when that time
arrived, which made the pick sensitive to belt-speed estimate noise (the "arrive → wait →
miss" lag). Replaced by a **fixed-point park position plus a live positional gate**
(`basis-theory.md` §4.3–4.4): the predictor chooses where to park, and the pick fires when
the tracked object physically reaches that position.

### 1.8. Single-threaded real-time loop — replaced (for `production` only)

The original scheduler blocked the whole loop inside `executor.execute()` for 4–7 s per
pick, so belt sampling, vision polling and re-anchoring all froze during a pick and every
second-and-later pick in a burst landed behind its part. Rebuilt as the two-thread model
(perception/state daemon + decision/execution main thread over one guarded `RealtimeState`).

**Still relevant**: the simulated scenarios were never migrated and still run the old
single-threaded harness — that is open issue **L7**, not history.

### 1.9. `run_test.py` subprocess + matplotlib launcher — removed

Conflicted with the OpenCV window on real hardware. Replaced by the in-process web dashboard
(`modules/interface.py`, `--interface`); simulation moved into the scheduler itself via
`--simulate-executor`.

### 1.10. Config keys retired during the frame migration

`pickup_window_x` / `pickup_window_y` (replaced by `conveyor.workspace_window_uv`),
`accuracy_points` (replaced by `conveyor.accuracy_points_uv`), `throughput_spawn_x`,
`period_s`, and the `object_A`-style class naming (replaced by `object_types.{QFP,TQFP}`).
`belt_speed_static_mm_s` was renamed from an earlier key **without a fallback** — an old
config file will not load its value silently, it will simply use the default.

### 1.11. Law-specific speed cadences and opportunistic commits — replaced by setpoint gates

**Replaced design** (until 2026-09-25): each speed law carried a `cadence`. `inverse_density`
computed its target inside the perception thread every 25 ms and was committed
"opportunistically" — from the executor's wait loops after contact and from the idle loop,
throttled to one commit per 0.75 s; `predictive_rank` was called directly by the loop, with its
own signature, only while the arm was free. Band clamping and the step filter lived inside each
law, and the loop special-cased every law by name.

**Why replaced**: adding a law meant editing the loop, and the invariants every law must keep
(L9 freeze, band, step, deadband) were re-implemented per law. Now the perception thread only
observes; a law is a function of a `SpeedView`; a named **setpoint gate** decides when it runs
(`inverse_density` defaults to `at_contact`, `predictive_rank` to `arm_free`); one commit policy
applies the band, step, deadband and resync; and the controller takes no decision at all while a
pick is committed. **Behaviour change**: `inverse_density` now commits at the arm-free instants
(every `speed.control_period_s` while idle) and once at cup contact, instead of every 0.75 s
from the perception target. On the PLC simulator the outcome stayed within run-to-run noise
(§3, 2026-09-25).

### 1.12. Built-in single-pick `spt` path — replaced by a dispatch rule

**Replaced design**: `spt` was not a plugin but `_build_realtime_pick_plan`, which predicted
every object's intercept with the realtime predictor (a 0-lead solve followed by a caller-side
lead) and ranked by start → pick → bin path length.

**Why replaced**: two intercept solvers had to be kept identical by a test. `spt` is now a
dispatch rule over the same jobs every planner sees, the registry expands it into a ranking, and
the commit step (live intercept, deadline, IK) is shared. The job pipeline's intercept on a
steady belt equals the old realtime predictor to 1e-9 s over a grid (the frozen reference lives
in `tests/test_scheduling.py`); the one addition is the deadline check at commit.

### 1.13. The planner called `kim` — renamed `drop_longest`

The robot's `kim` (2026-09-17 port) added jobs in belt order and, while the sequence was late,
dropped the job with the longest processing time — possibly several per insertion. That is a
Moore-Hodgson-style rule, not Kise-Ibaraki-Mine's algorithm, which removes exactly one job: the
one whose removal leaves an on-time set that frees the arm earliest. The research repo withdrew
the same mis-labelled implementation before the paper. The name `kim` now means the
single-removal algorithm (and `kim_release`, `cardinality_*`, `dp` came with it); the old rule is
kept as `drop_longest`, and `config.yaml` selects it so production behaviour did not change
(open-issues G10).

### 1.14. Single-thread simulated scenarios — retired

**Removed**: `test_throughput`, `test_accuracy`, `test_acceptance`, `evaluate`, with
`SimulatedExecutor`, `SimulatedSpeedSource`, `SimulatedImageProcessing`, `EvaluateExecutor`,
the `--simulate-executor` flag and their config keys.

**Why**: they ran a second, single-threaded program (open-issues L7): not the code the cell runs,
so neither a faithful command test (the PLC simulator is) nor a fast algorithm bench (the sandbox
is). The static-point accuracy runs they offered are the calibration procedures of
`basis-programming.md` §9.2 now. Only `production` and `test_vision_only` remain, in a scenario
registry that admits new scenarios.

---

## 2. Calibrations already applied

### 2026-07-09/10 — hardware run

* Sorting-bin drop positions `QFP` / `TQFP` and `pickup_height` nudged from live pick data.
* `pick_arrival_tolerance_mm` / `_max_mm` widened to 15/50 mm; the previous 5/10 mm band was
  too tight for the belt-speed range in use. *(The root cause behind needing the wider band
  is still open — see `open-issues.md` G6.)*
* `belt_speed_static_mm_s` raised to 120 mm/s; `belt_speed_min/max_mm_s` rebalanced to
  30–100 mm/s. *(This is what created the inconsistency logged as G1.)*

---

## 3. Repository restructuring

### 2026-07-11 — post-thesis cleanup

* Documentation consolidated into a fixed file standard under `doc/`.
* `report/` (the LaTeX graduation thesis), `tests/` (the old pytest harness),
  `doc/archive/`, `.trash/`, and all superseded documentation sources moved to a local-only,
  gitignored `.archive/` directory. Superseded sources included `theory_basis.md`,
  `academic_report.md`, the old status-log `ai_context.md`, `Yolo_training_report.md`,
  `evaluate.md`, `evaluate_filled.md`, `test.md`, and `rotate_t4_instability_report.md`.
* Personal information (thesis authors' names and student IDs, only ever present under
  `report/` and `doc/archive/report_draft_v1/`) purged from git history with
  `git-filter-repo`, together with heavy blobs (`report.zip`, `models/small@1280_old_dataset/`,
  per-epoch `.pt` checkpoints).
* A full mirror of the pre-purge repository is kept locally at
  `../Delta_robot_git_backup_2026-07-11.git` as the permanent archive of the original
  history — **never push it anywhere**.

`.archive/` is local-only and must never be read, referenced, or restored into the tracked
tree.

### 2026-08-10 — documentation re-standardisation

The repository moved from "finished thesis project" to "starting point for scheduling
research". Documentation was re-standardised to six files under `doc/`
(`context.md`, `basis-theory.md`, `basis-programming.md`, `open-issues.md`,
`decision-log.md`, `dev-note.md`).

* `README.md` was rewritten from ~370 lines to a lean quickstart. It had accumulated roughly
  forty statements contradicting the live code — a retired `test_conveyor` scenario, an
  `interpolar_points` default of 4 (the real value is 7), `intercept_lead_time_s` given as
  1.6 s (the real value is 0.8), a claim that `_belt_lead_offset_mm` was an empty hook
  returning `0.0` when it is implemented, a roadmap of already-completed work, a list of
  bugs all marked `[FIXED]`, and links to five documents that no longer exist
  (`realtime_pick_redesign.md`, `rebuild_plan.md`, `bug_report_final.md`, `frames.png`,
  `theory_basis.md`).
* All still-unresolved items were collected from `dev-note.md`, `README.md` §6,
  `basis-theory.md` prose and `calibrate_everything.py` into `open-issues.md`.
* Historical narrative was removed from the descriptive documents and collected here.

### 2026-09-16 — PLC documentation rewritten for the target Omron program

* `doc/PLC_Program_description/` (rung-by-rung notes of `Matching_Code_10`, with dead
  `file:///d:/...` source links) was deleted and replaced by `doc/plc/`, which describes the
  target program `delta_paper_0_2` and reviews it against the Python side. The Sysmac exports
  of both versions are in `OMRON/`.
* Changes in the target PLC program recorded there: the six hard-wired
  `MC_Inter_Curve_Vel` instances were replaced by one instance driven by
  `FB_ICV_Sequencer` (N = 2…32 points); the cosine corner-speed look-ahead was removed
  (blend segments now target `V_max`); the +0.08 s soft start was dropped from the segment
  time estimate; `argument_time` was removed from `From_pc` and `n_points` added;
  `FB_TeachingMode` was added.
* The defects found in the review were registered as open issues P1–P11.

### 2026-09-16 — PLC review re-checked against the Sysmac project files

* **Source of truth changed.** Both `.smc2` projects were unpacked and their declarations, ST
  bodies and ladder rungs dumped to text. This replaced the screenshots as the source for
  `doc/plc/`.
* **P8 closed.** `fb_PumpTimer` is declared `TP` in both programs, so the pump release
  timing is as intended.
* **P6 narrowed.** `ICV_t` is declared `ARRAY[0..30]`.
* **P2 and P9 confirmed** from the declarations: `Out_Error_IK_OWS` is a `VAR_OUTPUT`, and
  `start_teaching_mode` has no initial value.
* **Kinematic geometry corrected.** `kinematics.md` and `basis-theory.md` had carried
  `Base = 320` / `EndEffector = 94` from an old text export. The PLC declares 346.4 / 86.6
  (inradii 100 / 25 mm).
* **New files and defects.**
  - `program-old.md` documents the deployed program `Matching_Code_10`.
  - `config-review.md` re-derives the config from the PLC.
  - The deployed program's own defects were registered as O1–O5 and O9; configuration
    findings as G7 and G8.

### 2026-09-16 — PLC simulator built; `test_module.py` fake PLC replaced

* **What was replaced.** The old `modules/test_module.py` fake PLC had three faults:
  - it moved the arm linearly over `argument_time`, which the real PLC ignores;
  - it reported `task_state` 1/0 where the real PLC reports 2;
  - it reported the belt position in cm where the real field is mm.

  Its physics were replaced by `modules/plc_sim`: a scan-accurate port of `Matching_Code_10`,
  plus the Siemens belt and rotation, simulated boards and a simulated camera, all on the same
  JSON-lines protocol. `test_module.py` is now a launcher of that simulator, and
  `main.py --cli --dummy` uses it.
* **Open issue L8 closed.** `production` runs offline with `main.py --scheduler --sim`, and a
  60 s run completed with 16 boards placed (`plc/simulator.md` §4).
* **L7 narrowed** accordingly.
* **Unchanged.** `investigation/virtual_cell.py` remains as the parametric plant used by the
  pick-gate investigation.

### 2026-09-16 — Operator console added to the web dashboard

* **Before.** The dashboard was monitoring only: `GET` for the page, SSE and MJPEG. The
  scenario was fixed on the command line, and manual moves needed the CLI.
* **Now.** `main.py --interface` run on its own starts an operator console
  (`modules/supervisor.py` plus a `POST /api/*` hook in `modules/interface.py`). It offers
  manual control, belt and cup commands, scenario start/stop, teach points and a log. The
  existing Live and Charts views are kept.
* **Supporting changes.**
  - `run_scheduler_scenario` gained `stop_event`.
  - The executor construction in `main.py` moved into `_build_executor`, shared by the
    scheduler mode and the console.
* **Access.** The user chose LAN access without authentication; this is registered as open
  issue L12.
* **Manual command design.** Manual commands are gated on the arm being still, because of
  O2 and O6. The pump is switched in place with a zero-length command 3, because commands
  5/6 have no effect on the deployed PLC.

### 2026-09-16 — Pick-accuracy investigation promoted from a standalone report into `doc/`

* The Vietnamese narrative report `bao-cao-do-chinh-xac-pick.md` (repo root, written
  2026-08-28 after a three-round investigation, `investigation/REPORT*.md` and
  `results-*.txt`) was formalised as the English defect register
  [`pick-accuracy-findings.md`](pick-accuracy-findings.md) and registered in
  `open-issues.md` §C.1d as **T1–T9**.
* No code changed as part of this — every finding was already reproduced by an existing
  automated test before this promotion; this entry only records that the findings moved from
  an ungoverned root file into the tracked documentation set.
* Cross-references added: `open-issues.md` **C2**/**C3**/**L9** now point at the specific
  defects (T1–T3, T6–T7) that block acting on them; `basis-theory.md` §4.4/§4.5/§6.5 gained
  pointers to the same IDs, and §6.5 was corrected — it had described the gate-critical
  commit suppression as applying to the goto-flight wait loop, which it does not (**T7**).
* Headline finding: the pick gate's lead offset (`basis-theory.md` §4.4) omits the soft-start
  and descent time once oblique descent is disabled (**T1**), and the log field meant to
  calibrate it is itself biased when oblique is off (**T2**) — together the likely primary
  cause of the reported accuracy loss above ~100 mm/s. Neither is fixed yet; `robot_movement_delay_s`
  (**C2**) must not be recalibrated until T2 is fixed.


### 2026-09-16 — Pick-gate defects fixed; T1 re-assessed on the PLC simulator

Code (`modules/scheduler.py`, `modules/cli.py`):

* **T2** — with the oblique descent off, `plan.descend_time_s` (logged as `[GATE] t_d_model_s`,
  and used by the rotate-fallback deadline) is the vertical model `_descent_time_s(settings, 0)`;
  it no longer carries a belt slant.
* **T6/T7** — `RealtimeState.gate_critical` and the 2 s proximity window were replaced by
  `pick_committed`: speed commits are suppressed from goto dispatch until cup contact. The
  grip-instant commit was removed. Rejected alternative: keeping the proximity window and adding
  it to the goto loop — it still let a ramp run through the goto flight the plan was built on.
* **T8** — `ConveyorSpeedSource.sample` stamps the reading at the round-trip midpoint.
* **T9** — after a failed pick the next plan starts from the last reported pose.
* **T4 (partial)** — plans must park a full gate lead before the intercept (the predictor
  pushes the intercept downstream), and the gate aborts when fired more than
  `gate_late_abort_mm` late. The planning cliff itself remains.
* **O2 (PC side)** — arrival requires the pose inside the packet's final vertical segment.
* **O1 (PC side, realtime loop)** — plans with a waypoint failing `ik_reachable` are skipped.
* **O4** — CLI `pick`/`release` fill all points with the current pose (row deleted).

**T1 reversed.** The planned fix — adding the modelled vertical descent (0.39 s) to the gate
lead — was implemented and tested on the PLC simulator first. With a static belt and no servo
lag, the unchanged lead gripped every dispatched board (median along-belt error −0.8 mm at
60 mm/s, −2.9 mm at 100 mm/s), while the added term missed all 23 picks (−24 mm / −40 mm). The
empirical `robot_movement_delay_s` already contains the PLC's 80 ms State-10 descent
(`[GATE]` in simulation: gate→dispatch ≈ 0.09 s, dispatch→contact ≈ 0.10 s). The 47 mm
figure of 2026-08-28 came from `investigation/virtual_cell.py`, whose plant uses the S-curve
descent. The term was kept as an explicit, default-zero `pick_descent_time_s`.

**T5 not ported.** The sandbox backward pass makes the trajectory-time model longer, while the
deployed PLC already finishes motion 0.15–0.18 s earlier than the forward-only model
(`plc/config-review.md` §3). Porting it before the P7 patch would move the model away from the
real arm.


### 2026-09-16 — Config slimmed (no effective value changed)

* Removed dead keys `scheduler.belt_ramp_s` and `object_types.*.thickness_mm` (parsed, never
  used). `vision.belt_estimator.*` and `vision.capture.device` were kept: the estimator draws a
  camera-derived belt speed on the vision window, and the device key is read by
  `camera_calibrate.py`.
* `main.py:_build_executor` re-read eleven `scheduler` keys with its own fallback defaults,
  two of which disagreed with `SchedulerSettings` (`belt_speed_min/max_mm_s` 50/120 vs 30/0).
  It now takes them from `SchedulerSettings`, which gained `pick_arrival_tolerance_mm`,
  `pick_arrival_tolerance_max_mm` and `rotate_refresh_max_delta_deg`.
* `EthernetCom.DEFAULT_CONFIG` corrected (`port` 502→44818, `interpolar_points` 4→7), and
  invalid JSON now stops the program instead of silently running on those defaults.
* The file was re-rendered 249 → 161 lines by `modules/config_io.write_config` (scalar arrays
  on one line); both calibration tools now write through it instead of `json.dump(indent=4)`,
  which re-expanded the file on every save.
* Verified: the parsed config equals the previous file minus the two removed keys, and all 66
  previous `SchedulerSettings` fields are unchanged; investigation suites, the config checker
  and a PLC-simulator production run pass.
* YAML was deferred at this point: plain PyYAML would erase comments on every
  calibration-tool save (resolved 2026-09-17, below).

### 2026-09-17 — `config.json` replaced by a commented `config.yaml`

* Closes **G9**. Every key now carries its meaning, units and the `open-issues` item that
  governs an uncalibrated value, so a value's status no longer lives only in the docs.
* Plain PyYAML was rejected because it erases comments on every calibration-tool save; a
  hand-written file plus a machine-written overlay was rejected because a real value would
  then live in two files. `modules/config_io` uses `ruamel.yaml` round-trip instead (new
  dependency): it merges written values into the existing document, so unchanged values keep
  their comments.
* The replaced JSON writer's compact layout and `json.JSONDecodeError` handling went with it;
  `modules/rotate_sweep_sim.py`, which parsed the JSON directly, now reads through `config_io`.
* Verified: the parsed YAML equals the previous `config.json` exactly (key order aside).

### 2026-09-17 — Manual z band replaced by the `robot_limits` motion envelope

* The web console derived its z band from other keys, `[pickup_height − 2, clearance_height +
  20]` = [−305, −240], which rejected any goto or jog near the post-homing calibration pose
  (command 4 drives every joint to 0°, z = −230.2). The gateway checked only the XY radius
  (`limit_radius_xy`).
* Replaced by an explicit `robot_limits {radius_xy_mm, z_min_mm, z_max_mm}` block (z_max = −220,
  chosen by the user) enforced on every goto/trajectory point at `PLCGateway.send_package`, so
  the realtime loop, CLI and console share one envelope. The top-level `limit_radius_xy` key is
  refused at startup rather than silently ignored.

### 2026-09-17 — KIM planner and predictive-rank speed law ported as plugins

* Ported from the scheduling sandbox (`../python for scheduling`, experiments 3–4: KIM with
  predictive-rank was the best combination, +11 % sorted parts/min over the best constant
  speed). Nothing is imported from the sandbox at runtime; the timing model is the robot's
  own, generalised to arbitrary start pose/time and a ramping belt, and pinned to the realtime
  predictor on a steady belt by `investigation/test_planning.py`.
* **Default kept at the baselines** (`danger_distance`, `inverse_density`) by the user's
  choice: the sandbox arm is idealised (zero command latency), and the worst-case
  occupancy/grab times the new law's ceiling needs come from the unvalidated trajectory
  model (C5, C7).
* Deviations from the sandbox, and why:
  * Jobs are ordered by belt position before KIM runs: the robot tracker is a dict, and KIM's
    stable sort on `reachable_at = now` keeps the input order.
  * predictive-rank candidates must lie within one `belt_speed_max_step_mm_s` of the setpoint:
    the robot's commit is rate-limited, and a candidate further away would be scored at a speed
    the belt does not run at.
  * The tie rule follows the sandbox **code** (an equal score at a faster speed replaces the
    current setpoint), not its docstring (which says only a strict improvement does); the
    experiment results were produced by the code.
  * No stacking precedence: the robot's vision reports no occlusion, so `blockers` is empty.
  * Planning runs on a snapshot outside `state_lock`, and the chosen entry is re-validated
    under the lock, because the sandbox's instantaneous decisions take real time here.
* First PLC-simulator check (`investigation/sim_plugin_matrix`, 90 s, one seed each, every
  dispatched pick gripped, median along-belt error within ±0.8 mm in all runs):

  | planner / speed law | feed 2.5 s: placed / spawned | feed 1.2 s: placed / spawned |
  |---|---|---|
  | danger_distance / inverse_density | 28 / 37 | 25 / 75 |
  | kim / inverse_density | 33 / 37 | 35 / 77 |
  | danger_distance / predictive_rank | 27 / 38 | 23 / 75 |
  | kim / predictive_rank | 33 / 37 | 34 / 76 |

  KIM decisions took ≤ 12 ms and the re-validated contact time drifted ≤ 14 ms from the plan.
  predictive_rank held the belt at 30 mm/s throughout: with the model-derived worst cases its
  band is [30, 34.3] mm/s, so only 30 mm/s is on the 5 mm/s grid (open-issues C7). The runs
  are single-seed and short; they show the integration works, not an effect size.

### 2026-09-17 — Danger-line tier dropped; planner renamed `spt`; `adaptive_speed_enabled` removed

* The baseline planner ranked objects past 2/3 of the band first (most downstream), then by
  the shortest start → pick → bin path. The user dropped the danger tier: the rule is now pure
  SPT with path length as the processing-time proxy (on this arm a shorter path is a shorter
  cycle). Object loss near the band edge is left to the planner choice (`kim` optimises for it)
  rather than to a hard-coded override. The planner key `danger_distance` became `spt`.
* `scheduler.adaptive_speed_enabled` duplicated the speed-law choice; a static belt is now the
  `constant` speed law, and the key is refused at startup. `planner`, `speed_law` and
  `belt_speed_static_mm_s` moved to the top of the `scheduler` section.
* The 2026-09-17 simulator table above was measured with the danger tier still in place.

### 2026-09-17 — Robot pose read from the Omron UDP stream; run logs added

* The new Omron `Program_UDP` pushes X/Y/Z every ≈ 12 ms (measured: 83.3 packets/s, 0.04 ms
  jitter, no loss). The pose was previously read only by polling `pos_EE` over EtherNet/IP,
  bundled with the Siemens read in one ≈ 26 ms status round trip; the stream is fresher and
  lets a status poll read just the Siemens DB (≈ 9 ms).
* Kept: status polling itself. The belt position/speed and rotation live only in the Siemens DB,
  and the Omron handshake fields are not in the UDP payload, so the stream replaces the pose
  source, not the poll. Consumers see an unchanged status dict (`pos_EE` overlaid), so the
  scheduler, executors, CLI and console needed no changes; when the stream is silent the old
  full read is used.
* The three copies of the IPC closures in `main.py` (CLI raising nothing, scheduler and console
  raising) were merged into `PlcLink` with a `raise_errors` flag, which is also where the run
  log hooks every command and status poll.
* Bringing the stream up on the bench needed: the PC's `eno1` to own the PLC's destination IP
  (the PLC sent to a teammate's address first), and a ufw rule for UDP 9001 (default input
  policy DROP). An earlier receiver script also had an indentation error that made it exit
  before listening.
* The PLC worker now ignores SIGINT: Ctrl-C reached the whole process group and killed the
  worker with a traceback before the parent's orderly shutdown.

### 2026-09-17 — Belt feedback scale applied at the PLC worker, set to 0.5

* Symptom: the console chart showed ≈ 80 mm/s for a 40 mm/s command. `commands.jsonl` showed the
  PC sent 40; `status.csv` showed `speed_current` and the derivative of `conveyor_position`
  agreeing at ≈ 81 mm/s. A stopwatch measurement (165 mm in 4 s ≈ 41 mm/s) showed the belt
  follows the command and the Siemens feedback is ≈ 2×.
* `conveyor_position_scale_mm` used to be applied only to the position inside
  `ConveyorSpeedSource`; `speed_current` (dashboard chart, adaptive-speed resync) stayed raw.
  The scale now multiplies both at the worker boundary, so one config value makes every
  consumer consistent; `SchedulerSettings.conveyor_position_scale_mm` was removed to avoid
  applying it twice. The simulator path is exempt because it reports true millimetres.

### 2026-09-25 — Refactor: layers, typed settings, scheduling framework, in-repo sandbox

Executed from `doc/refactor-plan.md` (deleted afterwards).

* **Layers.** `modules/scheduler.py` (4 080 lines), `conveyor.py`, `EthernetCom.py`,
  `image_processing.py`, `planning/` and the top-level tools were split into `core/`
  (frames, tracking, kinematics, motion, trajectory, forecast, arm model, delta), `scheduling/`,
  `runtime/`, `comm/`, `vision/`, `ui/` and `tools/`, with imports pointing one way
  (`basis-programming.md` §1). `plc_sim` imports the PLC IK from `core/kinematics`;
  `test_module.py` became `python3 -m modules.plc_sim`; `CameraFrame`, `M_CAMERA_TO_ROBOT` and
  `BeltTracker.predict_position_R` (never used) were deleted; `modules/context.md` (a stale copy
  of `doc/context.md`) was deleted.
* **Settings.** `modules/settings.py` replaced `SchedulerSettings.from_config`, `load_config()`
  and the raw-key parsers of `supervisor`, `plc_sim`, `cli` and the tools: one nested dataclass
  per section, one default per key, unknown keys refused, moved keys refused with their new place,
  cell geometry required. `config.yaml` was restructured (`plc`, `robot`, `conveyor`,
  `object_types`, `vision`, `pick_gate`, `runtime`, `scheduling`, `speed`, `plc_sim`,
  `interface`); every value was carried over and checked key by key. Per-class data (bin, marker,
  symmetry, heading offset, YOLO class) moved into `object_types`. Removed keys: `default_speed`,
  `log_path`, `accuracy_*`, `throughput_spawn_y`, `throughput_emit_interval_s`,
  `test_acceptance_cycles`, `evaluate_*`, `conveyor.accuracy_points_uv`; `throughput_types` /
  `throughput_lanes` became `plc_sim.feed_types` / `feed_lanes`; `execution_margin_s` became
  `pick_gate.arrival_timeout_margin_s` (it is the pick executor's arrival margin);
  `nominal_xy/z_speed` and `release_descent_time_s` became `robot.packet_time.*` (they fill
  `argument_time`, which bounds the executor's arrival timeout — not a log-only value as the
  plan assumed). `main.py --set key=value` overrides any value for one run. Closes G5, G9.
* **Scheduling framework** (§1.11–1.13): registry of dispatch rules, planners, speed laws and
  setpoint gates; per-plugin typed config; one commit policy; `SpeedView` / `PlanContext` /
  `Job` as the plugin contract; `modules/scheduling/README.md` with worked examples that were run
  verbatim in the sandbox. Every algorithm of the research repo was ported: rules `edd`, `fifo`,
  `least_slack` (plus `spt`), planners `kim`, `kim_release`, `cardinality_arrival`,
  `cardinality_release`, `dp`, the four rollouts, speed laws `predictive_rank_hysteresis`,
  `bound_only`, `backlog_patience`, `min_slack`, `rate_schedule`, `periodic_two_speed`, and the
  gates `always`, `arm_free`, `at_contact`.
* **Arm model.** `core/arm_model.ArmModel` is the interface the job model uses; `core/delta.DeltaArm`
  is the one delta model, shared by the robot and the sandbox; `sandbox/models/point_mover.py` is
  an abstract arm.
* **Sandbox.** `sandbox/` replaced the research repository for algorithm work (feeders
  `script`, `periodic`, `poisson`, `regime`, `bursty`; model/plant split; parallel sweeps with a
  manifest). The research repo was tagged `paper-submitted` and frozen; the paper is reproduced
  from that tag. A 300 s run takes 0.05–0.16 s against 66 s for one run of the research simulator
  (profiled 2026-09-25: 95 % of its time went to 10⁶ trajectory-time evaluations). The
  `scheduling_sandbox` symlink and `investigation/_bridge.py` were removed, and with them the
  parity test (closes D10).
* **Scenarios.** Only `production` and `test_vision_only` remain (§1.14; closes L7).
  `--no-plc` runs `test_vision_only` on a static belt without a PLC.
* **CLI.** `INTERPOLAR_POINTS = 4` was left over from the original 4-point packet; the CLI now
  takes `plc.interpolar_points` from the config (the `main.py --interpolar-points` flag was
  removed with it).
* **Tests.** `investigation/` was dissolved: the lasting tests moved to `tests/` (runnable with
  `python3 -m unittest discover -s tests -t .`); `test_algorithm_parity`, `virtual_cell`,
  `run_production_sim`, the sweep scripts, `REPORT*.md` and `results-*.txt`, and the root reports
  `report-fable*.md` and `doc/bao-cao-*.md` went to the local archive.
  `derive_rank_bounds` became `modules.tools.derive_rank_bounds`; it gives the same numbers as
  before, which differ from the stale values in the config (open-issues C7).
* **Verification.** Before any change, eight `production --sim` runs of 120 s (planner `spt` /
  the old `kim` × laws `constant`, `inverse_density`, `predictive_rank`, plus a wide-band variant of
  each adaptive law) were recorded; the same eight on the new code (old `kim` = `drop_longest`)
  placed, as new / before: kim + constant 60 mm/s 47/49 / 48/49, kim + inverse_density 45/49 /
  42/49, kim + predictive_rank 46/49 / 46/50, kim + predictive_rank wide 47/49 / 46/49, spt +
  constant 47/49 / 47/49, spt + inverse_density 29/48 / 32/49, spt + inverse_density wide 47/49 /
  46/49, spt + predictive_rank 35/49 / 36/49 — within the run-to-run spread of the real-time
  simulator, with no traceback and no new warning besides a start-up pose-stream notice. Speed decisions on identical inputs were identical to the decimal.
  One wide-band `predictive_rank` run settled at 90–95 mm/s where the baseline stayed at 50–70;
  repeated runs of **both** the old and the new code landed in either regime (with one part in
  the workspace the score ties and the law takes the fastest speed), so this is the law's
  bistability, not a change. The test suite (79 tests) passes.

### 2026-09-29 — Data collection on the cell: part record, `simulate_feeder`, dashboard views

Goal: collect physical-cell data comparable with the paper's simulated cells without hand
feeding. Added:
- the part record (`runtime/outcomes.py`, `basis-programming.md` §2.6), which splits misses
  into grip failures and late misses;
- the `simulate_feeder` scenario, whose seeded virtual parts ride the real belt encoder;
- the dashboard's *Cell* and *Throughput* pages, and per-run planner / speed-law / feeder
  choice on the console;
- `tools/flow_report`.

`sandbox/feeders.py` moved to `modules/core/feeders.py` so the robot and the sandbox draw
arrivals from the same functions: nothing under `modules/` may import `sandbox/`. Each feeder
now declares its rate parameter (`rate_field`). Verified on the PLC simulator:
- runs from the CLI and from the console with a changed planner, law and seed;
- a forced delay mismatch gives `miss_grip/cup_off_part`;
- 89 tests.

---

## 4. Parked ideas

Not rejected, not scheduled — recorded so they are not re-invented from scratch.

* **Web GUI dashboard v2** — live 3-D end-effector trajectory, positional-error graphs,
  sorted-item database views. The current dashboard has a 2-D animated top view and the
  per-run throughput record.
* **SQL sorting database** — `product_types` (destination per class) plus `pick_history`
  (per-pick audit trail with timestamps and status). Would also give the independent pick
  count that open issue L6 needs.
* **Queueing-theory belt control** — the inverse-density rate regulation of
  `basis-theory.md` §6 is the near-term realisation; a full Little's-Law model of the cell is
  the longer-term evolution and overlaps directly with the current research direction.
* **Suction verification** — a vacuum-pressure sensor or a post-grip vision check would make
  the exactly-once pick policy retry-capable and would fix the measurement gap in L6.
* **Jerk/acceleration-limited profile smoothing** on top of the mandatory 3-D slope
  waypoints.
