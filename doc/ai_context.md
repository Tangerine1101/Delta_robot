# AI Context Summary: Delta Robot

> **Target Audience**: AI Coding Assistants, Subagents, and compact context updates during chat session resets.
> **Status (rotate fault — root cause CONFIRMED + fixed, 2026-07-06c)**: A real hardware `production` run (`production.log`, 12 picks) was correlated `[ROTATE]`×`[GATE]`×`[DETECT]` line-by-line, superseding the four-candidate hardware-diagnosis framing below. **Root cause: not a PLC/ST retrigger issue** (the `test_rotate.py` "CommandID edge-trigger" theory was checked against the log and falsified — a pick succeeded with no `change_speed` interleaved between the two `rotate_absolute` calls, and another failed *with* one interleaved). The actual bug is on the **PC side**: `RealtimePickExecutor._wait_for_arm_arrival`'s post-grip rotate dispatch was gated on observing `pose.z` inside the narrow `contact_z+2mm` band (only ~2mm of the ~13mm pre_pick↔pickup excursion); at a 50ms pose-poll interval the fast trapezoidal dip routinely never sampled inside that band, so the `rotate_absolute` command was **never sent at all** for 6/12 real picks (`rotate_at_end_deg` was exactly `0.0`, never a partial angle — proven by exact 1:1 correlation with the missing `[GATE]` line, which shares the same observation flag). **Fixed**: (1) `modules/scheduler.py` `_wait_for_arm_arrival` now also arms on a much wider mid-point band AND has a modeled-time fallback deadline (`dispatched_at + descend_time_s + trajectory_pick[1].time_s + margin`) that force-dispatches the rotate regardless of pose-sample luck — verified with a synthetic pose-feed test (fallback fires when the poll misses the descent entirely; normal path fires via the wider band otherwise; exactly one dispatch either way). `[GATE]` gained a `contact_observed` bool (false = degraded/late timing datum from the fallback path, calibration sample kept instead of lost). (2) The post-grip angle is now **refreshed from the object's live tracked heading at the gate** (not frozen at plan-build time, which measured up to ~3.4° stale in the log), guarded by `scheduler.rotate_refresh_max_delta_deg` (default 15°) to reject outliers. (3) `modules/image_processing.py` now remembers each track's last marker-resolved heading (`_marker_angle_by_track`) and reuses it when a frame misses the marker (e.g. board exiting the ROI) instead of falling back to the OBB symmetry fold, which was jumping ~90° on marker dropout (seen in the log as `yolo-000009`: 260.79°→343.2°) — only a track that has *never* resolved a marker uses the OBB fallback. **Correction to the block below**: `scheduler.rotate_sign` in the live `modules/config.json` is `1`, not `-1` — the `-1` mentioned there was the plan at the time, not what ended up deployed; `test_rotate.py`'s visual sweep still needs to run on hardware to confirm the correct sign, this fix is orthogonal to that. Compile check + `test_throughput`/`test_accuracy` sims + a synthetic-pose unit test of the new dispatch logic pass; hardware re-run still needed to confirm `rotate_at_end_deg == rotate_cmd_deg` on ~100% of real picks post-fix.
> **Status (rotate fault investigation + radian angle convention, 2026-07-06)**: First hardware run showed boards landing at **per-object, random-looking wrong orientations**. Audit of the whole angle chain found two provable constant errors and four candidate random causes; the constant ones are fixed, the random ones are instrumented for a hardware diagnosis. **(a) Angle convention rebuilt in three layers** (the "logical [-180,180) degrees" middle layer was retired): Layer 1 — the vision marker heading (measured vs image +y DOWN on raw pixels) is converted ONCE to an **R-frame heading in radians** by `ConveyorFrame.vision_heading_to_robot_rad` (`rad(h−90°)+θ_frame`, wrapped [-π,π); the −90° fixes a constant bias the old `+θ`-only formula had). Layer 2 — scheduler works in R-frame radians only (`TrackedObject.rotation_rad`, `PickPlan.rotate_rad`); post-grip command `θ_cmd = wrap_π(rotate_sign·(rotate_offset − φ_board))`; wrap = shortest way. Layer 3 — IPC boundary (`main.py` worker) converts cmd-7 packets to wire degrees via `robot_rad_to_wire_deg`/`wire_deg_to_robot_rad` (`EthernetCom.py`; replaced `logical_to_physical_rotate`/`physical_to_logical_rotate`); status feedback exposed in R-frame degrees. **CORRECTED 2026-07-06b — Layer 3 is now VERBATIM (no wrap):** the Siemens axis accepts signed `[-360,360]`, shares the R-frame zero, and its command value encodes **spin direction as well as position**, so the boundary is a plain radians→degrees identity clamped to `[-359,359]` (ST misbehaves at exactly ±360). CLI proof of the old fault: `rotate 180`→PLC reported `-180`, `rotate 270`→`-90` because `robot_rad_to_wire_deg` still called `wrap_rad` (180→-180, 270→-90); the sign flip at ±180 made a `179°→180°` step spin the axis the long way. Wrapping REMOVED here — the minimal-turn wrap stays ONCE upstream in the scheduler (relative to the homed 0), so production (always from 0) still gets the short way while manual/CLI absolute angles pass through untouched (`rotate 270`→270). (History: a prior `[0,360)`+180-shift revision drove "home to 0" to physical 180° — also removed.) `rotate_sign` `-1`, `oblique_descent_enabled` `false` per the user. CLI `rotate` still takes degrees (converted at parse). **(b) Per-class marker offset** `vision.orientation.offset_by_class` (the board→marker vector includes the marker's diagonal placement — ≈45° on a square QFP, different on TQFP; the global `offset_deg` could not capture both). **(c) New config** `scheduler.rotate_sign` (±1, physical axis direction vs R-frame CCW — unknown until probed) and `scheduler.rotate_home_tolerance_deg` (>0 ⇒ warn-only check that the cup is back at 0 when the pick fires). **(d) Diagnostics**: new **`python3 -m modules.test_rotate`** (Siemens-only probe: sweep 0/±90/±170 for remap+settle+implied axis speed, VISUAL direction check for rotate_sign, and a cmd-7→cmd-7 **retrigger test** — DB1 has no handshake; if ST edge-triggers on CommandID change, production's post-grip rotate is randomly dropped unless a change_speed interleaves — this is candidate random-cause #1); per-pick **`[ROTATE]` log** (`vision_angle/board_heading/rotate_cmd/rotate_at_gate/rotate_at_end`) — `rotate_at_gate`≉0 ⇒ home-to-0 unfinished at grip (candidate #2, axis too slow); `rotate_at_end`≉`rotate_cmd` ⇒ released mid-rotation (candidate #3); candidate #4 is an inverted axis (error = 2×heading, fixed by `rotate_sign: -1` after the probe). `RealtimeState.rotate_current_deg` now carries live Siemens feedback into the perception state. Calibration order: probe → rotate_sign → one run reading [ROTATE] → set `rotate_offset_deg` + `offset_by_class`. Compile + angle unit checks + throughput/accuracy sims pass.
> **Status (pick-accuracy + calibration overhaul, 2026-07-04)**: Four coupled fixes to the real pick path. **(1) Camera-latency compensation:** vision frames were anchored to the belt position at *ingest* time though captured ~80–150 ms earlier (exposure+YOLO+poll), so the tracker believed objects were upstream by `v·Δt`. Now the capture thread stamps each frame's decode time (`image_processing.py` `_capture_loop`/`_emit_detections`, backdated by ½ `exposure_time_absolute`), `BeltPositionTracker` keeps a `(t, position)` ring buffer with `position_at(t)` (interp), and `PickScheduler.ingest_detections(..., position_at=)` anchors each detection to the belt position **at its capture instant** (falls back to `p_now`; no-op for the static/simulated belt). **(2) Oblique descent (opt-in `scheduler.oblique_descent_enabled`, default OFF):** the pre_pick→pickup drop was vertical at a fixed R-frame XY, so contact happened with horizontal relative velocity = belt speed. **The goto/park is unchanged** (arm parks above the predicted in-workspace point `pick_position`, at pre_pick height); only the **pick-phase contact** is shifted **downstream** by `v·t_d` along `+u_hat` (`ConveyorFrame.u_hat`, `_contact_position`/`_descent_time_s`), so the short descent slants to track the object. Default OFF = vertical descent (contact == pick_position). Gate lead is unchanged (`command_delay + gate_sampling`, NOT `+t_d`); `[GATE]` log still prints `t_d_model_s`. **NOTE:** an earlier revision parked the arm *upstream* by `v·t_d` (moved the goto) — that flew the arm OUTSIDE the workspace at belt speed and was reverted 2026-07-05; the shift is on the contact (downstream), never the park. **(3) Rotation timeline:** `rotate_absolute` used to fire *between* gate-fire and pick dispatch (uncompensated round-trip on the critical path). New order: home the axis to logical 0 during goto flight → gate → pick → **after grip, once the arm lifts back to `z ≥ pre_pick`, rotate the attached board to normalise it to the bin orientation** (**angle formula + "logical degrees" convention superseded 2026-07-06 — see the top status block: the chain is now vision → R-frame radians → wire degrees, with a −90° convention fix, `rotate_sign`, and per-class marker offsets**). **(4) Calibration tooling:** new `modules/latency_probe.py` (`python3 -m modules.latency_probe [--target omron|siemens|both]`) times each PLC's status read separately (the IPC worker reads Omron then Siemens *sequentially*, so the old ~25 ms "round trip" was the sum; only Omron gates a pick) and suggests `ethernet_delay_s`; and a `[CONFIG-SUGGEST]` block at accuracy/acceptance scenario end (`EvaluateMetrics.config_suggestions`) reports `robot_movement_delay_s` = mean(measured wall − interpolator-modeled motion), `ethernet_delay_s` = round-trip avg, and measured vs configured XY speed. Compile + sim (throughput/accuracy) + realtime-plan + mock-PLC/probe smoke tests pass; the RealtimePickExecutor timeline itself needs hardware to exercise.
> **Status (real-time pick rewrite)**: The real conveyor pick path (`production`) was rebuilt as a **two-thread** PC-side scheduler to kill the multi-object "arrive → wait → miss" lag. A daemon **perception thread** (`_realtime_perception_loop`, ~25 ms) owns the only PLC status read and keeps shared `RealtimeState` (belt position/speed, `pos_EE`, the `BeltTracker`, claimed ids) fresh under `state_lock`; the **main decision/execution thread** selects an object (danger-zone priority), predicts a stable straight-down pick point, parks the arm with a 0.8 s lead (`intercept_lead_time_s`), then fires the pick on a **live positional gate** — when the tracked object's `u` reaches the parked pick `u` — with no post-park time math. Dispatch/status round-trips share one `ipc_lock`. New code lives in `RealtimePickExecutor` + `_run_realtime_pick_loop`; the old time-based executor/wait logic was removed. Design of record: `doc/archive/realtime_pick_redesign.md` + `doc/archive/rebuild_plan.md`. The simulated path (`test_throughput`/`test_accuracy`/`test_acceptance`/`evaluate`) is unchanged.
> **Status (test scenario standardization, 2026-06-25)**: `test_conveyor` **retired** — it was behaviorally identical to `production` (the only difference, a startup `change_speed`, already fires for `production` too when `adaptive_speed_enabled`). `test_accuracy` now: (a) never commands suction (`PickScheduler.scenario_name` gates the pick-phase gripper bits to all-zero in `_build_pick_plan`) since its objects are static fakes with no real board to grip; (b) spawns a full **wave** of `accuracy_spawn_uv` points at once and blocks further spawns until the previous wave's picks all finish (`SimulatedImageProcessing._wave_pending` + `notify_pick_finished`), instead of a fixed timer decoupled from pick completion; (c) on real hardware (no `--simulate-executor`) now runs on `EvaluateExecutor` (the same dispatch-and-wait-for-pos_EE-convergence backend `evaluate` uses) instead of `RealtimePickExecutor` — the latter unconditionally required a `RealtimeState` the single-thread loop never built, so real-hardware `test_accuracy` **crashed on the first pick** before this fix; a belt-gate design was also the wrong fit for static objects anyway. New scenario **`test_acceptance`** (same family as `test_accuracy`) runs exactly `scheduler.test_acceptance_cycles` (default 9) picks then stops, printing per-phase `[ACCEPT]` wall-time/distance (goto and pick measured separately, dispatch→settle, via `EvaluateExecutor.metrics`) and a final `[ACCEPT-SUMMARY]`. `test_vision_only`'s belt-speed-not-visible bug was **dead code, not a design gap**: `main.py` had an early-return that called `run_scheduler_scenario` without `executor=`, skipping the already-correct `NullExecutor(dispatch=..., request_status=...)` construction further down — deleted the dead path.
> **Status (adaptive belt speed, revised 2026-07-02)**: Implemented, **opt-in** via `scheduler.adaptive_speed_enabled` (was disabled after hardware testing showed uneven/laggy belt; the 2026-07-02 rework below addresses that, and it is being re-enabled for calibration). Density `N` is sensed **continuously by the perception thread** every ~25 ms tick (doc/theory_basis.md §6 — belt speed **inverse** to density N, `v = clamp(λ_nom·L_meas/N, v_min, v_cap)`). **Spacing cap (added 2026-07-02, §6.5):** the count-only law regulates *average* rate but is blind to clustering — a tight pair and a spread pair both read N=2 and got the same (too-fast) speed, so a tight trailing object passed `u_max` unpicked. A **cluster ceiling** `v ≤ g_min/pick_cycle_s` (`_spacing_speed_cap`; `g_min` = min adjacent gap over the leading `_SPACING_LEAD_OBJECTS`=4 objects) is now `min`-ed with the density speed and clamped to `v_min`; only the *imminent* cluster constrains the belt. `belt_speed_max_mm_s` (new config key) is a soft operational cap on `v_cap`, under the `belt_speed_hw_max_mm_s` safety limit. **Commit policy inverted (was the root cause of the laggy feel):** grip-instant-only commits gave ≤1 commit per 2–10 s pick cycle; `_commit_adaptive_speed` is now called **opportunistically** from the executor wait loops (goto flight, far gate-wait, post-grip return) and the idle loop, throttled ≥0.75 s apart, and suppressed only inside the **gate-critical window** (`RealtimeState.gate_critical`: object within ~2 s belt travel of the fire threshold). Each commit is **step-limited** to `belt_speed_max_step_mm_s` (default 20 → ramp ≤0.9 s at a_nom=22.31 mm/s², vs the 2.4 s ramp of the raw N=1↔2 54 mm/s jump) and **closed-loop**: perception stores PLC `speed_current` (`state.belt_speed_measured_mm_s`); if measured diverges from setpoint >3 s after the last commit the setpoint is re-sent. The startup `change_speed` seed (`test_conveyor_belt_speed_mm_s`, now an explicit config key) is sent **unconditionally** (adaptive off = static belt speed). Gate lead offset = `v·(T_delay + sampling latency ≈37 ms)`; every pick prints a `[GATE]` line (`gate_to_dispatch_s`/`dispatch_to_contact_s`) to calibrate `robot_movement_delay_s` — the configured 10 ms is far below the physical dispatch→contact chain (~0.3–0.5 s), a pending calibration item. Other keys under `scheduler`: `pick_cycle_s`, `pick_transit_min_s` (→ `v_cap=L/t_transit`), `belt_speed_headroom`, `belt_speed_min_mm_s`, `belt_speed_hw_max_mm_s`, `belt_speed_deadband_mm_s`, `belt_density_length_mm`, `belt_accel_mm_s2`/`belt_ramp_s` (informational); `conveyor.velocity_ema_alpha` now wires the BeltPositionTracker velocity EMA. **Open hardware question:** Siemens DB1 has no handshake — verify in TIA how the ST program latches CommandID (back-to-back `rotate_absolute` → `change_speed` writes ~tens of ms apart could drop/abort the rotation; document in doc/PLC_Program_description/).
> **Status (phantom re-pick fix; scope revised 2026-07-02)**: There was never intentional slip/miss-retry logic — `execute()` reports success once the arm's *motion* finishes; suction is never verified. A gap meant a completed pick's object stayed in `scheduler.tracker` (only `claimed_object_ids`, cleared right after `execute()`, excluded it), so it could be re-selected next plan build — "pick plan targets a board that no longer exists." **Fixed 2026-06-25**: post-`execute()` bookkeeping removes the object from the tracker, plus a defensive `planned_object_ids` skip in `_build_realtime_pick_plan`'s candidate loop. **Scope narrowed 2026-07-02**: exactly-once now keys off `plan.debug_info["pick_dispatched"]` — only a pick whose grip was actually commanded is removed; a **pre-grip abort** (goto timeout / gate stall / track lost) just unclaims and leaves the object tracked for re-planning (the old blanket removal permanently dropped still-pickable objects and deflated density N, telling the belt to speed up right after a failure). The gate abort itself is no longer a fixed wall-clock deadline (`predicted_pick_time + margin` fired spuriously whenever the belt slowed after plan-build) but a **progress-based stall check** (`pick_object_stalled`: object u advances <0.5 mm for ~3 s / track gone). See doc/theory_basis.md §6.8.
> **Status**: Phase 3 image processing **rebuilt** (self-contained). `VisionImageProcessing` (YOLO-OBB + centroid tracker) runs in-process via background threads and opens a live overlay window (boxes + id/type/angle + CAM/PROC FPS + belt-speed estimate). Camera frames are captured with **PyAV** (FFmpeg-backed) — sustains **~30 fps at 1080p MJPG**; the old `cv2.VideoCapture` V4L2 backend was the real <20 fps bottleneck (not the model/GPU). The module no longer imports `YOLO_OBB/src/*` and no longer reads `system_config.yaml`: every vision parameter lives in the `vision` section of `config.json`. Default model `models/nano@1920/weights/best.pt` (mAP50-95≈0.983, per `models/nano@1920/results.csv`). `models/nano@1280/weights/best.pt` (mAP50-95≈0.986) is available as a faster, slightly lower-resolution alternative but is not the active `config.json` weight. Also computes a belt-speed estimate from object tracking (`BeltVelocityEstimator`) — **informational only**, logged/drawn but NOT fed to the scheduler. Belt position for operation still arrives pre-decoded from Siemens as a single `conveyor_position` field (**≈mm as of June 2026 → `conveyor_position_scale_mm = 1.0`**, not the old cm/×10 — the ×10 inflated belt speed ~10× and broke every `test_conveyor` pick); raw `encoderA`/`encoderB` removed. Scenarios `production`, `test_vision_only` wire the real camera into the scheduler. Object types: `QFP` / `TQFP`.

---

## 1. AI Rules & Startup Protocol

1. **Rulebook Selection (to avoid duplication)**:
   * **If you are Claude** (using Claude Code tool): Read and follow [CLAUDE.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/CLAUDE.md) in the root. **Ignore** `agents.md`.
   * **If you are not Claude** (using a different AI agent): Read and follow [agents.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/agents.md) in the root. **Ignore** `CLAUDE.md`.
2. **First step**: Always read this file `doc/ai_context.md` at startup.

---

## 2. Codebase Structure & Directory Guidelines

Below is the directory tree of the repository with the description of each file/folder:

```
Delta_robot/
├── main.py                    # Primary orchestrator for CLI and scheduler modes
├── camera_calibrate.py        # Camera calibration tool (ROI, trigger line, pixels/mm)
├── calibrate_everything.py    # Boundaries checker & safety validator tool
├── calib_result.jpg           # Camera calibration visual result artifact
├── README.md                  # Quickstart and repository entry overview
├── requirements.txt           # Python dependency list
├── CLAUDE.md                  # Claude developer rulebook (Claude-only)
├── agents.md                  # General AI developer rulebook (non-Claude-only)
├── data.log                   # Active runtime performance log (gitignored)
├── test_conveyor.log          # Conveyor tracking test log (gitignored)
├── test_module.log            # Mock PLC server log (gitignored)
│
├── modules/                   # System core python modules
│   ├── scheduler.py           # Real-time two-thread pick loop (RealtimePickExecutor), trajectory generation, speed estimator
│   ├── conveyor.py            # Coordinate transformations, tracker, encoder decoder
│   ├── EthernetCom.py         # PLC socket gateway (snap7 + pylogix)
│   ├── image_processing.py    # YOLO-OBB inference and PyAV camera capture threads
│   ├── interface.py           # In-process Web Dashboard (stdlib http.server + SSE)
│   ├── cli.py                 # Interactive command-line command builder and parser
│   ├── test_module.py         # Standalone fake PLC simulator (TCP socket JSON-lines)
│   └── config.json            # Active system configurations and parameters
│
├── doc/                       # System documentation
│   ├── ai_context.md          # THIS FILE: Consolidated AI reference & coding guide
│   ├── theory_basis.md        # Human-oriented mathematical concepts & brainstorming
│   ├── academic_report.md     # Academic mathematical derivations & kinematics archive
│   ├── Yolo_training_report.md # ⚠ UNTRUSTWORTHY: written against an OUTDATED repo. Do NOT cite as ground truth (e.g. it stated the OBB angle range as [-45,135) but the code folds to [-90,90)). Code/config win on any conflict; use only as loose background.
│   ├── Manuals/                # PLC & hardware datasheets (OMRON NX1P2, Siemens S7-1200, brochures)
│   ├── archive/                # Superseded design docs/debugging reports (historical; NOT current reference)
│   │   └── report_draft_v1/    # Old failed Markdown→docx thesis attempt, superseded by report/ (LaTeX)
│   └── PLC_Program_description/ # PLC Structured Text & Ladder breakdowns
│       ├── main_logic.md      # Rung-by-rung breakdown of main PLC program
│       ├── inverse_kinematics.md # Inverse kinematics ST program derivations
│       ├── calc_forward_kinematic.md # Forward kinematics 3-sphere intersection ST
│       ├── MC_inter_curve_vel.md # S-curve/Trapezoidal trajectory generator ST
│       ├── s_and_trapodize.md # Mathematical justification of Trapezoidal fallback
│       ├── easy_understand_talet_3d.md # LERP parametric synchronization proof
│       └── Ethercat_config.md # PDO mappings & DC synchronization details
│
├── report/                    # LaTeX graduation thesis resources (see report/README.md)
│   ├── README.md               # Describes the layout below; current completeness status
│   ├── main.tex                # Main thesis entry point (skeleton; \input's tex/ sections)
│   ├── diagram/                 # Mermaid (.mmd) flowchart sources + rendered .png/.svg/.pdf
│   ├── img/                    # General photos (hardware, workspace) — currently empty
│   ├── src/                    # Raw mixed-format material (pdf/docx/md) to mine for prose — currently empty
│   ├── ref/                     # Academic references: textbooks, papers, FAE thesis template PDF
│   └── tex/                     # Individual .tex section files included by main.tex — currently empty
│
├── tests/                     # Unit tests
│   └── test_trajectory_planning.py
│
├── models/                    # Trained YOLO model weights
│   ├── nano@1280/             # YOLO-OBB 1280p models
│   ├── nano@1920/             # YOLO-OBB 1920p models (Default active model)
│   └── small@1280_old_dataset/
```

### Directories to AVOID / Ignore:
* **`.trash/`**: Contains legacy backup files. **NEVER read, edit, or reference** anything here.
* **`doc/Manuals/`**: Large PDF documentation files. Only open when checking physical registers.
* **`doc/archive/`**: Superseded/historical material. Not current reference.
* **`.git/`, `.venv/`, `.agents/`, `__pycache__/`, `modules/__pycache__/`**: System metadata and Python caches. **IGNORE**.

---

## 3. Technical Specs & PLC Data Contracts

### 3.1. Byte Order (Endianness)

| Side | Byte Order |
|------|-----------|
| PC (x86/x64) | Little-Endian |
| Siemens S7-1200 | **Big-Endian** |
| Omron NX1P2 (via pylogix tags) | Handled by pylogix — no manual swap needed |

* Structs exchanged with Siemens via `snap7` **must** use `ctypes.BigEndianStructure`.
* Do **not** use `_pack_` with `BigEndianStructure` — it is unsupported by ctypes.
* If all fields are 4-byte aligned (`c_int32`, `c_float`), removing `_pack_ = 1` has no effect on layout.

### 3.2. PC ↔ Siemens S7-1200 DB Contracts
PC communicates with Siemens PLC via `snap7` Modbus/TCP. 
* CPU must have **PUT/GET communication enabled**.
* Data Blocks (DB1, DB2) in TIA Portal must have **"Optimized block access" disabled** (to expose physical byte offsets).

**DB1 (PC → PLC, 12 bytes total):**
| Offset | Python field | PLC type | Size | Description |
|--------|-------------|----------|------|-------------|
| 0 | `CommandID` | DINT | 4 B | Command ID |
| 4 | `rotate` | REAL | 4 B | Absolute rotation angle (4th DOF) |
| 8 | `speed` | REAL | 4 B | Requested conveyor speed (mm/s) |

**DB2 (PLC → PC, 20 bytes total):**
| Offset | Python field | PLC type | Size | Description |
|--------|-------------|----------|------|-------------|
| 0 | `rotate_current` | REAL | 4 B | Current suction cup rotation angle |
| 4 | `speed_current` | REAL | 4 B | Current conveyor belt speed (mm/s) |
| 8 | `task_doing` | DINT | 4 B | Command ID currently executing |
| 12 | `task_state` | DINT | 4 B | Inconsistent/Legacy status (Use `bit_doing` instead) |
| 16 | `conveyor_position` | REAL | 4 B | Pre-decoded belt position in mm (`scale = 1.0`) |

### 3.3. PC → Omron NX1P2 Packet Contract
PC writes to Omron global tag `pc_package` (handled via `pylogix` EtherNet/IP):
* Struct array length must be **padded to exactly `interpolar_points` elements** (default 7).
* Value layout:
  ```python
  {
      "commandID": int,
      "argument_number": int,
      "argument_x": [float] * 7,
      "argument_y": [float] * 7,
      "argument_z": [float] * 7,
      "argument_e": [byte] * 7,     # Gripper: 0 = OFF, 1 = ON
      "argument_time": [float] * 7,  # Segment duration (seconds). Ignored by PLC
      "bit_doing": byte              # Handshake (PC writes 1, PLC resets to 0)
  }
  ```
* For `goto_absolute` commands, `argument_e` must be all zeros.
* Omron NX1P2 firmware **ignores** `argument_time` (motors run at fixed maximum speed). PC-side values are only scheduler approximations.

### 3.4. Command ID Mapping (COMMAND_ID)
```python
COMMAND_ID = {
    "stop": 0,          # Omron + Siemens
    "goto_relative": 1, # Omron
    "goto_absolute": 2, # Omron
    "go_trajectory": 3, # Omron
    "calibrate": 4,     # Omron
    "pick": 5,          # Omron
    "release": 6,       # Omron
    "rotate_absolute": 7,  # Siemens (4th DOF suction cup)
    "change_speed": 8,     # Siemens (conveyor speed)
    "plan_siemen": 9,      # Siemens (planning)
    "enable": 10,          # Omron
}
```

---

## 4. Trajectory Templates & Safety Invariants

### 4.1. The 7-Point Trajectory Template
Every pick-and-place operation consists of two sequential phases, each using a 7-point template aligned with the Omron packet layout.

```
       B_goto (Clearance) ── diagonal 3D slope ──> C_goto (Slope transition)
              ▲                                              │
              │                                              ▼
        A_goto/start                                 D_goto (Pre-pick)
                                                             │
                                                             ▼
                                                      A_pick (Suction ON)
                                                             │
                                                             ▼
       C_pick (Clearance) <── diagonal 3D slope ─── B_pick (Slope transition)
              │
              ▼
       D_pick/place (Release)
```

1. **Goto Trajectory (Move to Pre-Pick)**:
   * **A (Start)**: Current robot resting position. Gripper: `0` (OFF).
   * **B (Lift)**: Vertical lift to `clearance_height`. Gripper: `0` (OFF).
   * **C (Clearance Blend)**: Safe horizontal shift toward target. Gripper: `0` (OFF).
   * **D, E (Clearance Cruise)**: Traversal at clearance height. Gripper: `0` (OFF).
   * **F (Slope Transition)**: Angle downward approaching target. Gripper: `0` (OFF).
   * **G (Pre-pick)**: Standby position directly above moving item. Gripper: `0` (OFF).
2. **Pick Trajectory (Pick & Sort)**:
   * **P1 (Intercept)**: Descent to `pickup_height` at interception coordinate. Gripper: `1` (ON).
   * **P2 (Lift)**: Vertical lift to `slope_transition_height`. Gripper: `1` (ON).
   * **P3-P6 (Transfer)**: 3D sloped translation toward destination sorting bin. Gripper: `1` (ON).
   * **P7 (Place)**: Final descent to `place_height` at sorting bin. Gripper: `0` (OFF/Release).

### 4.2. Safety Heights & Workspace Boundaries
* **Height hierarchy**: `clearance_height > slope_transition_height > pre_pick_height > pickup_height` must hold.
* **Workspace Window**: Bounded by C-frame `conveyor.workspace_window_uv = [u_min, u_max, v_min, v_max]`. Outer coordinates are discarded.
* **Physical reach boundary**: A circle of radius `limit_radius_xy` (default 180.0 mm) around the robot origin `(0, 0)`. Checks are executed at the `PLCGateway.send_package` choke; violations raise a `WorkspaceLimitError` and reject the command.

---

## 5. Camera & Exposure Control (Manual Exposure)

Webcam auto-exposure dynamically changes exposure time based on ambient lighting, causing the FPS to drop below 15. This introduces motion blur and breaks object tracking. To maintain a constant **30 FPS**, manual exposure must be configured.

### 5.1. Target Properties & API Sequence
Always disable auto-exposure **before** writing the manual exposure value.
* **Windows (DirectShow)**: `cv2.CAP_PROP_AUTO_EXPOSURE` is set to `1` (manual). `cv2.CAP_PROP_EXPOSURE` is a log2 value (e.g., `-6` is ~15ms, ideal for 30 FPS).
* **Linux (V4L2)**: `cv2.CAP_PROP_AUTO_EXPOSURE` is set to `1` (manual). `cv2.CAP_PROP_EXPOSURE` is in microseconds (e.g., `10000` is 10ms).

*Note: The project uses **PyAV** in `image_processing.py` to capture frames, bypassing OpenCV's slow V4L2 backend and sustaining a stable ~30 FPS at 1080p.*

---

## 6. Software Threading & Scenarios Reference

### 6.1. Threading Layout

The communication worker, perception capture process, and dashboard are unchanged. What
changed is that the **real** pick path (`production`) now runs the
scheduler itself as **two cooperating threads** sharing one guarded `RealtimeState`
(`scheduler.py:140`), instead of one blocking single-threaded loop.

1. **Communication worker** (`multiprocessing.Process`, `main.py` IPC worker): the single
   gateway for snap7 & pylogix PLC reads/writes. Every dispatch/status round-trip goes
   through it.
2. **Decision/execution — main thread** (`_run_realtime_pick_loop`, `scheduler.py:2207`):
   selects the highest-priority unclaimed object (danger-zone tier first), predicts the pick
   point + 0.8 s lead, builds goto/pick packets, claims the object, dispatches, and runs the
   `RealtimePickExecutor`. Its wait loops (`_wait_for_arm_arrival` `:751`,
   `_wait_for_object_arrival` `:800`) read arm/object state **from shared memory and issue no
   PLC I/O of their own**.
3. **Perception/state — daemon thread** (`_realtime_perception_loop`, `scheduler.py:2362`):
   loops at **~25 ms**, performs the **only** regular PLC status read, updates belt
   position/speed and `pos_EE`, polls vision, refreshes the `BeltTracker` (pruning only
   *unclaimed* stale objects), and emits dashboard/`[SPEED]`/`[DETECT]` events. Never renders
   the OpenCV GUI.
4. **Perception capture** (`image_processing.py` background threads): PyAV frame ingest +
   YOLO-OBB inference, consumed by thread 3.
5. **User Interface Dashboard** (`interface.py`): serves remote MJPEG video and telemetry
   over SSE.

**Two PC-side locks** (`scheduler.py:143-144`):
- `ipc_lock` — wraps **every** dispatch/status round-trip so exactly one is in flight (the
  IPC worker drains and discards mismatched responses, so concurrent callers would eat each
  other's replies).
- `state_lock` — guards `RealtimeState` (belt/pose snapshot, `BeltTracker` mutation, the
  `claimed_object_ids` set) between the perception thread (ingest/prune) and the main thread
  (read/claim).

### 6.2. Scenario Matrix
All scenarios are executed via `main.py --scheduler --scenario <name>`.

| Scenario | Vision | Robot | Conveyor Speed | Display |
|---|---|---|---|---|
| `test_throughput` | Simulated | Sim/Real (`RealtimePickExecutor`) | Synthetic | Web (`--interface`) |
| `test_accuracy` | Simulated | Sim/Real (`EvaluateExecutor`) | Static (None) | Web (`--interface`) |
| `test_acceptance` | Simulated | Sim/Real (`EvaluateExecutor`) | Static (None) | Web + console `[ACCEPT]`/`[ACCEPT-SUMMARY]` |
| `evaluate` | Simulated | Sim/Real (`EvaluateExecutor`) | Synthetic | Console |
| `test_vision_only` | **Real camera** | Idle (`NullExecutor`) | Siemens PLC | Web or native cv2 |
| `production` | **Real camera** | Real (two-thread) | Siemens PLC | Web or native cv2 |

> **`production` execution model**: runs the §6.1 two-thread `RealtimePickExecutor` loop —
> danger-zone priority selection, a 0.8 s downstream park lead, and a **positional pick gate**
> (fire when the live tracked object reaches the parked pick `u`; no post-park arrival-time
> computation). `test_vision_only` keeps the perception thread live but uses a `NullExecutor`
> (no arm). `test_accuracy`/`test_acceptance`/`evaluate` use `EvaluateExecutor` instead — no
> belt-gate (their targets are static, not a moving tracked object): dispatch a phase, poll
> `pos_EE` until it converges, record the wall time. All of these (plus `test_throughput`) run
> on the original single-threaded harness, not the two-thread realtime loop. `test_conveyor`
> was retired 2026-06-25 (behaviorally identical to `production`); use `production`.

---

## 7. Verification & Testing Commands

```bash
# 1. Compile check all python files
python3 -m py_compile main.py modules/cli.py modules/EthernetCom.py modules/image_processing.py modules/scheduler.py modules/test_module.py modules/conveyor.py modules/interface.py

# 2. Run scheduler simulation throughput scenario
python3 main.py --scheduler --scenario test_throughput --duration 12.0 --simulate-executor

# 3. Run scheduler simulation accuracy scenario
python3 main.py --scheduler --scenario test_accuracy --duration 5.0 --simulate-executor

# 4. Verify test module logic with a dry run
python3 -m modules.test_module --port 1502 --self-test --duration 2.0

# 5. Run evaluate scenario (continuous box <-> 3 accuracy_points; Ctrl-C to stop).
python3 main.py --scheduler --scenario evaluate --simulate-executor --duration 10.0

# 6. Vision smoke test + overlay window (requires physical camera)
python3 -m modules.image_processing                        # runs until q / Ctrl-C

# 7. Production dry-run (real vision, simulated robot)
python3 main.py --scheduler --scenario production --simulate-executor --duration 20

# 8. Web dashboard smoke test (no hardware — synthetic events at http://localhost:8000)
python3 -m modules.interface

# 9. Vision-only (real camera, no robot) + live web dashboard (annotated MJPEG + data)
python3 main.py --scheduler --scenario test_vision_only --interface --duration 20

# 10. Real-hardware acceptance run: exactly test_acceptance_cycles picks (default 9), then
#     stops on its own and prints a final [ACCEPT-SUMMARY] (per-phase goto/pick wall times).
python3 main.py --scheduler --scenario test_acceptance --interface
```
