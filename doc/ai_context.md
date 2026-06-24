# AI Context Summary: Delta Robot

> **Target Audience**: AI Coding Assistants, Subagents, and compact context updates during chat session resets.
> **Status (real-time pick rewrite)**: The real conveyor pick path (`production` / `test_conveyor`) was rebuilt as a **two-thread** PC-side scheduler to kill the multi-object "arrive → wait → miss" lag. A daemon **perception thread** (`_realtime_perception_loop`, ~25 ms) owns the only PLC status read and keeps shared `RealtimeState` (belt position/speed, `pos_EE`, the `BeltTracker`, claimed ids) fresh under `state_lock`; the **main decision/execution thread** selects an object (danger-zone priority), predicts a stable straight-down pick point, parks the arm with a 1.6 s lead (`intercept_lead_time_s`), then fires the pick on a **live positional gate** — when the tracked object's `u` reaches the parked pick `u` — with no post-park time math. Dispatch/status round-trips share one `ipc_lock`. New code lives in `RealtimePickExecutor` + `_run_realtime_pick_loop`; the old time-based executor/wait logic was removed. Design of record: `doc/realtime_pick_redesign.md` + `doc/rebuild_plan.md`. The simulated path (`test_throughput`/`test_accuracy`/`evaluate`) is unchanged.
> **Status**: Phase 3 image processing **rebuilt** (self-contained). `VisionImageProcessing` (YOLO-OBB + centroid tracker) runs in-process via background threads and opens a live overlay window (boxes + id/type/angle + CAM/PROC FPS + belt-speed estimate). Camera frames are captured with **PyAV** (FFmpeg-backed) — sustains **~30 fps at 1080p MJPG**; the old `cv2.VideoCapture` V4L2 backend was the real <20 fps bottleneck (not the model/GPU). The module no longer imports `YOLO_OBB/src/*` and no longer reads `system_config.yaml`: every vision parameter lives in the `vision` section of `config.json`. Default model `models/nano@1920/weights/best.pt` (mAP50-95≈0.983, per `models/nano@1920/results.csv`). `models/nano@1280/weights/best.pt` (mAP50-95≈0.986) is available as a faster, slightly lower-resolution alternative but is not the active `config.json` weight. Also computes a belt-speed estimate from object tracking (`BeltVelocityEstimator`) — **informational only**, logged/drawn but NOT fed to the scheduler. Belt position for operation still arrives pre-decoded from Siemens as a single `conveyor_position` field (**≈mm as of June 2026 → `conveyor_position_scale_mm = 1.0`**, not the old cm/×10 — the ×10 inflated belt speed ~10× and broke every `test_conveyor` pick); raw `encoderA`/`encoderB` removed. Scenarios `production`, `test_conveyor`, `test_vision_only` wire the real camera into the scheduler. Object types: `QFP` / `TQFP`.

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
│   ├── calibration.py         # Workspace and camera calibration utilities
│   ├── test_module.py         # Standalone fake PLC simulator (TCP socket JSON-lines)
│   └── config.json            # Active system configurations and parameters
│
├── doc/                       # System documentation
│   ├── ai_context.md          # THIS FILE: Consolidated AI reference & coding guide
│   ├── theory_basis.md        # Human-oriented mathematical concepts & brainstorming
│   ├── academic_report.md     # Academic mathematical derivations & kinematics archive
│   ├── rebuild_plan.md        # Design of record: real-time two-thread pick scheduler (detailed spec)
│   ├── realtime_pick_redesign.md # Design of record: real-time pick flow summary (post-implementation)
│   ├── archive/               # Superseded debugging reports (historical; NOT current reference)
│   └── PLC_Program_description/ # PLC Structured Text & Ladder breakdowns
│       ├── main_logic.md      # Rung-by-rung breakdown of main PLC program
│       ├── inverse_kinematics.md # Inverse kinematics ST program derivations
│       ├── calc_forward_kinematic.md # Forward kinematics 3-sphere intersection ST
│       ├── MC_inter_curve_vel.md # S-curve/Trapezoidal trajectory generator ST
│       ├── s_and_trapodize.md # Mathematical justification of Trapezoidal fallback
│       ├── easy_understand_talet_3d.md # LERP parametric synchronization proof
│       └── Ethercat_config.md # PDO mappings & DC synchronization details
│
├── tests/                     # Unit tests
│   └── test_trajectory_planning.py
│
├── models/                    # Trained YOLO model weights
│   ├── nano@1280/             # YOLO-OBB 1280p models
│   ├── nano@1920/             # YOLO-OBB 1920p models (Default active model)
│   └── small@1280_old_dataset/
│
└── runs/                      # YOLO validation and training plots
```

### Directories to AVOID / Ignore:
* **`.trash/`**: Contains legacy backup files. **NEVER read, edit, or reference** anything here.
* **`doc/Manuals/`**: Large PDF documentation files. Only open when checking physical registers.
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
changed is that the **real** pick path (`production` / `test_conveyor`) now runs the
scheduler itself as **two cooperating threads** sharing one guarded `RealtimeState`
(`scheduler.py:140`), instead of one blocking single-threaded loop.

1. **Communication worker** (`multiprocessing.Process`, `main.py` IPC worker): the single
   gateway for snap7 & pylogix PLC reads/writes. Every dispatch/status round-trip goes
   through it.
2. **Decision/execution — main thread** (`_run_realtime_pick_loop`, `scheduler.py:2207`):
   selects the highest-priority unclaimed object (danger-zone tier first), predicts the pick
   point + 1.6 s lead, builds goto/pick packets, claims the object, dispatches, and runs the
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
| `test_throughput` | Simulated | Sim/Real | Synthetic | Web (`--interface`) |
| `test_accuracy` | Simulated | Sim/Real | Static (None) | Web (`--interface`) |
| `evaluate` | Simulated | Sim/Real | Synthetic | Console |
| `test_vision_only` | **Real camera** | Idle (`NullExecutor`) | Siemens PLC | Web or native cv2 |
| `test_conveyor` | **Real camera** | Real (two-thread) | Siemens PLC | Web or native cv2 |
| `production` | **Real camera** | Real (two-thread) | Siemens PLC | Web or native cv2 |

> **`production` / `test_conveyor` execution model**: these run the §6.1 two-thread
> `RealtimePickExecutor` loop — danger-zone priority selection, a 1.6 s downstream park lead,
> and a **positional pick gate** (fire when the live tracked object reaches the parked pick
> `u`; no post-park arrival-time computation). `test_vision_only` keeps the perception thread
> live but uses a `NullExecutor` (no arm). The simulated scenarios above retain the original
> single-threaded harness.

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

# 10. Conveyor test (real camera + robot + Siemens conveyor_position feedback) + dashboard
python3 main.py --scheduler --scenario test_conveyor --interface --duration 30
```
