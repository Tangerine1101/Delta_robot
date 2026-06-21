# CLAUDE.md — Delta Robot Project

## 0. Language
Except the final report messages must be Vietnamese, you can freely use English for thinking and document writing for saving compute sake.

## 1. Startup Protocol

**Always read `doc/ai_context.md` first** before reading any other file in this project.
It contains the current status, command mapping, PLC data contract, and verification commands.

---

## 2. File Access Rules

### Read freely:
- `main.py`, `README.md`
- `modules/` — all `.py` files and `config.json`
- `doc/ai_context.md`, `doc/system_reference.md`, `doc/plc_programing.md`

### Read with caution:
- `doc/human_ideas.md` — read for context only; **never edit this file** unless explicitly asked by the user. It is the human team's brainstorming space.
- `doc/Manuals/*.pdf` — large files. Read only when looking for a specific hardware register or electrical specification.

### Alway write documents in English.

### Never read or edit:
- `.trash/` — legacy backups. **Do not open, recover, or reference anything from here** unless the user explicitly asks.
- `.git/`, `.venv/`, `.agents/`, `__pycache__/`, `modules/__pycache__/` — system metadata and cache. Ignore completely.

---

## 3. Code Change Rules

- **Never commit** `data.log`, `test_module.log`, or `__pycache__/` directories.
- **Never remove or reorder** fields in `SiemensSendPacket` or `SiemensReceivePacket` — the byte layout must match the PLC DB offsets exactly.
- **Never change** the default `interpolar_points` value in `config.json` without updating every downstream array that pads to that size.
- After any change to `EthernetCom.py`, `scheduler.py`, or `cli.py`, run the compile check:
  ```bash
  python3 -m py_compile main.py modules/cli.py modules/EthernetCom.py modules/image_processing.py modules/scheduler.py modules/test_module.py modules/conveyor.py modules/interface.py
  ```

---

## 4. Embedded & PLC Programming Rules

### 4.1. Byte Order (Endianness)

| Side | Byte Order |
|------|-----------|
| PC (x86/x64) | Little-Endian |
| Siemens S7-1200 | **Big-Endian** |
| Omron NX1P2 (via pylogix tags) | Handled by pylogix — no manual swap needed |

- Structs exchanged with Siemens via `snap7` **must** use `ctypes.BigEndianStructure`.
- Do **not** use `_pack_` with `BigEndianStructure` — it is unsupported by ctypes.
- If all fields are 4-byte aligned (`c_int32`, `c_float`), removing `_pack_ = 1` has no effect on layout.

### 4.2. Siemens DB Layout Contract

**DB1 (PC → PLC, 12 bytes total):**

| Offset | Python field | PLC type | Size |
|--------|-------------|----------|------|
| 0 | `CommandID` | DINT | 4 B |
| 4 | `rotate` | REAL | 4 B |
| 8 | `speed` | REAL | 4 B |

**DB2 (PLC → PC, 20 bytes total):**

| Offset | Python field | PLC type | Size |
|--------|-------------|----------|------|
| 0 | `rotate_current` | REAL | 4 B |
| 4 | `speed_current` | REAL | 4 B |
| 8 | `task_doing` | DINT | 4 B |
| 12 | `task_state` | DINT | 4 B |
| 16 | `conveyor_position` | REAL | 4 B |

- `conveyor_position` is the belt position in **cm** (the PLC accumulates and reports it pre-decoded). The PC multiplies by `conveyor.conveyor_position_scale_mm` (default 10.0) to get mm. This replaced the former raw `encoderA`/`encoderB` quadrature counts (DB2 shrank from 24 → 20 bytes).
- TIA Portal DB must have **"Optimized block access" disabled** for snap7 raw byte access to work.
- CPU must have **"Permit access with PUT/GET"** enabled under Protection & Security.

### 4.3. Omron Packet Contract

All packets to Omron tag `pc_package` must be **padded to exactly `interpolar_points` elements** (default 7):
```python
{
    "commandID": int,
    "argument_number": int,
    "argument_x": [float] * 7,
    "argument_y": [float] * 7,
    "argument_z": [float] * 7,
    "argument_e": [0 or 1] * 7,   # gripper: 0=OFF, 1=ON
    "argument_time": [float] * 7,  # segment duration in seconds — PLC firmware ignores this array; still send all 7 elements to maintain struct contract
    "bit_doing": 1,                # PC sets 1; PLC resets to 0 after ingestion
}
```
- For `goto_absolute` commands, `argument_e` must always be all zeros.
- **`argument_time` is ignored by Omron firmware** — robots run at fixed maximum speed. `nominal_xy_speed` / `nominal_z_speed` in `SchedulerSettings` are PC-side timing approximations only.
- **`task_state`** — PLC behavior is inconsistent: values 0/1/2 observed; 2 = done; no error value exists. `bit_doing` has replaced its role as the primary completion signal. All existing `task_state` references in code are intentionally kept.
- **`grab` / `place` CLI commands** — these do NOT actuate suction on the real PLC. The `pick`/`release` command IDs (5/6) are no-ops in the current PLC program; `grab`/`place` fall back to `goto_absolute` with `argument_e=0`. Known limitation — not fixed in Phase 1.

### 4.4. Safety Invariants

The scheduler enforces this height hierarchy — never write logic that violates it:
```
clearance_height > slope_transition_height > pre_pick_height > pickup_height
```
*(Z-axis is negative-down. Less negative = higher. Example: -290 > -295 > -300 > -310)*

- XY travel during slope segments (between `slope_transition_height` and `clearance_height`) is permitted — the 7-point trajectory geometry uses diagonal slopes at the pick/place endpoints.
- Coordinates outside `workspace_window_uv` (C-frame) must be **discarded**, not clamped.

### 4.5. Timing & Dispatch

- Pick dispatch must compensate for both robot movement delay (default 0.05 s) and Ethernet delay (default 0.002 s):
  ```
  t_dispatch = t_pick - t_robot_movement_delay - t_ethernet_delay
  ```
- Do not introduce `time.sleep()` on the main scheduler loop — it blocks the pick window.
- **Omron NX1P2 firmware ignores `argument_time`** and runs servos at a fixed maximum speed. PC-side `nominal_xy_speed` / `nominal_z_speed` are scheduler-side timing approximations only — they do not throttle motion. For true mechanism-speed measurement, gate the next phase on `pos_EE` convergence (see the `evaluate` scenario), not on `argument_time`. See `doc/ai_context.md` section 3.

### 4.6. Command IDs

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
}
```

---

## 5. Verification Commands

```bash
# Compile check
python3 -m py_compile main.py modules/cli.py modules/EthernetCom.py modules/image_processing.py modules/scheduler.py modules/test_module.py modules/conveyor.py modules/interface.py

# Throughput simulation
python3 main.py --scheduler --scenario test_throughput --duration 12.0 --simulate-executor

# Accuracy simulation
python3 main.py --scheduler --scenario test_accuracy --duration 5.0 --simulate-executor

# Evaluate simulation
python3 main.py --scheduler --scenario evaluate --simulate-executor --duration 10.0

# Test module dry run (fake PLC self-test)
python3 -m modules.test_module --port 1502 --self-test --duration 2.0

# Vision smoke test (requires physical camera + board on belt)
python3 -m modules.image_processing --duration 15

# Production dry-run (real vision, simulated robot)
python3 main.py --scheduler --scenario production --simulate-executor --duration 20
```

**Vision dependencies** (install once into `.venv`):
```bash
.venv/bin/pip install "ultralytics>=8.3.0" "opencv-python>=4.9.0" "PyYAML>=6.0"
```
`YOLO_OBB/` is the teammate's repo — gitignored, treated read-only.
