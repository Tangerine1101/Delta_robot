# AI Context — Delta Robot

> **Target audience**: AI coding assistants and subagents working in this repository.
> **Read this file first** (per `CLAUDE.md` §1 / `AGENTS.md` §1) before any other file.

This project is a Delta-robot pick-and-place sorting cell: a Python control PC coordinates an
Omron NX1P2 PLC (arm + suction motion) and a Siemens S7-1200 PLC (conveyor speed + 4th-DOF
rotation), with a real-time vision pipeline (YOLO-OBB) tracking parts on a moving belt.

---

## 1. Current phase — read this before proposing work

The cell is built and running. The repository is **no longer in incremental-development
mode**: it is the experimental platform for a research project on **online pick scheduling
integrated with conveyor speed control** (see `doc/proposal/`, problem formulated as
$1 \mid r_j(t),\, d_j(t),\, \text{online} \mid \sum U_j$ — each part on the belt carries a
deadline set by when it leaves the reachable window, and belt speed is treated as a
scheduling decision rather than a separate control problem).

Declared work phases:

| Phase | Scope |
|---|---|
| **NEAR** | Redesign of the pick scheduler and the conveyor speed controller |
| **MID** | Rework of the packet layout and the PLC ↔ IPC communication method |
| **FAR** | Vision-model fine-tuning, configuration slimming |

Consequences for anyone working here:

* The scheduler and speed controller documented in `basis-theory.md` §4 and §6 are the
  **baseline being replaced**, not a design to defend or extend incrementally.
* **Every unresolved problem is registered in [`open-issues.md`](open-issues.md).** Read it
  before proposing changes — several obvious-looking improvements are blocked on a
  calibration or a hardware fact listed there. It is the only file where open problems live.
* A value in `modules/config.json` being present is **not** evidence it was measured.

---

## 2. Document map

| File | What it holds | Who edits it |
|---|---|---|
| `context.md` | THIS FILE — onboarding, rules, directory map | anyone |
| [`basis-theory.md`](basis-theory.md) | The *why* of every algorithm: coordinate transforms, kinematics, trajectory profiles, tracking/interception, orientation chain, adaptive speed law | anyone |
| [`basis-programming.md`](basis-programming.md) | The *how*: process/thread architecture, PLC data contracts, trajectory templates, scenario matrix, config-key reference, verification commands | anyone |
| [`open-issues.md`](open-issues.md) | **Single register of everything unresolved** — pending calibration, config inconsistencies, architectural limits, reproducibility gaps | anyone, including AI agents |
| [`decision-log.md`](decision-log.md) | History only: superseded designs and why they were rejected, applied calibrations, repo restructuring, parked ideas | append-only |
| [`dev-note.md`](dev-note.md) | Human developer's working notes and hardware quirks | **human only — AI must not edit** |

**Rule of thumb**: descriptive documents (`basis-*`) describe only current behaviour. If you
find yourself writing "used to", "was replaced by", or "[FIXED]" in them, that text belongs in
`decision-log.md`. If you find yourself writing "TODO" or "not yet calibrated", it belongs in
`open-issues.md`.

---

## 3. Directory Structure

```
Delta_robot/
├── main.py                    # Orchestrator: CLI + scheduler entry point, IPC worker process
├── camera_calibrate.py        # Camera calibration tool (ROI, trigger line, pixels/mm)
├── calibrate_everything.py    # Whole-config consistency + workspace boundary checker
├── README.md                  # Short project overview + quickstart (doc/ is authoritative)
├── requirements.txt           # Python dependency list
├── CLAUDE.md                  # Claude developer rulebook (Claude-only)
├── AGENTS.md                  # General AI developer rulebook (non-Claude-only)
│
├── modules/                   # System core Python modules
│   ├── scheduler.py           # Real-time two-thread pick loop, trajectory generation, adaptive speed
│   ├── conveyor.py            # Coordinate transforms, tracker, encoder decoder
│   ├── EthernetCom.py         # PLC socket gateway (snap7 + pylogix)
│   ├── image_processing.py    # YOLO-OBB inference + PyAV camera capture threads
│   ├── interface.py           # In-process web dashboard (stdlib http.server + SSE)
│   ├── cli.py                 # Interactive command-line command builder/parser
│   ├── test_module.py         # Standalone fake PLC simulator (TCP socket, JSON-lines)
│   ├── latency_probe.py       # PLC round-trip latency calibration tool
│   ├── test_rotate.py         # 4th-DOF rotation probe (sign, axis speed, cmd-7 retrigger)
│   ├── rotate_sweep_sim.py    # Offline sweep of the angle chain (no hardware)
│   └── config.json            # Active system configuration (see basis-programming.md §7)
│
├── doc/                        # Documentation (see §2)
│   ├── context.md, basis-theory.md, basis-programming.md
│   ├── open-issues.md, decision-log.md, dev-note.md
│   ├── proposal/               # Research proposal / paper sources (LaTeX)
│   ├── PLC_Program_description/ # PLC Structured Text & Ladder rung-by-rung breakdowns
│   └── Manuals/                 # PLC & hardware datasheets (open only to check a register)
│
└── models/                    # Trained YOLO weights
    ├── nano@1280/              # YOLO-OBB 1280p models
    └── nano@1920/              # YOLO-OBB 1920p models (default active model)
```

**Ignored working directories** (present on disk, not part of the project): `runs/` (YOLO
training/inference output), `.report/`, `.venv/`, `__pycache__/`, and `.archive/` — a
local-only archive of the graduation thesis, the old test harness and superseded docs.
`.archive/` must never be read, referenced, or restored into the tracked tree; what moved
there and why is in `decision-log.md` §3.

---

## 4. AI Rules & Startup Protocol

1. **Rulebook selection**: if you are Claude, follow `CLAUDE.md` and ignore `AGENTS.md`; any
   other AI agent does the reverse.
2. **First step**: read this file, then `open-issues.md`.
3. **Language**: conversational replies to the user are in Vietnamese; all documentation and
   code comments are in English.

### File access rules

* Read freely: `main.py`, `README.md`, `modules/**/*.py`, `modules/config.json`, and
  `doc/*.md`.
* Read with caution: `doc/Manuals/*.pdf` (large hardware documentation — open only when
  checking a specific physical register).
* Never read or edit: `.archive/`, `.git/`, `.venv/`, `__pycache__/`, `runs/`.
* `doc/dev-note.md` is maintained by the human developer — **do not edit it** unless
  explicitly asked.
* `doc/open-issues.md` may be updated by AI agents: add a newly discovered problem, or remove
  a row when the problem is genuinely closed (and record the resolution in
  `decision-log.md`). Never leave `[FIXED]` entries behind.

### Code change rules

* **Never commit** `data.log` or other runtime log files, or `__pycache__/` directories.
* **Never remove or reorder** fields in `SiemensSendPacket` / `SiemensReceivePacket` — the
  byte layout must match the PLC DB offsets exactly (`basis-programming.md` §3.2).
* **Never change** the default `interpolar_points` value in `config.json` without updating
  every downstream array that pads to that size.
* After any change to `EthernetCom.py`, `scheduler.py`, or `cli.py`, run the compile check in
  `basis-programming.md` §8.
