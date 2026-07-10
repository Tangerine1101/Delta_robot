# AI Context — Delta Robot

> **Target audience**: AI coding assistants and subagents working in this repository.
> **Read this file first** (per `CLAUDE.md` §0 / `AGENTS.md` §1) before any other file in the
> project. It replaces the old `doc/ai_context.md`.

This project is a Delta-robot pick-and-place sorting cell: a Python control PC coordinates an
Omron NX1P2 PLC (arm + suction motion) and a Siemens S7-1200 PLC (conveyor speed + 4th-DOF
rotation) with a real-time vision pipeline (YOLO-OBB) tracking parts on the belt. The academic
thesis (bảo vệ xong, kết quả xuất sắc) lives outside this working tree now — see §4.

For the *why* behind every algorithm (coordinate transforms, kinematics, tracking, adaptive
speed, rotation chain), read **[`basis-theory.md`](basis-theory.md)**. For *how the program is
wired* (concurrency, PLC contracts, trajectory templates, scenarios, config keys), read
**[`basis-programming.md`](basis-programming.md)**. For open calibration items and
developer-maintained notes, read **[`dev-note.md`](dev-note.md)**.

---

## 1. Directory Structure

```
Delta_robot/
├── main.py                    # Orchestrator: CLI + scheduler entry point, IPC worker process
├── camera_calibrate.py        # Camera calibration tool (ROI, trigger line, pixels/mm)
├── calibrate_everything.py    # Whole-config consistency + workspace boundary checker
├── README.md                  # Quickstart (may lag basis-programming.md — that file wins)
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
│   └── config.json            # Active system configuration (see basis-programming.md §7)
│
├── doc/                        # Documentation (4-file standard, see below)
│   ├── context.md              # THIS FILE
│   ├── basis-theory.md         # Theoretical framework for every algorithm
│   ├── basis-programming.md    # Program architecture, PLC contracts, scenarios, config keys
│   ├── dev-note.md             # Developer-maintained notes (AI does not edit unless asked)
│   ├── PLC_Program_description/ # PLC Structured Text & Ladder rung-by-rung breakdowns (kept as-is)
│   └── Manuals/                 # PLC & hardware datasheets (kept as-is; open only when checking registers)
│
└── models/                    # Trained YOLO weights
    ├── nano@1280/              # YOLO-OBB 1280p models
    └── nano@1920/               # YOLO-OBB 1920p models (default active model)
```

`.archive/` (local-only, gitignored, **not tracked in git or in this tree listing**) holds the
graduation thesis (`report/`), the old test harness (`tests/`), superseded documentation
sources, legacy backups, and other material kept for reference but out of the active repo.
Never read or reference it in code-facing work; see `dev-note.md` if you need the history.

---

## 2. AI Rules & Startup Protocol

1. **Rulebook selection**: if you are Claude, read and follow `CLAUDE.md` and ignore
   `AGENTS.md`; any other AI agent does the reverse.
2. **First step**: always read this file at startup.
3. **Language**: conversational replies to the user are in Vietnamese; the (now archived)
   thesis was written in English; working/code documents may be English to save compute.

### File access rules

* Read freely: `main.py`, `README.md`, `modules/**/*.py`, `modules/config.json`,
  `doc/context.md`, `doc/basis-theory.md`, `doc/basis-programming.md`.
* Read with caution: `doc/Manuals/*.pdf` (large hardware documentation — open only when
  checking a specific physical register).
* Never read or edit: `.archive/` (legacy/thesis material), `.git/`, `.venv/`,
  `__pycache__/`, `modules/__pycache__/`.
* `doc/dev-note.md` is maintained by the human developer — do not edit it unless explicitly
  asked to.
* Always write documentation in English.

### Code change rules

* **Never commit** `data.log` or other runtime log files, or `__pycache__/` directories.
* **Never remove or reorder** fields in `SiemensSendPacket` / `SiemensReceivePacket` — the
  byte layout must match the PLC DB offsets exactly (`basis-programming.md` §3.2).
* **Never change** the default `interpolar_points` value in `config.json` without updating
  every downstream array that pads to that size.
* After any change to `EthernetCom.py`, `scheduler.py`, or `cli.py`, run the compile check in
  `basis-programming.md` §8.

---

## 3. Current Status

The thesis defense is complete (kết quả xuất sắc). The repository is now in **post-thesis
cleanup**: documentation consolidated to this 4-file standard, thesis/test/legacy material
moved to `.archive/` (local-only), and personal information purged from git history. Active
development priorities and open hardware-calibration items are tracked in `dev-note.md` —
consult it before assuming any parameter is production-ready.

---

## 4. What Moved to `.archive/`

The following used to live in the tracked tree and are now local-only archive material —
useful for history, not for day-to-day reference:

* `report/` — the LaTeX graduation thesis and all its source material (contains the authors'
  names/student IDs; **never** reference or restore this into the tracked tree).
* `tests/` — the old `pytest` unit test harness (`test_trajectory_planning.py`).
* Superseded documentation sources: `theory_basis.md`, `academic_report.md`, `ai_context.md`
  (old status-log version), `Yolo_training_report.md`, `evaluate.md`, `evaluate_filled.md`,
  `test.md`, `rotate_t4_instability_report.md` — their durable content was folded into
  `basis-theory.md` / `basis-programming.md` / `dev-note.md`.
* `doc/archive/` (old design docs, superseded bug reports, the pre-LaTeX thesis draft) and
  `.trash/` (legacy backups).
* Stray working files: `diff_all.txt`, `scheduler_6ed7b21.py`, `mermaid-filter.err`, old
  runtime logs, `calib_result.jpg`, and old model checkpoints
  (`models/small@1280_old_dataset/`, per-epoch `.pt` files other than `best.pt`/`last.pt`).
