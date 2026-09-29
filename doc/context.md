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

* The planners and speed laws are plugins of one framework (`modules/scheduling/`, contracts
  in its `README.md`); new algorithms are added there and measured in `sandbox/` before they
  run on the cell. The laws documented in `basis-theory.md` §6–§7 are baselines, not designs
  to defend.
* **Every unresolved problem is registered in [`open-issues.md`](open-issues.md).** Read it
  before proposing changes — several obvious-looking improvements are blocked on a
  calibration or a hardware fact listed there. It is the only file where open problems live.
* A value in `modules/config.yaml` being present is **not** evidence it was measured.

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
| [`plc/`](plc/README.md) | PLC programs (deployed Omron `Matching_Code_10`, target `delta_paper_0_2`), PC↔PLC data contract, defect review with proposed ST patches, config review against the PLC, PLC simulator | anyone |
| [`pick-accuracy-findings.md`](pick-accuracy-findings.md) | Pick-gate timing defect register (T1–T9): why picks land off-centre and worsen above ~120 mm/s, evidence, fix roadmap | anyone |
| [`../modules/scheduling/README.md`](../modules/scheduling/README.md) | How to write a dispatch rule, planner or speed law: the contracts, with worked examples | anyone |
| [`../sandbox/README.md`](../sandbox/README.md) | The offline algorithm bench: configuration, model/plant, sweeps, what is modelled | anyone |

**Rule of thumb**: descriptive documents (`basis-*`) describe only current behaviour. If you
find yourself writing "used to", "was replaced by", or "[FIXED]" in them, that text belongs in
`decision-log.md`. If you find yourself writing "TODO" or "not yet calibrated", it belongs in
`open-issues.md`.

---

## 3. Directory Structure

```
Delta_robot/
├── main.py                    # Entry point: --cli, --scheduler, or the operator console (--interface)
├── camera_calibrate.py        # Camera calibration tool (ROI, trigger line, pixels/mm)
├── calibrate_everything.py    # Whole-config consistency + workspace boundary checker
├── README.md                  # Short project overview + quickstart (doc/ is authoritative)
├── requirements.txt           # Python dependency list
├── CLAUDE.md, AGENTS.md       # AI developer rulebooks (Claude / other agents)
│
├── modules/                   # The control software (layers: basis-programming.md §1)
│   ├── settings.py            # Every config key: type, one default, loader, moved-key errors
│   ├── config.yaml            # Active configuration (reference: basis-programming.md §8)
│   ├── config_io.py           # Comment-preserving config.yaml reader/writer
│   ├── runlog.py              # Per-run debug logs under log/
│   ├── core/                  # Geometry and time models: frames, tracking, kinematics (PLC IK/FK),
│   │                          #   motion (PLC time model), trajectory templates, forecast, arm models,
│   │                          #   feeders (seeded arrivals)
│   ├── scheduling/            # Which part next, which belt speed: rules, planners, speed laws,
│   │                          #   gates, commit policy (README.md = the plugin contracts)
│   ├── runtime/               # Realtime cell: decision loop, perception thread, pick executor,
│   │                          #   pick gate, speed controller, scenario registry, part record,
│   │                          #   virtual feeder
│   ├── comm/                  # PLC data contract, Omron/Siemens gateways, PLC worker, UDP pose stream
│   ├── vision/                # PyAV capture, YOLO-OBB pipeline, board heading, ROI, centroid tracker
│   ├── ui/                    # Web dashboard, operator console back-end, interactive CLI
│   ├── tools/                 # Probes: latency, rotation, angle sweep, frame converter, rank bounds, UDP;
│   │                          #   flow_report (input vs throughput across runs)
│   └── plc_sim/               # PLC simulator (Omron Matching_Code_10 port, Siemens, boards, camera)
│
├── sandbox/                   # Offline algorithm bench: config, arm models, feeders, sweeps (README.md)
├── tests/                     # Unit and integration tests: python3 -m unittest discover -s tests -t .
│
├── doc/                       # Documentation (see §2)
│   ├── context.md, basis-theory.md, basis-programming.md
│   ├── open-issues.md, decision-log.md, dev-note.md
│   ├── pick-accuracy-findings.md  # Pick-gate timing defect register (T1-T9) + fix roadmap
│   ├── proposal/              # Research proposal / paper sources (LaTeX)
│   ├── plc/                   # PLC programs, PC↔PLC data contract, PLC defect review,
│   │                          #   config review, simulator (start at plc/README.md)
│   └── Manuals/               # PLC & hardware datasheets (open only to check a register)
│
├── log/                       # Run logs written by main.py (git-ignored; basis-programming.md §2.5)
├── OMRON/                     # Omron Sysmac exports: matching code/ (current), delta_paper_0_2/ (target)
├── windows/                   # Windows launchers of the operator console (setup.bat, run.bat)
└── models/                    # Trained YOLO weights (nano@1280, nano@1920 = default)
```

**Ignored working directories** (present on disk, not part of the project): `runs/` (YOLO
training/inference output), `sandbox/results/` (sweep output), `.report/`, `.venv/`,
`__pycache__/`, and `.archive/` — a local-only archive of the graduation thesis, the old test
harness, the 2026-08 pick-accuracy investigation and superseded docs.

The scheduling research that produced the published results lives in a separate repository
(`../python for scheduling`, frozen at tag `paper-submitted`); algorithm work continues in
`sandbox/`.
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

* Read freely: `main.py`, `README.md`, `modules/**/*.py`, `modules/config.yaml`,
  `modules/scheduling/README.md`, `sandbox/`, `tests/`, and `doc/*.md`.
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
  byte layout must match the PLC DB offsets exactly (`basis-programming.md` §3.2,
  `modules/comm/packets.py`).
* **Never change** `plc.interpolar_points` in `config.yaml` without updating every
  downstream array that pads to that size.
* Respect the layering (`basis-programming.md` §1); a new config key is a field in
  `modules/settings.py`, a moved key an entry in `settings.MOVED_KEYS`.
* After any Python change, run the compile check and the test suite in
  `basis-programming.md` §9.
