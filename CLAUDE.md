# CLAUDE.md — Delta Robot Project (Claude Guidelines)

## 0. Language
Conversational report/status messages addressed to the user (chat replies, progress summaries) must be in Vietnamese. You may otherwise use English freely for thinking and for working documents to save compute.

## 1. Startup Protocol

**Always read `doc/context.md` first** before reading any other file in this project.
It contains the directory tree guidelines, PLC memory contracts, manual camera exposure details, and verification commands. For the theoretical basis of the algorithms see `doc/basis-theory.md`; for program architecture and config keys see `doc/basis-programming.md`; for developer notes and pending calibration see `doc/dev-note.md`.
Since you are using Claude, you must follow this file (`CLAUDE.md`) and ignore `agents.md`.

---

## 2. File Access Rules

### Read freely:
- `main.py`, `README.md`
- `modules/` — all `.py` files and `config.json`
- `doc/context.md`, `doc/basis-theory.md`, `doc/basis-programming.md`, `doc/dev-note.md`

### Read with caution:
- `doc/Manuals/*.pdf` — large hardware documentation.

### Always write documents in English.

### Never read or edit:
- `.archive/` — local-only archive (thesis, legacy backups, superseded docs). Do not open or reference.
- `doc/dev-note.md` — maintained by the human developer; do not edit unless explicitly asked.
- `.git/`, `.venv/`, `.agents/`, `__pycache__/`, `modules/__pycache__/` — system metadata and cache. Ignore completely.

---

## 3. Code Change Rules

- **Never commit** `data.log` or other runtime log files, or `__pycache__/` directories.
- **Never remove or reorder** fields in `SiemensSendPacket` or `SiemensReceivePacket` — the byte layout must match the PLC DB offsets exactly.
- **Never change** the default `interpolar_points` value in `config.json` without updating downstream arrays that pad to that size.
- After any change to `EthernetCom.py`, `scheduler.py`, or `cli.py`, run the compile check:
  ```bash
  python3 -m py_compile main.py modules/cli.py modules/EthernetCom.py modules/image_processing.py modules/scheduler.py modules/test_module.py modules/conveyor.py modules/interface.py
  ```
