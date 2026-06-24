# agents.md — Delta Robot Project (General AI Guidelines)

## 0. Language
Except the final report messages must be Vietnamese, you can freely use English for thinking and document writing for saving compute sake.

## 1. Startup Protocol

**Always read `doc/ai_context.md` first** before reading any other file in this project.
It contains the active status, directory tree guidelines, PLC memory contracts, manual camera exposure details, and verification commands.
Since you are a general AI coding agent, you must follow this file (`agents.md`) and ignore `CLAUDE.md`.

---

## 2. File Access Rules

### Read freely:
- `main.py`, `README.md`
- `modules/` — all `.py` files and `config.json`
- `doc/ai_context.md`, `doc/theory_basis.md`

### Read with caution:
- `doc/academic_report.md` — academic equations and reference derivations. Do not use as primary code reference.
- `doc/Manuals/*.pdf` — large hardware documentation.

### Always write documents in English.

### Never read or edit:
- `.trash/` — legacy backups. Do not open or reference.
- `.git/`, `.venv/`, `.agents/`, `__pycache__/`, `modules/__pycache__/` — system metadata and cache. Ignore completely.

---

## 3. Code Change Rules

- **Never commit** `data.log`, `test_module.log`, `test_conveyor.log` or `__pycache__/` directories.
- **Never remove or reorder** fields in `SiemensSendPacket` or `SiemensReceivePacket` — the byte layout must match the PLC DB offsets exactly.
- **Never change** the default `interpolar_points` value in `config.json` without updating downstream arrays that pad to that size.
- After any change to `EthernetCom.py`, `scheduler.py`, or `cli.py`, run the compile check:
  ```bash
  python3 -m py_compile main.py modules/cli.py modules/EthernetCom.py modules/image_processing.py modules/scheduler.py modules/test_module.py modules/conveyor.py modules/interface.py
  ```
