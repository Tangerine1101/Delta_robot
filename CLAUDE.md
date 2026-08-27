# CLAUDE.md — Delta Robot Project (Claude Guidelines)

## 0. Language
Conversational report/status messages addressed to the user (chat replies, progress summaries) must be in Vietnamese. All documentation and code comments are written in English.

## 1. Startup Protocol

**Always read `doc/context.md` first**, then **`doc/open-issues.md`**, before reading any other file in this project.
`context.md` holds the current project phase, the document map, the directory tree, and the file-access rules. `open-issues.md` is the single register of every unresolved problem — several plausible-looking improvements are already blocked on an item listed there.

Since you are using Claude, you must follow this file (`CLAUDE.md`) and ignore `AGENTS.md`.

The `doc/` set is six files:

| File | Contents |
|---|---|
| `context.md` | Onboarding: phase, document map, directory tree, rules |
| `basis-theory.md` | The theory behind every algorithm |
| `basis-programming.md` | Architecture, PLC contracts, scenarios, config keys, verification commands |
| `open-issues.md` | Everything unresolved — **you may update this file** |
| `decision-log.md` | History: superseded designs and why they were replaced |
| `dev-note.md` | The human developer's bench notes — **do not edit** |

---

## 2. File Access Rules

### Read freely:
- `main.py`, `README.md`
- `modules/` — all `.py` files and `config.json`
- `doc/*.md`

### Read with caution:
- `doc/Manuals/*.pdf` — large hardware documentation; open only to check a specific register.

### Never read or edit:
- `.archive/` — local-only archive (thesis, legacy backups, superseded docs). Do not open or reference.
- `doc/dev-note.md` — maintained by the human developer; do not edit unless explicitly asked.
- `.git/`, `.venv/`, `.agents/`, `runs/`, `__pycache__/`, `modules/__pycache__/` — system metadata, caches and training artefacts. Ignore completely.

---

## 3. Documentation Rules

- **Descriptive documents describe only current behaviour.** If you are about to write "used to", "was replaced by", "previously", or "[FIXED]" in `basis-theory.md` / `basis-programming.md` / `README.md`, that text belongs in `doc/decision-log.md` instead.
- **Unresolved problems go to `doc/open-issues.md`** — never leave a TODO, a "not yet calibrated" aside, or a known limitation buried in prose elsewhere.
- When you close an issue, **delete its row** from `open-issues.md` and record the resolution in `decision-log.md`. Do not accumulate `[FIXED]` entries.
- A config key being documented is not a claim that its value was measured; check `open-issues.md` §A–§B before trusting a number.

---

## 4. Code Change Rules

- **Never commit** `data.log` or other runtime log files, or `__pycache__/` directories.
- **Never remove or reorder** fields in `SiemensSendPacket` or `SiemensReceivePacket` — the byte layout must match the PLC DB offsets exactly.
- **Never change** the default `interpolar_points` value in `config.json` without updating downstream arrays that pad to that size.
- After any change to `EthernetCom.py`, `scheduler.py`, or `cli.py`, run the compile check:
  ```bash
  python3 -m py_compile main.py modules/cli.py modules/EthernetCom.py modules/image_processing.py modules/scheduler.py modules/test_module.py modules/conveyor.py modules/interface.py
  ```
