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
- `modules/` — all `.py` files and `config.yaml`, and `modules/scheduling/README.md`
- `sandbox/`, `tests/`
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
- **Never change** `plc.interpolar_points` in `config.yaml` without updating downstream arrays that pad to that size.
- Respect the layering (`doc/basis-programming.md` §1): `core` never imports `scheduling`, `scheduling` never imports `runtime`, nothing under `modules/` imports `sandbox/`. A new config key is a dataclass field in `modules/settings.py` (its one default); a moved key gets an entry in `settings.MOVED_KEYS`.
- After any Python change, run the compile check and the test suite (no hardware needed):
  ```bash
  python3 -m compileall -q main.py calibrate_everything.py camera_calibrate.py modules sandbox tests
  python3 -m unittest discover -s tests -t .
  ```

---

## 5. Subagents — pick the model by task difficulty

When spawning an agent, set `model` to match how hard the task is, not a single default:

| Model | Use for |
|---|---|
| `opus` | Design and architecture, cross-module reasoning, timing- or safety-critical code (scheduler, pick gate, speed controller, PLC data contracts), root-cause investigation |
| `sonnet` | Routine implementation against a clear spec, moderate-breadth exploration, test updates, doc rewrites |
| `haiku` | Simple tasks with short context only: grep/listing, single-file lookups, running compile checks or tests and reporting the output, formatting |

If a `haiku`/`sonnet` agent returns an uncertain or shallow result on a hard question, redo it with a stronger model rather than trusting it.
