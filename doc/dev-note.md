# Dev Notes — Delta Robot

> **Maintained by the human developer.** AI assistants should read this for context but must
> **not** edit it unless explicitly asked to. Record here whatever `basis-theory.md` /
> `basis-programming.md` are too formal for: calibration TODOs, hardware quirks, open
> questions, things tried and abandoned.

---

## Pending hardware calibration

- **`rotate_sign`** — currently `1` in `config.json`. Confirm with the visual sweep in
  `python3 -m modules.test_rotate` (+90° commanded must turn the cup CCW seen from above; if
  not, flip to `-1`).
- **`robot_movement_delay_s`** (currently `0.17`) / **`ethernet_delay_s`** (currently `0.016`)
  — refine from the per-pick `[GATE]` log (`dispatch_to_contact_s`) and
  `modules/latency_probe.py --target siemens` (this is the one that gates picks).
- **`oblique_descent_enabled`** (currently `False`) — re-evaluate once `interpolator.a_max` is
  calibrated against the real belt; the descent-time model over-estimates $t_d$ until then, so
  keep off in production until re-tested.
- **Siemens DB1 handshake** — DB1 has no handshake bit; verify in TIA Portal whether the ST
  program edge-triggers on `CommandID` change (would silently drop a second `rotate_absolute`
  sent back-to-back with `change_speed`). Document findings in
  `doc/PLC_Program_description/` if confirmed either way.
- **`rotate_offset_deg` / `offset_by_class`** — re-check after any marker/mounting change on
  the suction cup; calibrated via a hardware run reading the `[ROTATE]` log.

## Recent hardware calibration (2026-07-09/10, applied)

- Sorting-bin drop positions `QFP`/`TQFP` and `pickup_height` nudged from live pick data.
- `pick_arrival_tolerance_mm` / `_max_mm` widened (15/50mm) — the previous 5/10mm band was too
  tight for the belt-speed range now in use.
- `belt_speed_static_mm_s` raised to 120 mm/s; `belt_speed_min/max_mm_s` rebalanced to
  30–100 mm/s.

## Things tried and abandoned

- **Grip-instant-only speed commits** — too coarse (≤1 commit per pick cycle, 2–10s); belt felt
  laggy on hardware. Replaced by the opportunistic commit policy
  (`basis-theory.md` §6.5).
- **Wrapping the wire-degree boundary to [-180,180)** — caused near-full-turn spins on a
  179°→180° step because the Siemens command encodes spin direction, not just position. Fixed
  by making Layer 3 verbatim (`basis-theory.md` §5.2).
- **Parking the arm upstream by `v·t_d` for oblique descent** — flew the arm outside the
  workspace at belt speed. Only the pick-phase *contact* point shifts downstream now, never the
  park/goto.
- **Fixed wall-clock gate-abort deadline** — fired spuriously whenever the belt slowed
  mid-cycle. Replaced by a progress-based stall check (object `u` advances <0.5mm for ~3s).

## Repo housekeeping (2026-07-11)

- Consolidated all documentation into the 4-file standard
  (`basis-theory.md`/`basis-programming.md`/`context.md`/`dev-note.md`).
  `report/`, `tests/`, `doc/archive/`, `.trash/`, and superseded doc sources moved to a
  local-only `.archive/` directory (gitignored).
- Purged personal information (thesis authors' names/student IDs, only ever present under
  `report/` and `doc/archive/report_draft_v1/`) from git history via `git-filter-repo`, along
  with heavy blobs (`report.zip`, old model checkpoints under
  `models/small@1280_old_dataset/` and per-epoch `.pt` files).
- A full mirror of the pre-purge repository is kept locally
  (`../Delta_robot_git_backup_2026-07-11.git`) as the permanent archive of the original
  history — **never push it anywhere**.
- README.md quickstart sections may still reference retired concepts (e.g. `test_conveyor`,
  `run_test.py`) — `basis-programming.md` is authoritative on current scenarios/architecture;
  update README opportunistically when touched.
