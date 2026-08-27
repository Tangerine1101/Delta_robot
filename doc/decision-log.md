# Decision Log — Delta Robot

> ## ⚠ HISTORY ONLY — nothing in this file describes how the system works today.
>
> Every entry below is a design that was **tried and replaced**, a calibration that has
> **already been applied**, or an idea that is **parked**. Read it to understand *why* the
> current design is shaped the way it is — never to learn what the code does. For current
> behaviour read [`basis-theory.md`](basis-theory.md) and
> [`basis-programming.md`](basis-programming.md); for what is still unresolved read
> [`open-issues.md`](open-issues.md).
>
> Its purpose is to preserve the *why-not* reasoning — the material a paper needs when
> justifying a design choice against the obvious alternatives — without letting that
> reasoning contaminate the descriptive documents.

---

## 1. Superseded algorithms and design choices

### 1.1. Belt speed proportional to load — rejected

**Rejected model**: $v = A \cdot N + v_{\min}$ ("run the belt fast when the cell is busy").

**Why rejected**: belt speed does not set throughput; the serial arm's pick cycle does.
Speeding the belt up under high density only shortens each part's transit through the
workspace and pushes parts past $u_{\max}$ unpicked. Replaced by the **inverse** density law
(`basis-theory.md` §6.2), where higher density means a *slower* belt.

### 1.2. Speed commits at the grip instant only — replaced

The original controller committed a `change_speed` only at the moment of grip: at most one
commit per pick cycle, i.e. one per 2–10 s. Density changes therefore sat uncommitted for
seconds, and on hardware the belt felt laggy and uneven. Quantitatively, the large speed
jumps that policy allowed need acceleration ramps longer than the window it assumed was
free.

Replaced by the inverted policy (`basis-theory.md` §6.5): commits are *opportunistic* —
allowed from the executor wait loops and the idle loop, throttled ≥ 0.75 s apart — and
suppressed **only** inside the gate-critical window.

### 1.3. Fixed wall-clock gate-abort deadline — replaced

A pick that had not fired within a fixed wall-clock deadline was aborted. This fired
spuriously whenever the belt slowed after plan-build, since the deadline had been computed
against the earlier, faster belt. Replaced by a **progress-based stall check**: abort when
the tracked object's $u$ advances less than 0.5 mm over several seconds, or when the track
is lost.

### 1.4. Blanket object removal on a failed pick — replaced

Any failed pick used to remove the object from the tracker. That dropped objects which were
still perfectly pickable, and it also deflated the density count $N$, which sped the belt up
immediately after a failure — precisely the wrong response. Replaced by the exclusivity rule
of `basis-theory.md` §4.7: removal keys off whether the **grip was dispatched**, and a
pre-grip abort only unclaims.

### 1.5. Parking the arm upstream by $v \cdot t_d$ for the oblique descent — replaced

The first version of the belt-tracking slanted descent shifted the whole *park* position
upstream by the board's travel during the descent. At operating belt speed this placed the
arm outside the workspace. Corrected so that only the pick-phase **contact** point shifts
downstream; the goto/park target is unchanged (`basis-theory.md` §4.5).

### 1.6. Wrapping the wire angle to $[-180°, 180°)$ — replaced

The Siemens rotation command encodes **spin direction as well as position**, so wrapping the
commanded angle at the IPC boundary flipped the direction of travel: a $179° \to 180°$ step
became $179° \to -180°$ and drove the axis nearly a full turn the wrong way. This was the
"random over-rotation" fault. Fixed by making the wire layer a verbatim
radians→degrees identity clamped to $\pm 359$, with the single minimal-turn wrap moved up to
the algorithm layer (`basis-theory.md` §5.2, three-layer convention).

### 1.7. Time-based pick dispatch — replaced

The predictor originally converged on a pick *time* and fired the pick when that time
arrived, which made the pick sensitive to belt-speed estimate noise (the "arrive → wait →
miss" lag). Replaced by a **fixed-point park position plus a live positional gate**
(`basis-theory.md` §4.3–4.4): the predictor chooses where to park, and the pick fires when
the tracked object physically reaches that position.

### 1.8. Single-threaded real-time loop — replaced (for `production` only)

The original scheduler blocked the whole loop inside `executor.execute()` for 4–7 s per
pick, so belt sampling, vision polling and re-anchoring all froze during a pick and every
second-and-later pick in a burst landed behind its part. Rebuilt as the two-thread model
(perception/state daemon + decision/execution main thread over one guarded `RealtimeState`).

**Still relevant**: the simulated scenarios were never migrated and still run the old
single-threaded harness — that is open issue **L7**, not history.

### 1.9. `run_test.py` subprocess + matplotlib launcher — removed

Conflicted with the OpenCV window on real hardware. Replaced by the in-process web dashboard
(`modules/interface.py`, `--interface`); simulation moved into the scheduler itself via
`--simulate-executor`.

### 1.10. Config keys retired during the frame migration

`pickup_window_x` / `pickup_window_y` (replaced by `conveyor.workspace_window_uv`),
`accuracy_points` (replaced by `conveyor.accuracy_points_uv`), `throughput_spawn_x`,
`period_s`, and the `object_A`-style class naming (replaced by `object_types.{QFP,TQFP}`).
`belt_speed_static_mm_s` was renamed from an earlier key **without a fallback** — an old
config file will not load its value silently, it will simply use the default.

---

## 2. Calibrations already applied

### 2026-07-09/10 — hardware run

* Sorting-bin drop positions `QFP` / `TQFP` and `pickup_height` nudged from live pick data.
* `pick_arrival_tolerance_mm` / `_max_mm` widened to 15/50 mm; the previous 5/10 mm band was
  too tight for the belt-speed range in use. *(The root cause behind needing the wider band
  is still open — see `open-issues.md` G6.)*
* `belt_speed_static_mm_s` raised to 120 mm/s; `belt_speed_min/max_mm_s` rebalanced to
  30–100 mm/s. *(This is what created the inconsistency logged as G1.)*

---

## 3. Repository restructuring

### 2026-07-11 — post-thesis cleanup

* Documentation consolidated into a fixed file standard under `doc/`.
* `report/` (the LaTeX graduation thesis), `tests/` (the old pytest harness),
  `doc/archive/`, `.trash/`, and all superseded documentation sources moved to a local-only,
  gitignored `.archive/` directory. Superseded sources included `theory_basis.md`,
  `academic_report.md`, the old status-log `ai_context.md`, `Yolo_training_report.md`,
  `evaluate.md`, `evaluate_filled.md`, `test.md`, and `rotate_t4_instability_report.md`.
* Personal information (thesis authors' names and student IDs, only ever present under
  `report/` and `doc/archive/report_draft_v1/`) purged from git history with
  `git-filter-repo`, together with heavy blobs (`report.zip`, `models/small@1280_old_dataset/`,
  per-epoch `.pt` checkpoints).
* A full mirror of the pre-purge repository is kept locally at
  `../Delta_robot_git_backup_2026-07-11.git` as the permanent archive of the original
  history — **never push it anywhere**.

`.archive/` is local-only and must never be read, referenced, or restored into the tracked
tree.

### 2026-08-10 — documentation re-standardisation

The repository moved from "finished thesis project" to "starting point for scheduling
research". Documentation was re-standardised to six files under `doc/`
(`context.md`, `basis-theory.md`, `basis-programming.md`, `open-issues.md`,
`decision-log.md`, `dev-note.md`).

* `README.md` was rewritten from ~370 lines to a lean quickstart. It had accumulated roughly
  forty statements contradicting the live code — a retired `test_conveyor` scenario, an
  `interpolar_points` default of 4 (the real value is 7), `intercept_lead_time_s` given as
  1.6 s (the real value is 0.8), a claim that `_belt_lead_offset_mm` was an empty hook
  returning `0.0` when it is implemented, a roadmap of already-completed work, a list of
  bugs all marked `[FIXED]`, and links to five documents that no longer exist
  (`realtime_pick_redesign.md`, `rebuild_plan.md`, `bug_report_final.md`, `frames.png`,
  `theory_basis.md`).
* All still-unresolved items were collected from `dev-note.md`, `README.md` §6,
  `basis-theory.md` prose and `calibrate_everything.py` into `open-issues.md`.
* Historical narrative was removed from the descriptive documents and collected here.

---

## 4. Parked ideas

Not rejected, not scheduled — recorded so they are not re-invented from scratch.

* **Web GUI dashboard v2** — live 3-D end-effector trajectory, positional-error graphs from
  `data.log`, sorted-item database views. The current in-process dashboard covers telemetry
  and MJPEG only.
* **SQL sorting database** — `product_types` (destination per class) plus `pick_history`
  (per-pick audit trail with timestamps and status). Would also give the independent pick
  count that open issue L6 needs.
* **Queueing-theory belt control** — the inverse-density rate regulation of
  `basis-theory.md` §6 is the near-term realisation; a full Little's-Law model of the cell is
  the longer-term evolution and overlaps directly with the current research direction.
* **Suction verification** — a vacuum-pressure sensor or a post-grip vision check would make
  the exactly-once pick policy retry-capable and would fix the measurement gap in L6.
* **Jerk/acceleration-limited profile smoothing** on top of the mandatory 3-D slope
  waypoints.
