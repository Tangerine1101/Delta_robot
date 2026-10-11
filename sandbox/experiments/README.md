# Experiments — how to run them and read the results

Vietnamese version: [README.vi.md](README.vi.md).

An experiment is one YAML file in this directory. You run it with one command and get a report
(tables and charts) in `sandbox/results/<experiment>-<time>/`. Nothing here needs the robot.

## 1. Run

```bash
bash sandbox/run_experiments.sh phase0           # Phase 0: does the sandbox reproduce the cell?  (~10 min)
bash sandbox/run_experiments.sh phase1           # Phase 1: the map of W against load            (~30 min)
bash sandbox/run_experiments.sh phase0 --quick   # a coarse version of any experiment (a few minutes)
bash sandbox/run_experiments.sh phase1 --dry-run # only count the runs and estimate the time
```

The same through Python (always the repository's `.venv`, the system Python lacks `ruamel`):

```bash
.venv/bin/python -m sandbox exp --list           # every experiment with its one-line description
.venv/bin/python -m sandbox exp w_map [--quick] [--dry-run] [--workers N]
.venv/bin/python -m sandbox report sandbox/results/<dir>   # redraw the tables and charts of a finished sweep
```

A run uses all cores but one. Results are not tracked by git; `manifest.json` in each result
directory records the git revision, the spec and every resolved configuration.

## 2. What you get

| File | What it holds |
|---|---|
| `report.md` | Open this first: the charts, the best level of each factor, the table of every combination (mean ± standard deviation over the seeds) |
| `chart_*.png` | The charts of the report |
| `summary.csv` | The table of every combination, for your own plots |
| `runs.csv` | One row per simulation (every factor and every metric) — the raw data |
| `manifest.json` | Provenance: git revision, spec, resolved configs |
| `<run>/parts.csv`, `flow.csv` | (replay only) the part record and flow of each sandbox run, in the columns of the cell's own logs |

Metrics worth knowing: `pick_share` = picked / arrived; `covered_share` = misses on a part that
another part lay on / arrived; `lost_share` = left the workspace unpicked; `mean_belt_mm_s`;
`dip_s` = seconds in which the 60 s throughput fell more than 8 parts/min below the input
(the dashboard's "dips"); `bypassed_lost` = parts lost after the arm waited for another part
that was farther away (open-issues S2).

## 3. The experiments

### Phase 0 — `phase0_replay` (kind `replay`)

*Question:* does the sandbox reproduce the cell? A sandbox sweep is worth running only if it does.

*How:* each logged cell run (`log/<session>/runNN_*_simulate_feeder/`) is replayed with its own
arrivals (`arrivals.csv`) under its own settings (the session's `config.yaml` plus the run's
overrides) and the two part records are compared, part by part in arrival order.

*Read it:* `report.md` has, per run, a table of checks against gates (picked within 5 %, mean belt
within 10 %, at least 85 % of parts with the same outcome), the outcome counts and the confusion
matrix, and a three-panel chart: input/output throughput, belt speed, and which parts failed, cell
against sandbox. The **Verdict** at the end says pass or FAIL per run. `_cal` entries use the
plant as identified from the cell (`sim.goto_extra_s`, `sim.pick_extra_s`); `_uncal` entries add
the model's command delay to every leg, as the sandbox did before the identification. A single
part can differ (the planner is chaotic); judge the aggregates. Two sets: `phase0_replay`
(seed 110, three runs of 2026-10-07/08) and `phase0_replay_1001` (seed 1001, the four runs of
2026-10-10: W = 15, 25, 50 s and constant 20 mm/s).

*Decision:* if the replays at the W you care about fail, fix the sandbox first (open-issues S3);
do not trust a sweep there.

`phase0_noise` replays each of five cell runs seven times with the plant unchanged and the
simulation clock shifted by a few milliseconds, to measure how much of a gap to the cell is the closed
loop's own scatter: judge a replay by whether the cell's value lies inside that spread. The report also
gives, per replay, the belt-speed RMSE, the 60 s throughput RMSE and the KS distance of the pick-interval
distribution (informational, not gated).

### The dp study — `p1_select`, `p1_confirm`, `p1_regime`, `solve_time`

`bash sandbox/run_experiments.sh p1` (1–2 h): the 20 settings of `predictive_rank` (W) and `constant`
(speed) for dp on selection seeds, the same for dp / KIM / SPT on confirmation seeds, a varying load,
and the decision time against dp `max_jobs` and the predictive-rank step; then the select-and-confirm
analyses (`python3 -m sandbox confirm`). What each is for, the seed groups and the results:
`doc/experiment_notes.md`.

### Phase 1 — `w_map`, `w_map_dp` (kind `sweep`)

*Question:* which scoring horizon W of `predictive_rank` is good, and does it depend on the load
and on stack awareness?

*How:* the grid of W × arrival rate × stack awareness, five seeds, one hour each (`w_map`, KIM),
and the same map for `dp` at the cell's load (`w_map_dp`). W = 1000 s is printed as `inf`.

*Read it:* in `report.md`, `pick_share` against W (one panel per load): a flat line is a W that
does not matter at that load, a line that falls to the right is the cliff. The **Best horizon_s**
table gives the best W per load and the plateau of W within 0.5 points of it. The heat map is the
same data at a glance.

## 4. Write a new experiment

Copy a spec, change it, run `exp <name>`. No Python is needed for a sweep.

```yaml
description: one line, shown by `exp --list`
cell_config: modules/config.yaml        # optional: scheduling / speed / feeder start as the cell's file has them
base:                                   # dotted overrides of sandbox/config.yaml, applied to every run
  sim.duration_s: 3600
  sim.stacking: true
seeds: [110, 111, 112]
grid:                                   # every combination is a case ...
  scheduling.planner: [kim, dp]
  feeder.poisson.rate_per_min: [16, 24]
cases:                                  # ... and/or named cases
  - {name: slow belt, set: {speed.law: constant, speed.static_mm_s: 20}}
quick:                                  # what --quick changes: base is merged, seeds/grid/cases replaced
  base: {sim.duration_s: 600}
  seeds: [110]
report:
  title: Planner against load
  metrics: [pick_share, covered_share, mean_belt_mm_s]   # columns of the tables (see sandbox/report.py)
  labels: {horizon_s: {1000: inf}}                        # how a level is printed
  charts:
    - {kind: line, x: rate_per_min, y: pick_share, series: planner}      # also: heatmap, bar
  best:
    - {over: planner, metric: pick_share, group: [rate_per_min], tolerance: 0.005}
```

* Keys are those of `sandbox/config.yaml` (`python3 -m sandbox run --set key=value` takes the same).
  An unknown key is an error, so a typo cannot silently run the wrong experiment.
* A **factor** (a setting that differs between cases) becomes a column of `runs.csv` named after the
  last part of its key (`rate_per_min`, `horizon_s`, `stack_aware`); two keys that end alike keep
  their full name. Use those names in `report:`.
* Anything that is not a grid — Phase 0 replays a log — has its own `kind:`. A new kind is a function
  with the signature of `_sweep` in `sandbox/exp.py` registered in `KINDS`.
* To compare with the cell, `cell_config:` takes `modules/config.yaml` or a session's
  `log/<session>/config.yaml`; `sandbox/cellrun.py` is the reader of cell logs.
* Cost: `exp <name> --dry-run` estimates the wall time from earlier runs. A one-hour run takes
  5–100 s; the load and the planner (`dp` is slowest) decide.

## 5. Known limits of the sandbox

They are the reason Phase 0 exists; the current state is in `doc/open-issues.md` (S2, S3).
The latest Phase 0 result and the rule for trusting a sweep are in S3.
