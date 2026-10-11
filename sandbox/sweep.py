"""An experiment = a YAML spec of cases x seeds, run in parallel, written as one CSV.

    description: planners under load
    cell_config: modules/config.yaml       # optional: start from this config's scheduling / speed / feeder
    base: {sim.duration_s: 300}            # dotted overrides applied to every run
    seeds: [1, 2, 3]
    grid:                                  # every combination becomes a case ...
      scheduling.planner: [kim, spt]
      feeder.poisson.rate_per_min: [18, 24]
    cases:                                 # ... and/or named cases
      - {name: rank, set: {speed.law: predictive_rank}}
    quick: {base: {sim.duration_s: 300}, seeds: [1]}   # optional: what --quick changes
    report: {...}                          # optional: tables and charts (sandbox/report.py)

Results go to sandbox/results/<spec>-<timestamp>/ (git-ignored): `runs.csv`, one row per run
(the factors that vary between cases are columns of their own), and `manifest.json` (git
revision, the spec, the resolved configs) so a number can be traced to the code and inputs that
produced it. With a `report:` block the same directory also gets `report.md`, `summary.csv` and
the charts (`python3 -m sandbox report DIR` redraws them).
"""

from __future__ import annotations

import csv
import itertools
import json
import os
import subprocess
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Any

from modules.config_io import read_config
from sandbox.cellrun import cell_overrides
from sandbox.config import SANDBOX_DIR, build_run, load_model_file, load_run_config

RESULTS_DIR = SANDBOX_DIR / "results"


def simulate(overrides: dict[str, Any]):
    """One finished `Simulation` (its part record and belt log are still on it)."""
    from sandbox.sim import Simulation

    run = build_run(load_run_config(overrides=overrides))
    sim = Simulation(run, load_model_file(run.arm.model))
    sim.run_sim()
    return sim


def run_one(overrides: dict[str, Any]) -> dict[str, Any]:
    """One simulation; importable by worker processes."""
    return simulate(overrides).metrics.summary()


def write_run(sim, overrides: dict[str, Any], out_dir: str | Path) -> dict[str, Any]:
    """The part record of a finished simulation in `out_dir`: `parts.csv` (the columns of the
    robot's, for `python3 -m sandbox dips`), `flow.csv` (trailing 60 s input and output, belt
    speed), `waits.json` (every commit that made the arm wait, with the parts it could have
    taken) and `summary.json`."""
    from sandbox.flow import flow_series

    summary = sim.metrics.summary()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rows = sim.records()
    with open(out / "parts.csv", "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    belt = dict(sim.belt_log)
    with open(out / "flow.csv", "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("t", "input_per_min", "output_per_min", "belt_mm_s"))
        writer.writerows((t, round(i, 2), round(o, 2), belt.get(float(t), ""))
                         for t, i, o in flow_series([{k: str(v) for k, v in r.items()} for r in rows]))
    (out / "waits.json").write_text(json.dumps(sim.wait_log, indent=1))
    (out / "summary.json").write_text(json.dumps({"overrides": overrides, **summary}, indent=2, default=str))
    return summary


def run_recorded(overrides: dict[str, Any], out_dir: str | Path) -> dict[str, Any]:
    """One simulation whose part record is written to `out_dir` (see `write_run`)."""
    return write_run(simulate(overrides), overrides, out_dir)


def load_spec(spec_path: str | Path, quick: bool = False) -> dict[str, Any]:
    """A spec as a plain dict; with `quick`, its `quick:` block replaces `seeds` / `grid` /
    `cases` and is merged into `base` (a shorter, coarser version of the same experiment)."""
    from sandbox.cellrun import plain

    spec = plain(read_config(spec_path))
    if quick:
        override = spec.get("quick") or {}
        if not override:
            raise SystemExit(f"{spec_path}: no `quick:` block, nothing for --quick to change")
        spec["base"] = {**(spec.get("base") or {}), **(override.get("base") or {})}
        for key in ("seeds", "grid", "cases"):
            if key in override:
                spec[key] = override[key]
    return spec


def expand(spec: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    base = dict(cell_overrides(spec["cell_config"])) if spec.get("cell_config") else {}
    base.update(spec.get("base") or {})
    cases: list[tuple[str, dict[str, Any]]] = []
    grid = spec.get("grid") or {}
    if grid:
        keys = list(grid)
        for values in itertools.product(*(grid[k] for k in keys)):
            setting = dict(zip(keys, values))
            name = ",".join(f"{k.split('.')[-1]}={v}" for k, v in setting.items())
            cases.append((name, {**base, **setting}))
    for case in spec.get("cases") or []:
        cases.append((str(case["name"]), {**base, **(case.get("set") or {})}))
    if not cases:
        cases.append(("base", base))
    return cases


def factors_of(cases: list[tuple[str, dict[str, Any]]]) -> dict[str, str]:
    """{column name: dotted key} of the settings that differ between the cases. A column is
    named after the last part of its key, or the whole key when two would collide."""
    keys = sorted({k for _, overrides in cases for k in overrides
                   if len({repr(o.get(k)) for _, o in cases}) > 1 and k != "sim.seed"})
    short = [k.split(".")[-1] for k in keys]
    return {(k.split(".")[-1] if short.count(k.split(".")[-1]) == 1 else k): k for k in keys}


def _cell_text(value: Any) -> Any:
    return "" if value is None else value


def estimate_wall_s(jobs: list[tuple[str, int, dict[str, Any]]], workers: int) -> float:
    """Wall time of a sweep, from the speed of the earlier ones (seconds of wall time per
    simulated second); 50 s per simulated hour when there is no history."""
    wall = sim = 0.0
    for runs in RESULTS_DIR.glob("*/runs.csv"):
        try:
            manifest = json.loads((runs.parent / "manifest.json").read_text())
            durations = {n: float(r["sim"]["duration_s"]) for n, r in manifest["resolved"].items()}
            with open(runs, newline="") as handle:
                for row in csv.DictReader(handle):
                    if row["case"] in durations and row.get("wall_s"):
                        wall += float(row["wall_s"])
                        sim += durations[row["case"]]
        except (OSError, KeyError, ValueError):
            continue
    rate = wall / sim if sim > 0 else 50.0 / 3600.0
    total = sum(float(build_run(load_run_config(overrides=o)).sim.duration_s) * rate for _, _, o in jobs)
    return total / max(1, workers)


def _git_revision() -> str | None:
    try:
        rev = subprocess.run(["git", "rev-parse", "HEAD"], cwd=SANDBOX_DIR, capture_output=True,
                             text=True, timeout=5).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain"], cwd=SANDBOX_DIR, capture_output=True,
                               text=True, timeout=5).stdout.strip()
        return rev + ("+dirty" if dirty else "")
    except Exception:
        return None


def sweep(spec_path: str | Path, workers: int | None = None, *, quick: bool = False,
          dry_run: bool = False) -> Path | None:
    spec_path = Path(spec_path)
    spec = load_spec(spec_path, quick)
    seeds = [int(x) for x in (spec.get("seeds") or [1])]
    cases = expand(spec)
    factors = factors_of(cases)
    jobs = [(name, seed, {**overrides, "sim.seed": seed}) for (name, overrides), seed in itertools.product(cases, seeds)]
    workers = workers or max(1, (os.cpu_count() or 2) - 1)
    if dry_run:
        minutes = estimate_wall_s(jobs, workers) / 60.0
        print(f"[SWEEP] {spec_path.name}{' (quick)' if quick else ''}: {len(jobs)} runs "
              f"({len(cases)} cases x {len(seeds)} seeds) on {workers} processes, about {minutes:.0f} min")
        return None
    out = RESULTS_DIR / f"{spec_path.stem}{'-quick' if quick else ''}-{datetime.now():%Y%m%d-%H%M%S}"
    out.mkdir(parents=True)
    manifest = {
        "spec": spec_path.name,
        "description": spec.get("description"),
        "quick": quick,
        "git": _git_revision(),
        "started": datetime.now().isoformat(timespec="seconds"),
        "cases": {name: overrides for name, overrides in cases},
        "seeds": seeds,
        "factors": factors,
        "spec_full": spec,
        "resolved": {name: asdict(build_run(load_run_config(overrides=overrides))) for name, overrides in cases},
    }
    print(f"[SWEEP] {len(jobs)} runs ({len(cases)} cases x {len(seeds)} seeds) on {workers} processes -> {out}")
    started = time.perf_counter()
    rows = []
    case_overrides = dict(cases)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = [(name, seed, pool.submit(run_one, overrides)) for name, seed, overrides in jobs]
        for name, seed, future in futures:
            result = future.result()
            levels = {col: _cell_text(case_overrides[name].get(key)) for col, key in factors.items()}
            rows.append({"case": name, "seed": seed, **levels, **result})
            print(f"  {name:<40} seed {seed:<3} picked {result['picked']:>4}/{result['spawned']:<4} "
                  f"rate {result['pick_rate']}  {result['wall_s']:.1f}s", flush=True)
    manifest["wall_s"] = round(time.perf_counter() - started, 1)
    with open(out / "runs.csv", "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str))
    print(f"[SWEEP] done in {manifest['wall_s']} s")
    if spec.get("report"):
        from sandbox.report import build_report

        build_report(out)
    return out
