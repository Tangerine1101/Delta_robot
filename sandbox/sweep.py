"""An experiment = a YAML spec of cases x seeds, run in parallel, written as one CSV.

    description: planners under load
    base: {sim.duration_s: 300}            # dotted overrides applied to every run
    seeds: [1, 2, 3]
    grid:                                  # every combination becomes a case ...
      scheduling.planner: [kim, spt]
      feeder.poisson.rate_per_min: [18, 24]
    cases:                                 # ... and/or named cases
      - {name: rank, set: {speed.law: predictive_rank}}

Results go to sandbox/results/<spec>-<timestamp>/ (git-ignored): `runs.csv`, one row per run,
and `manifest.json` (git revision, the spec, the resolved configs) so a number can be traced to
the code and inputs that produced it.
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
from sandbox.config import SANDBOX_DIR, build_run, load_model_file, load_run_config

RESULTS_DIR = SANDBOX_DIR / "results"


def run_one(overrides: dict[str, Any]) -> dict[str, Any]:
    """One simulation; importable by worker processes."""
    from sandbox.sim import Simulation

    raw = load_run_config(overrides=overrides)
    run = build_run(raw)
    sim = Simulation(run, load_model_file(run.arm.model))
    return sim.run_sim().summary()


def expand(spec: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    base = dict(spec.get("base") or {})
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


def _git_revision() -> str | None:
    try:
        rev = subprocess.run(["git", "rev-parse", "HEAD"], cwd=SANDBOX_DIR, capture_output=True,
                             text=True, timeout=5).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain"], cwd=SANDBOX_DIR, capture_output=True,
                               text=True, timeout=5).stdout.strip()
        return rev + ("+dirty" if dirty else "")
    except Exception:
        return None


def sweep(spec_path: str | Path, workers: int | None = None) -> Path:
    spec_path = Path(spec_path)
    spec = read_config(spec_path)
    seeds = [int(s) for s in (spec.get("seeds") or [1])]
    cases = expand(spec)
    jobs = [(name, seed, {**overrides, "sim.seed": seed}) for (name, overrides), seed in itertools.product(cases, seeds)]
    out = RESULTS_DIR / f"{spec_path.stem}-{datetime.now():%Y%m%d-%H%M%S}"
    out.mkdir(parents=True)
    manifest = {
        "spec": spec_path.name,
        "description": spec.get("description"),
        "git": _git_revision(),
        "started": datetime.now().isoformat(timespec="seconds"),
        "cases": {name: overrides for name, overrides in cases},
        "seeds": seeds,
        "resolved": {name: asdict(build_run(load_run_config(overrides=overrides))) for name, overrides in cases},
    }
    workers = workers or max(1, (os.cpu_count() or 2) - 1)
    print(f"[SWEEP] {len(jobs)} runs ({len(cases)} cases x {len(seeds)} seeds) on {workers} processes -> {out}")
    started = time.perf_counter()
    rows = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = [(name, seed, pool.submit(run_one, overrides)) for name, seed, overrides in jobs]
        for name, seed, future in futures:
            result = future.result()
            rows.append({"case": name, "seed": seed, **result})
            print(f"  {name:<40} seed {seed:<3} picked {result['picked']:>4}/{result['spawned']:<4} "
                  f"rate {result['pick_rate']}  {result['wall_s']:.1f}s", flush=True)
    manifest["wall_s"] = round(time.perf_counter() - started, 1)
    with open(out / "runs.csv", "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str))
    print(f"[SWEEP] done in {manifest['wall_s']} s")
    return out
