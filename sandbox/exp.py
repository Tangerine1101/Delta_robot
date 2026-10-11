"""Run a named experiment: `python3 -m sandbox exp <name> [--quick] [--dry-run]`.

An experiment is a YAML file in `sandbox/experiments/`; its `kind` says how it runs:

* `sweep` (default) — cases x seeds in parallel, then the tables and charts of its `report:`
  block (`sandbox/sweep.py`, `sandbox/report.py`);
* `replay` — logged cell runs replayed and compared with the cell (`sandbox/replay.py`).

A new experiment is a new file; a new kind of experiment is an entry in `KINDS`.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

from modules.config_io import read_config
from sandbox.config import SANDBOX_DIR

EXPERIMENTS_DIR = SANDBOX_DIR / "experiments"


def _sweep(path: Path, workers: int | None, quick: bool, dry_run: bool) -> Path | None:
    from sandbox.sweep import sweep

    return sweep(path, workers, quick=quick, dry_run=dry_run)


def _replay(path: Path, workers: int | None, quick: bool, dry_run: bool) -> Path | None:
    from sandbox.replay import run_replay

    if dry_run:
        n = len(read_config(path)["runs"])
        print(f"[REPLAY] {path.name}: {n} cell runs, one sandbox run each (about {n * (0.2 if quick else 0.7):.0f} min)")
        return None
    return run_replay(path, quick=quick)


KINDS: dict[str, Callable[[Path, int | None, bool, bool], Path | None]] = {"sweep": _sweep, "replay": _replay}


def find_spec(name: str) -> Path:
    path = Path(name)
    if path.is_file():
        return path
    path = EXPERIMENTS_DIR / (name if name.endswith(".yaml") else f"{name}.yaml")
    if not path.is_file():
        raise SystemExit(f"no experiment '{name}' (python3 -m sandbox exp --list)")
    return path


def describe() -> str:
    lines = []
    for path in sorted(EXPERIMENTS_DIR.glob("*.yaml")):
        spec = read_config(path)
        lines.append(f"  {path.stem:<22} {str(spec.get('kind', 'sweep')):<7} {str(spec.get('description') or '').strip()}")
    return "Experiments (sandbox/experiments/):\n" + "\n".join(lines)


def run_experiment(name: str, *, workers: int | None = None, quick: bool = False, dry_run: bool = False) -> Path | None:
    path = find_spec(name)
    kind = str(read_config(path).get("kind", "sweep"))
    if kind not in KINDS:
        raise SystemExit(f"{path.name}: unknown kind '{kind}' (known: {', '.join(KINDS)})")
    return KINDS[kind](path, workers, quick, dry_run)
