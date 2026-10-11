"""A logged cell run (`simulate_feeder`), read back so the sandbox can be set up like it.

A run directory `log/<session>/runNN_<time>_simulate_feeder/` holds `summary.json`, `parts.csv`,
`flow.csv` and `arrivals.csv`; the session directory above it holds the `config.yaml` the
console started with. `CellRun.overrides()` turns them into sandbox overrides: the cell's
`scheduling:` and `speed:` sections, the feeder's parameters, the run's own command-line
overrides, and the logged arrivals replayed by the `script` feeder. `cell_overrides` does the
section part alone for any `config.yaml` (a spec's `cell_config:`).
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from modules.config_io import read_config

REPO_DIR = Path(__file__).resolve().parent.parent
# What the sandbox takes from a cell config; the cell hardware stays in sandbox/models/.
_SECTIONS = ("scheduling", "speed")


ARCHIVE = REPO_DIR / "log" / "archived.zip"
CACHE_DIR = REPO_DIR / "sandbox" / "results" / "cell_logs"


def resolve(path: str | Path) -> Path:
    """A path as written in a spec: absolute, or relative to the repository root."""
    path = Path(path)
    return path if path.is_absolute() else REPO_DIR / path


def resolve_run(path: str | Path) -> Path:
    """A logged run directory: where the spec says, or, when `log/` has been archived into
    `log/archived.zip`, a copy of the run (and its session's `config.yaml`) unpacked under
    `sandbox/results/cell_logs/`."""
    found = resolve(path)
    if found.is_dir():
        return found
    try:
        relative = found.relative_to(REPO_DIR / "log").as_posix()
    except ValueError:
        return found
    if not ARCHIVE.is_file():
        return found
    import zipfile

    session = relative.split("/")[0]
    with zipfile.ZipFile(ARCHIVE) as archive:
        names = archive.namelist()
        # Sessions logged before the archive was made sit in its `archived/` folder.
        for prefix in ("", "archived/"):
            run = prefix + relative + "/"
            wanted = [n for n in names if n.startswith(run) or n == f"{prefix}{session}/config.yaml"]
            if any(n.startswith(run) for n in wanted):
                for name in wanted:
                    if not (CACHE_DIR / name).exists():
                        archive.extract(name, CACHE_DIR)
                return CACHE_DIR / (prefix + relative)
    return found


def plain(value: Any) -> Any:
    """ruamel containers as plain dicts and lists."""
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value


def flatten(prefix: str, value: Any, out: dict[str, Any]) -> dict[str, Any]:
    if isinstance(value, dict):
        for key, item in value.items():
            flatten(f"{prefix}.{key}" if prefix else str(key), item, out)
    else:
        out[prefix] = plain(value)
    return out


def cell_overrides(config_path: str | Path) -> dict[str, Any]:
    """Dotted sandbox overrides carrying the `scheduling` / `speed` sections, the feeder's
    parameters and the stacking switch of a `modules/config.yaml`-shaped file."""
    return settings_overrides(read_config(resolve(config_path)))


def settings_overrides(raw: dict[str, Any]) -> dict[str, Any]:
    """The same from sections already in memory (a config file, or the `settings` a run's
    summary.json records)."""
    out: dict[str, Any] = {}
    for section in _SECTIONS:
        flatten(section, raw.get(section) or {}, out)
    feeder = raw.get("feeder") or {}
    if feeder.get("kind"):
        out["feeder.kind"] = str(feeder["kind"])
    for kind, params in (feeder.get("kinds") or {}).items():
        flatten(f"feeder.{kind}", params, out)
    if "stacking" in feeder:
        out["sim.stacking"] = bool(feeder["stacking"])
    return out


def _read_csv(path: Path) -> list[dict[str, str]]:
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


@dataclass
class CellRun:
    path: Path
    summary: dict[str, Any]
    parts: list[dict[str, str]]
    flow: list[dict[str, str]]

    @classmethod
    def load(cls, path: str | Path) -> "CellRun":
        path = resolve_run(path)
        missing = [name for name in ("summary.json", "parts.csv", "arrivals.csv")
                   if not (path / name).is_file()]
        if missing:
            raise SystemExit(f"{path}: not a simulate_feeder run directory (missing {', '.join(missing)})")
        flow = path / "flow.csv"
        return cls(path, json.loads((path / "summary.json").read_text()),
                   _read_csv(path / "parts.csv"), _read_csv(flow) if flow.is_file() else [])

    @property
    def session_config(self) -> Path:
        return self.path.parent / "config.yaml"

    @property
    def arrivals(self) -> Path:
        return self.path / "arrivals.csv"

    @property
    def duration_s(self) -> float:
        return float(self.summary["duration_s"])

    def overrides(self) -> dict[str, Any]:
        """The sandbox run that sees this run's input under this run's settings."""
        snapshot = self.summary.get("settings")
        # A run records its own settings; the session's config.yaml is the fallback for older logs
        # (and is stale once the console has edited the file between runs).
        out = settings_overrides(snapshot) if snapshot else cell_overrides(self.session_config)
        out.update({k: plain(v) for k, v in (self.summary.get("overrides") or {}).items()
                    if k.split(".")[0] in _SECTIONS})
        out.update({"feeder.kind": "script", "feeder.script.arrivals_csv": str(self.arrivals),
                    "sim.duration_s": float(math.ceil(self.duration_s)) + 1.0})
        out.setdefault("sim.stacking", True)
        return out

    def label(self) -> str:
        s = self.summary
        return f"{s.get('planner')} / {s.get('speed_law')} / {'aware' if s.get('stack_aware') else 'blind'}"
