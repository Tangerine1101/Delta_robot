"""Throughput over time, one figure per run: `python3 -m modules.tools.log_timeline log/<session>`.

Reads the part record a scenario run writes (`flow.csv`, `parts.csv`, `summary.json`; see
doc/basis-programming.md §2.5) from the folder you name: a single run folder, a session folder
holding several runs, or `log/` itself. Writes `<run>_timeline.png` per run into `--out`:

* input and throughput (parts/min) per flow bin;
* the belt speed per bin;
* the density (parts between the camera origin and u_max) per bin;
* cumulative parts seen against cumulative parts picked, from `parts.csv`.

    python3 -m modules.tools.log_timeline log/20261005-175405_console
    python3 -m modules.tools.log_timeline log/ --out report/ --smooth 3 --min-duration 60
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

from modules.tools.flow_report import find_runs

# Categorical slots 1-3 of the reference palette (light surface); text stays in ink tokens.
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def load_matplotlib():
    """pyplot on the Agg backend, or None when matplotlib is missing."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib not installed: no figure")
        return None
    plt.rcParams.update({
        "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb",
        "axes.edgecolor": GRID, "axes.labelcolor": MUTED, "text.color": INK,
        "xtick.color": MUTED, "ytick.color": MUTED, "axes.grid": True, "grid.color": GRID,
        "grid.linewidth": 0.8, "axes.spines.top": False, "axes.spines.right": False,
        "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlelocation": "left",
        "axes.axisbelow": True, "legend.frameon": False, "font.size": 9,
    })
    return plt


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def number(value: str | None) -> float | None:
    try:
        return float(value) if value not in (None, "") else None
    except ValueError:
        return None


def smooth(values: list[float | None], width: int) -> list[float | None]:
    """Trailing mean over `width` bins (1 = none); a missing bin is skipped."""
    if width <= 1:
        return values
    out: list[float | None] = []
    for i in range(len(values)):
        window = [v for v in values[max(0, i - width + 1):i + 1] if v is not None]
        out.append(sum(window) / len(window) if window else None)
    return out


def cumulative(times: list[float], end: float) -> tuple[list[float], list[int]]:
    """Step series (t, count) of events at `times`, closed at `end`."""
    ordered = sorted(times)
    return [0.0, *ordered, max(end, ordered[-1] if ordered else 0.0)], [0, *range(1, len(ordered) + 1), len(ordered)]


def run_title(summary: dict[str, Any]) -> str:
    aware = "stack-aware" if summary.get("stack_aware") else "stack-blind"
    feeder = summary.get("feeder") or {}
    return (f"{summary.get('scenario')} · {summary.get('planner')} · {aware} · {summary.get('speed_law')}"
            + (f" · {feeder['kind']} seed {feeder.get('seed')}" if feeder else ""))


def plot_run(plt, directory: Path, summary: dict[str, Any], out: Path, width: int) -> Path | None:
    bins = read_csv(directory / "flow.csv")
    parts = read_csv(directory / "parts.csv")
    if not bins:
        print(f"[WARN] {directory}: no flow.csv, skipped")
        return None
    mid = [(float(b["t_start"]) + float(b["t_end"])) / 2.0 for b in bins]
    series = {key: smooth([number(b.get(key)) for b in bins], width)
              for key in ("input_per_min", "throughput_per_min", "mean_belt_mm_s", "mean_density")}
    end = float(bins[-1]["t_end"])

    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True,
                             gridspec_kw={"height_ratios": [1.3, 0.8, 0.8, 1.3]})
    rate, belt, density, total = axes
    rate.plot(mid, series["input_per_min"], color=ORANGE, linewidth=2, label="input")
    rate.plot(mid, series["throughput_per_min"], color=BLUE, linewidth=2, label="throughput")
    rate.set_ylabel("parts / min")
    rate.set_title("Input and throughput")
    rate.legend(loc="upper right", ncols=2)
    belt.plot(mid, series["mean_belt_mm_s"], color=AQUA, linewidth=2)
    belt.set_ylabel("mm / s")
    belt.set_title("Belt speed")
    density.plot(mid, series["mean_density"], color=INK, linewidth=2)
    density.set_ylabel("parts")
    density.set_title("Density in the picking region")

    seen = [float(r["t_seen"]) for r in parts if number(r.get("t_seen")) is not None]
    picked = [number(r.get("t_contact")) if number(r.get("t_contact")) is not None else number(r.get("t_closed"))
              for r in parts if r.get("outcome") == "picked"]
    for times, colour, name in ((seen, ORANGE, "seen"), ([t for t in picked if t is not None], BLUE, "picked")):
        x, y = cumulative(times, end)
        total.step(x, y, where="post", color=colour, linewidth=2, label=name)
        if y:
            total.annotate(f"{y[-1]}", (x[-1], y[-1]), xytext=(4, 0), textcoords="offset points",
                           color=INK, va="center")
    total.set_ylabel("parts")
    total.set_xlabel("run time (s)")
    total.set_title("Cumulative parts seen and picked")
    total.legend(loc="upper left", ncols=2)
    total.set_xlim(0, end * 1.04)
    fig.suptitle(run_title(summary), x=0.01, ha="left", fontsize=11, color=INK)
    fig.tight_layout()
    path = out / f"{'_'.join(directory.parts[-2:])}_timeline.png"
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("root", nargs="?", default="log", help="a run folder, a session folder or log/")
    parser.add_argument("--out", default=None, help="output folder (default: <root>/timelines)")
    parser.add_argument("--scenario", default=None, help="keep only runs of this scenario")
    parser.add_argument("--min-duration", type=float, default=0.0, help="drop runs shorter than this (s)")
    parser.add_argument("--smooth", type=int, default=1, help="trailing mean over this many flow bins")
    args = parser.parse_args()

    root = Path(args.root)
    if not root.exists():
        print(f"{root} does not exist")
        return 1
    plt = load_matplotlib()
    if plt is None:
        return 1
    out = Path(args.out) if args.out else (root if root.is_dir() else root.parent) / "timelines"
    out.mkdir(parents=True, exist_ok=True)
    written = 0
    for directory in find_runs([root]):
        try:
            summary = json.loads((directory / "summary.json").read_text())
        except (OSError, json.JSONDecodeError) as exc:
            print(f"[WARN] skipped {directory}: {exc}")
            continue
        if args.scenario and summary.get("scenario") != args.scenario:
            continue
        if (summary.get("duration_s") or 0.0) < args.min_duration:
            continue
        path = plot_run(plt, directory, summary, out, args.smooth)
        if path is not None:
            written += 1
            print(path)
    print(f"{written} timeline(s) -> {out}")
    return 0 if written else 1


if __name__ == "__main__":
    raise SystemExit(main())
