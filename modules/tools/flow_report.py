"""Input vs throughput across runs: collect every run's part record into tables and a figure.

Every scenario run with a run log writes `summary.json` and `flow.csv` (runtime/outcomes.py).
This finds them under the given folders and writes, into `--out`:

* `runs.csv`    — one row per run: set-up (scenario, planner, speed law, feeder, seed) and
                  totals (input, picked, misses by kind, rates, pick rate, contact error);
* `windows.csv` — every flow.csv bin of every run, labelled with its run's set-up;
* `flow_report.png` — throughput against input, one small point per time bin and one large
                  point per run, coloured by planner / speed law; and the outcome split per run.

    python3 -m modules.tools.flow_report log/ [--out report/] [--scenario simulate_feeder]
    python3 -m modules.tools.flow_report log/ --min-bin-s 60     # time bins of >= 60 s (default 30)
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

RUN_FIELDS = ("run", "scenario", "planner", "speed_law", "static_mm_s", "feed", "feeder_kind",
              "feeder_seed", "feeder_rate", "sim", "duration_s", "input", "picked", "miss_grip",
              "miss_late", "lost_track", "on_belt", "pick_rate", "input_per_min",
              "throughput_per_min", "mean_belt_mm_s", "median_error_mm", "miss_reasons")


def find_runs(roots: list[Path]) -> list[Path]:
    found = set()
    for root in roots:
        if (root / "summary.json").exists():
            found.add(root)
        found.update(p.parent for p in root.rglob("summary.json"))
    return sorted(found)


def _feeder_rate(feeder: dict[str, Any] | None) -> float | None:
    if not feeder:
        return None
    params = feeder.get("params") or {}
    for key in ("rate_per_min", "nominal_rate_per_min", "mean_rate_per_min"):
        if key in params:
            return params[key]
    if "interval_s" in params and params["interval_s"]:
        return round(60.0 / float(params["interval_s"]), 3)
    return None


def run_row(directory: Path, summary: dict[str, Any]) -> dict[str, Any]:
    feeder = summary.get("feeder") or {}
    row = {key: summary.get(key) for key in RUN_FIELDS}
    row.update({
        "run": str(directory),
        "feeder_kind": feeder.get("kind"),
        "feeder_seed": feeder.get("seed"),
        "feeder_rate": _feeder_rate(feeder),
        "miss_reasons": json.dumps(summary.get("miss_reasons") or {}, ensure_ascii=True),
    })
    return row


def read_bins(directory: Path, min_bin_s: float) -> list[dict[str, Any]]:
    path = directory / "flow.csv"
    if not path.exists():
        return []
    with open(path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    merged, acc = [], None
    for row in rows:
        t0, t1 = float(row["t_start"]), float(row["t_end"])
        if acc is None:
            acc = {"t_start": t0, "t_end": t1, "input": 0, "picked": 0, "miss_grip": 0, "miss_late": 0}
        acc["t_end"] = t1
        for key in ("input", "picked", "miss_grip", "miss_late"):
            acc[key] += int(row.get(key) or 0)
        if acc["t_end"] - acc["t_start"] >= min_bin_s:
            merged.append(acc)
            acc = None
    # A run's last bin is cut short by the run end; a few seconds make a wild rate.
    full = max((b["t_end"] - b["t_start"] for b in merged), default=0.0)
    merged = [b for b in merged if b["t_end"] - b["t_start"] >= 0.5 * full]
    for bin_ in merged:
        span = bin_["t_end"] - bin_["t_start"]
        bin_["input_per_min"] = round(60.0 * bin_["input"] / span, 3)
        bin_["throughput_per_min"] = round(60.0 * bin_["picked"] / span, 3)
    return merged


def label(row: dict[str, Any]) -> str:
    return f"{row.get('planner')} / {row.get('speed_law')}"


def plot(runs: list[dict[str, Any]], windows: list[dict[str, Any]], out: Path) -> Path | None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib not installed: no figure")
        return None
    labels = sorted({label(r) for r in runs})
    colours = {name: plt.cm.tab10(i % 10) for i, name in enumerate(labels)}
    fig, (ax, bars) = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": [1.1, 1]})

    top = 1.0
    for w in windows:
        ax.scatter(w["input_per_min"], w["throughput_per_min"], s=12, alpha=0.35,
                   color=colours[w["label"]], linewidths=0)
        top = max(top, w["input_per_min"], w["throughput_per_min"])
    for r in runs:
        if r.get("input_per_min") is None:
            continue
        ax.scatter(r["input_per_min"], r["throughput_per_min"] or 0.0, s=90, color=colours[label(r)],
                   edgecolors="black", linewidths=0.8, label=label(r))
        top = max(top, r["input_per_min"], r["throughput_per_min"] or 0.0)
    top *= 1.08
    ax.plot([0, top], [0, top], "--", color="grey", linewidth=1, label="throughput = input")
    ax.set_xlim(0, top)
    ax.set_ylim(0, top)
    ax.set_xlabel("input (parts/min)")
    ax.set_ylabel("throughput (parts/min)")
    ax.set_title("Throughput vs input (small: time bins, large: whole runs)")
    handles, names = ax.get_legend_handles_labels()
    unique = dict(zip(names, handles))
    ax.legend(unique.values(), unique.keys(), fontsize=8, loc="upper left")
    ax.grid(alpha=0.3)

    order = sorted(runs, key=lambda r: (label(r), r.get("input_per_min") or 0.0))
    names = [f"{label(r)}\nseed {r.get('feeder_seed')}  in {r.get('input_per_min')}/min" for r in order]
    left = [0.0] * len(order)
    for key, colour in (("picked", "#2dc653"), ("miss_grip", "#ff5a5f"), ("miss_late", "#ffb703"),
                        ("lost_track", "#8a93a3")):
        values = [float(r.get(key) or 0) for r in order]
        bars.barh(range(len(order)), values, left=left, color=colour, label=key)
        left = [a + b for a, b in zip(left, values)]
    bars.set_yticks(range(len(order)))
    bars.set_yticklabels(names, fontsize=7)
    bars.invert_yaxis()
    bars.set_xlabel("parts")
    bars.set_title("Outcome of every part, per run")
    bars.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    path = out / "flow_report.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("roots", nargs="*", default=["log"], help="folders to search for summary.json")
    parser.add_argument("--out", default="log/flow_report", help="output folder")
    parser.add_argument("--scenario", default=None, help="keep only runs of this scenario")
    parser.add_argument("--min-duration", type=float, default=0.0, help="drop runs shorter than this (s)")
    parser.add_argument("--min-bin-s", type=float, default=30.0,
                        help="merge consecutive flow.csv bins until each spans at least this long (s)")
    args = parser.parse_args()

    runs, windows = [], []
    for directory in find_runs([Path(r) for r in args.roots]):
        try:
            summary = json.loads((directory / "summary.json").read_text())
        except (OSError, json.JSONDecodeError) as exc:
            print(f"[WARN] skipped {directory}: {exc}")
            continue
        if args.scenario and summary.get("scenario") != args.scenario:
            continue
        if (summary.get("duration_s") or 0.0) < args.min_duration:
            continue
        row = run_row(directory, summary)
        runs.append(row)
        for bin_ in read_bins(directory, args.min_bin_s):
            windows.append({"run": row["run"], "label": label(row), "planner": row["planner"],
                            "speed_law": row["speed_law"], "feeder_seed": row["feeder_seed"], **bin_})
    if not runs:
        print("no run records found (a run writes summary.json only with its run log enabled)")
        return 1

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "runs.csv", "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(RUN_FIELDS))
        writer.writeheader()
        writer.writerows(runs)
    with open(out / "windows.csv", "w", newline="", encoding="utf-8") as handle:
        fields = list(windows[0]) if windows else ["run"]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(windows)
    figure = plot(runs, windows, out)
    print(f"{len(runs)} run(s), {len(windows)} bin(s) -> {out}/runs.csv, windows.csv"
          + (f", {figure.name}" if figure else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
