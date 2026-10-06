"""Compare throughput across set-ups: `python3 -m modules.tools.log_compare log/`.

Finds every run under the folder you name (`summary.json`), groups them by planner, stack
awareness and speed law, and writes into `--out`:

* `compare.csv` — one row per group (runs, mean throughput, mean pick rate, mean input rate,
  outcome and miss-reason totals);
* `compare.png` — throughput per group (bar = mean, dots = runs), pick rate per group, and
  where each group's parts went (picked and every miss reason, as a share of its input).

Runs differing only in seed or input rate fall into one group, so keep the input comparable
(`--min-input` and `--max-input` filter on input parts/min).

    python3 -m modules.tools.log_compare log/ --scenario simulate_feeder
    python3 -m modules.tools.log_compare log/20261005-175405_console --out report/
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any

from modules.tools.flow_report import find_runs
from modules.tools.log_timeline import BLUE, GRID, INK, MUTED, load_matplotlib

# Fixed slots of the reference palette for the stacked outcomes: picked first, then the misses
# in the order they are met. Anything beyond the list folds into "other".
PICKED_COLOUR = BLUE
MISS_COLOURS = ["#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#4a3aa7", "#e34948"]
OTHER_COLOUR = "#8a8a85"


def group_name(summary: dict[str, Any]) -> str:
    aware = "aware" if summary.get("stack_aware") else "blind"
    return f"{summary.get('planner')} / {aware} / {summary.get('speed_law')}"


def collect(roots: list[Path], scenario: str | None, min_input: float, max_input: float) -> dict[str, list[dict[str, Any]]]:
    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for root in roots:
        for directory in find_runs([root]):
            try:
                summary = json.loads((directory / "summary.json").read_text())
            except (OSError, json.JSONDecodeError) as exc:
                print(f"[WARN] skipped {directory}: {exc}")
                continue
            if scenario and summary.get("scenario") != scenario:
                continue
            rate = summary.get("input_per_min")
            if summary.get("throughput_per_min") is None or rate is None or not min_input <= rate <= max_input:
                continue
            groups[group_name(summary)].append(summary)
    return dict(sorted(groups.items()))


def mean(values: list[float | None]) -> float | None:
    clean = [v for v in values if v is not None]
    return round(statistics.fmean(clean), 3) if clean else None


def group_rows(groups: dict[str, list[dict[str, Any]]]) -> list[dict[str, Any]]:
    rows = []
    for name, runs in groups.items():
        reasons: dict[str, int] = defaultdict(int)
        for run in runs:
            for key, count in (run.get("miss_reasons") or {}).items():
                reasons[key] += count
        rows.append({
            "group": name, "runs": len(runs),
            "input_per_min": mean([r.get("input_per_min") for r in runs]),
            "throughput_per_min": mean([r.get("throughput_per_min") for r in runs]),
            "pick_rate": mean([r.get("pick_rate") for r in runs]),
            "input": sum(r.get("input") or 0 for r in runs),
            "picked": sum(r.get("picked") or 0 for r in runs),
            "on_belt": sum(r.get("on_belt") or 0 for r in runs),
            "miss_reasons": json.dumps(dict(sorted(reasons.items())), ensure_ascii=True),
        })
    return rows


def plot(plt, groups: dict[str, list[dict[str, Any]]], rows: list[dict[str, Any]], path: Path) -> None:
    names = list(groups)
    y = list(range(len(names)))
    fig, (rate, pick, split) = plt.subplots(1, 3, figsize=(15, 1.6 + 0.7 * len(names) + 2.2), sharey=True,
                                            gridspec_kw={"width_ratios": [1, 1, 1.5]})
    for ax, key, title, scale in ((rate, "throughput_per_min", "Throughput (parts / min)", 1.0),
                                  (pick, "pick_rate", "Pick rate (% of parts that reached a verdict)", 100.0)):
        ax.barh(y, [(row[key] or 0.0) * scale for row in rows], height=0.55, color=BLUE, alpha=0.85)
        for i, name in enumerate(names):
            values = [r.get(key) for r in groups[name] if r.get(key) is not None]
            ax.scatter([v * scale for v in values], [i] * len(values), s=22, color=INK,
                       edgecolors="#fcfcfb", linewidths=1, zorder=3)
            if rows[i][key] is not None:
                reach = max([rows[i][key], *values]) * scale
                ax.annotate(f"{rows[i][key] * scale:.1f}", (reach, i), xytext=(8, 0),
                            textcoords="offset points", va="center", color=INK, fontsize=8)
        ax.set_title(title)
        ax.set_xlim(0, None)
        ax.grid(axis="y", visible=False)
    rate.set_yticks(y, [f"{n}\n({len(groups[n])} run{'s' if len(groups[n]) != 1 else ''})" for n in names], fontsize=8)
    rate.invert_yaxis()

    keys = sorted({k for row in rows for k in json.loads(row["miss_reasons"])},
                  key=lambda k: -sum(json.loads(r["miss_reasons"]).get(k, 0) for r in rows))
    shown, folded = keys[:len(MISS_COLOURS)], keys[len(MISS_COLOURS):]
    left = [0.0] * len(rows)
    inputs = [max(row["input"], 1) for row in rows]
    layers = [("picked", PICKED_COLOUR, [row["picked"] for row in rows])]
    layers += [(k, MISS_COLOURS[i], [json.loads(row["miss_reasons"]).get(k, 0) for row in rows])
               for i, k in enumerate(shown)]
    if folded:
        layers.append(("other misses", OTHER_COLOUR,
                       [sum(json.loads(row["miss_reasons"]).get(k, 0) for k in folded) for row in rows]))
    layers.append(("still on belt", GRID, [row["on_belt"] for row in rows]))
    for label, colour, counts in layers:
        share = [100.0 * c / n for c, n in zip(counts, inputs)]
        split.barh(y, share, left=left, height=0.55, color=colour, edgecolor="#fcfcfb", linewidth=1.5, label=label)
        left = [a + b for a, b in zip(left, share)]
    split.set_title("Where the parts went (% of input)")
    split.set_xlim(0, 100)
    split.grid(axis="y", visible=False)
    split.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncols=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("roots", nargs="*", default=["log"], help="folders to search for summary.json")
    parser.add_argument("--out", default="log/compare", help="output folder")
    parser.add_argument("--scenario", default=None, help="keep only runs of this scenario")
    parser.add_argument("--min-input", type=float, default=0.0, help="drop runs with less input (parts/min)")
    parser.add_argument("--max-input", type=float, default=float("inf"), help="drop runs with more input (parts/min)")
    args = parser.parse_args()

    groups = collect([Path(r) for r in args.roots], args.scenario, args.min_input, args.max_input)
    if not groups:
        print("no run records found (a run writes summary.json only with its run log enabled)")
        return 1
    rows = group_rows(groups)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "compare.csv", "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    plt = load_matplotlib()
    if plt is not None:
        plot(plt, groups, rows, out / "compare.png")
    print(f"{sum(len(g) for g in groups.values())} run(s) in {len(groups)} group(s) -> {out}/compare.csv"
          + (", compare.png" if plt is not None else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
