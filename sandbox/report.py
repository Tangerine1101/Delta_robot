"""Tables and charts of a finished sweep, driven by the `report:` block of its spec.

    report:
      title: Horizon W against load
      metrics: [pick_share, covered_share, mean_belt_mm_s]    # columns of the tables
      group: [rate_per_min, stack_aware, horizon_s]           # rows of the main table (default: all factors)
      labels: {horizon_s: {1000: "inf"}}                      # how a level is printed
      charts:
        - {kind: line, x: horizon_s, y: pick_share, series: stack_aware, facet: rate_per_min}
        - {kind: heatmap, x: horizon_s, y: rate_per_min, value: pick_share, facet: stack_aware}
        - {kind: bar, x: planner, y: picks_per_min, series: stack_aware}
      best:                                                   # which level of `over` is best
        - {over: horizon_s, metric: pick_share, group: [rate_per_min, stack_aware], tolerance: 0.005}

A factor is a setting that differs between the cases (a column of `runs.csv`, named in the
manifest). Everything is averaged over the seeds (and over any factor a chart does not show);
the bands and ± are one standard deviation over those runs. Writes `report.md`, `summary.csv`
and `chart_*.png` next to `runs.csv`; `python3 -m sandbox report DIR` redraws them.
"""

from __future__ import annotations

import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any

# name: (axis label, format) of the columns of runs.csv a report can use, and the derived ones.
METRICS: dict[str, tuple[str, str]] = {
    "pick_share": ("picked / arrived", "{:.3f}"),
    "covered_share": ("misses on covered parts / arrived", "{:.3f}"),
    "lost_share": ("left the workspace unpicked / arrived", "{:.3f}"),
    "bypassed_share": ("bypassed and then lost / arrived", "{:.3f}"),
    "picks_per_min": ("picks per minute", "{:.2f}"),
    "mean_belt_mm_s": ("mean belt speed (mm/s)", "{:.1f}"),
    "dip_s": ("time in throughput dips (s)", "{:.0f}"),
    "dips": ("throughput dips", "{:.1f}"),
    "idle_s": ("arm idle (s)", "{:.0f}"),
    "long_wait_s": ("arm waiting on long commits (s)", "{:.0f}"),
    "bypassed_lost": ("parts lost after being bypassed", "{:.1f}"),
    "covered_misses": ("misses on covered parts", "{:.1f}"),
    "median_abs_error_mm": ("median pick error (mm)", "{:.2f}"),
    "wall_s": ("wall time (s)", "{:.1f}"),
    "law_ms_mean": ("speed-law decision, mean (ms)", "{:.2f}"),
    "law_ms_p95": ("speed-law decision, 95th percentile (ms)", "{:.2f}"),
    "law_ms_max": ("speed-law decision, maximum (ms)", "{:.1f}"),
    "plan_ms_mean": ("commit planning, mean (ms)", "{:.2f}"),
    "plan_ms_p95": ("commit planning, 95th percentile (ms)", "{:.2f}"),
    "plan_ms_max": ("commit planning, maximum (ms)", "{:.1f}"),
    "plan_jobs_mean": ("parts in the queue when planning", "{:.1f}"),
    "plan_jobs_max": ("most parts in the queue when planning", "{:.0f}"),
}
DEFAULT_METRICS = ["pick_share", "picks_per_min", "covered_share", "lost_share", "mean_belt_mm_s", "dip_s"]


def _number(text: Any) -> Any:
    if isinstance(text, (int, float)):
        return text
    try:
        value = float(text)
    except (TypeError, ValueError):
        return text
    return int(value) if value.is_integer() and "." not in str(text) and "e" not in str(text).lower() else value


def read_runs(run_dir: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    manifest = json.loads((run_dir / "manifest.json").read_text())
    with open(run_dir / "runs.csv", newline="") as handle:
        rows = [{k: _number(v) for k, v in row.items()} for row in csv.DictReader(handle)]
    for row in rows:
        spawned = row.get("spawned") or 0
        for name, source in (("pick_share", "picked"), ("covered_share", "covered_misses"),
                             ("lost_share", "lost"), ("bypassed_share", "bypassed_lost")):
            row[name] = row[source] / spawned if spawned and isinstance(row.get(source), (int, float)) else ""
    return rows, manifest


def _key(value: Any) -> tuple[int, Any]:
    """Numbers sort as numbers, before text; booleans as False < True."""
    if isinstance(value, (int, float)):
        return (0, float(value))
    return (1, {"False": 0, "True": 1}.get(str(value), str(value)))


def label(labels: dict[str, dict[str, str]], factor: str, value: Any) -> str:
    table = {str(k): str(v) for k, v in (labels.get(factor) or {}).items()}
    if str(value) in table:
        return table[str(value)]
    if isinstance(value, float) and value.is_integer():
        value = int(value)
    return str(value)


def stats(rows: list[dict[str, Any]], metric: str) -> tuple[float, float, int]:
    values = [r[metric] for r in rows if isinstance(r.get(metric), (int, float))]
    if not values:
        return float("nan"), float("nan"), 0
    return statistics.fmean(values), (statistics.stdev(values) if len(values) > 1 else 0.0), len(values)


def aggregate(rows: list[dict[str, Any]], by: list[str]) -> dict[tuple, list[dict[str, Any]]]:
    groups: dict[tuple, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[tuple(row.get(f, "") for f in by)].append(row)
    return dict(sorted(groups.items(), key=lambda item: tuple(_key(v) for v in item[0])))


def _fmt(metric: str, value: float) -> str:
    return "-" if value != value else METRICS.get(metric, ("", "{:.2f}"))[1].format(value)


def main_table(rows, by, metrics, labels) -> tuple[list[str], list[dict[str, Any]]]:
    """(markdown lines, summary rows) of the means over the seeds, one line per combination of `by`."""
    summary: list[dict[str, Any]] = []
    lines = ["| " + " | ".join([*by, "runs", *metrics]) + " |", "|" + "---|" * (len(by) + 1 + len(metrics))]
    for key, group in aggregate(rows, by).items():
        record: dict[str, Any] = dict(zip(by, key))
        cells = [label(labels, f, v) for f, v in zip(by, key)] + [str(len(group))]
        record["runs"] = len(group)
        for metric in metrics:
            mean, sd, _ = stats(group, metric)
            record[f"{metric}_mean"], record[f"{metric}_sd"] = mean, sd
            cells.append(f"{_fmt(metric, mean)} ± {_fmt(metric, sd)}" if len(group) > 1 else _fmt(metric, mean))
        lines.append("| " + " | ".join(cells) + " |")
        summary.append(record)
    return lines, summary


def best_table(rows, spec: dict[str, Any], labels) -> list[str]:
    """For each combination of `group`, the level of `over` with the best mean `metric`
    (`goal`: max, default, or min) and the levels within `tolerance` of it (the plateau)."""
    over, metric = spec["over"], spec["metric"]
    group, tol = list(spec.get("group") or []), float(spec.get("tolerance", 0.0))
    sign = -1.0 if spec.get("goal", "max") == "min" else 1.0
    lines = [f"| {' | '.join(group)} | best {over} | {metric} | within {tol:g} of the best |",
             "|" + "---|" * (len(group) + 3)]
    for key, sub in aggregate(rows, group).items():
        means = {level: stats(g, metric)[0] for level, g in aggregate(sub, [over]).items()}
        means = {k[0]: v for k, v in means.items() if v == v}
        if not means:
            continue
        top = max(means.values(), key=lambda v: sign * v)
        plateau = [lv for lv, v in means.items() if sign * (top - v) <= tol]
        best_level = [lv for lv, v in means.items() if v == top][0]
        lines.append("| " + " | ".join([*(label(labels, f, v) for f, v in zip(group, key)),
                                        label(labels, over, best_level), _fmt(metric, top),
                                        ", ".join(label(labels, over, lv) for lv in plateau)]) + " |")
    return lines


# ---- charts ------------------------------------------------------------------------------
def _plt():
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:       # pragma: no cover
        raise SystemExit(f"charts need matplotlib: run with .venv/bin/python ({exc})")
    return plt


def _axes(plt, n: int, width: float = 4.6, height: float = 3.6):
    cols = min(n, 3)
    rows = -(-n // cols)
    fig, axes = plt.subplots(rows, cols, figsize=(width * cols, height * rows), squeeze=False)
    flat = list(axes.ravel())
    for extra in flat[n:]:
        extra.axis("off")
    return fig, flat[:n]


def _metric_label(metric: str) -> str:
    return METRICS.get(metric, (metric, ""))[0]


def chart_line(rows, spec, labels, path: Path) -> None:
    plt = _plt()
    x, y, series, facet = spec["x"], spec["y"], spec.get("series"), spec.get("facet")
    levels = [k[0] for k in aggregate(rows, [x])]
    panels = list(aggregate(rows, [facet]).items()) if facet else [((), rows)]
    fig, axes = _axes(plt, len(panels))
    for ax, (fkey, sub) in zip(axes, panels):
        for skey, ssub in (aggregate(sub, [series]).items() if series else [((), sub)]):
            pts = [(i, *stats(g, y)[:2]) for i, level in enumerate(levels)
                   for g in [aggregate(ssub, [x]).get((level,), [])] if g]
            if not pts:
                continue
            xs, mean, sd = zip(*pts)
            name = f"{series}={label(labels, series, skey[0])}" if series else y
            line, = ax.plot(xs, mean, marker="o", ms=4, label=name)
            ax.fill_between(xs, [m - s for m, s in zip(mean, sd)], [m + s for m, s in zip(mean, sd)],
                            alpha=0.15, color=line.get_color())
        ax.set_xticks(range(len(levels)), [label(labels, x, v) for v in levels])
        ax.set_xlabel(x)
        ax.set_ylabel(spec.get("ylabel") or _metric_label(y))
        if spec.get("ylim"):
            ax.set_ylim(*spec["ylim"])
        ax.grid(alpha=0.3)
        if facet:
            ax.set_title(f"{facet} = {label(labels, facet, fkey[0])}")
        if series:
            ax.legend(fontsize=8)
    fig.suptitle(spec.get("title") or f"{_metric_label(y)} against {x}")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def chart_heatmap(rows, spec, labels, path: Path) -> None:
    plt = _plt()
    x, y, value, facet = spec["x"], spec["y"], spec["value"], spec.get("facet")
    xs = [k[0] for k in aggregate(rows, [x])]
    ys = [k[0] for k in aggregate(rows, [y])]
    panels = list(aggregate(rows, [facet]).items()) if facet else [((), rows)]
    fig, axes = _axes(plt, len(panels), width=5.2)
    cells = {(fk, k): stats(g, value)[0] for fk, sub in panels for k, g in aggregate(sub, [x, y]).items()}
    finite = [v for v in cells.values() if v == v]
    lo, hi = spec.get("vmin", min(finite, default=0)), spec.get("vmax", max(finite, default=1))
    image = None
    for ax, (fkey, _) in zip(axes, panels):
        grid = [[cells.get((fkey, (xv, yv)), float("nan")) for xv in xs] for yv in ys]
        image = ax.imshow(grid, origin="lower", aspect="auto", vmin=lo, vmax=hi, cmap=spec.get("cmap", "viridis"))
        ax.set_xticks(range(len(xs)), [label(labels, x, v) for v in xs])
        ax.set_yticks(range(len(ys)), [label(labels, y, v) for v in ys])
        ax.set_xlabel(x)
        ax.set_ylabel(y)
        for r, yv in enumerate(ys):
            for c, xv in enumerate(xs):
                v = grid[r][c]
                if v == v:
                    ax.text(c, r, _fmt(value, v), ha="center", va="center", fontsize=7,
                            color="white" if (v - lo) / ((hi - lo) or 1) < 0.55 else "black")
        if facet:
            ax.set_title(f"{facet} = {label(labels, facet, fkey[0])}")
    fig.colorbar(image, ax=axes, shrink=0.85, label=_metric_label(value))
    fig.suptitle(spec.get("title") or f"{_metric_label(value)}")
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)


def chart_bar(rows, spec, labels, path: Path) -> None:
    plt = _plt()
    x, y, series = spec["x"], spec["y"], spec.get("series")
    levels = [k[0] for k in aggregate(rows, [x])]
    groups = [k[0] for k in aggregate(rows, [series])] if series else [None]
    fig, (ax,) = _axes(plt, 1, width=max(4.6, 1.1 * len(levels) * len(groups) / 2 + 2))
    width = 0.8 / len(groups)
    for g, gv in enumerate(groups):
        means, sds = [], []
        for level in levels:
            sub = [r for r in rows if r.get(x) == level and (series is None or r.get(series) == gv)]
            mean, sd, _ = stats(sub, y)
            means.append(mean)
            sds.append(sd)
        ax.bar([i + g * width - 0.4 + width / 2 for i in range(len(levels))], means, width, yerr=sds,
               capsize=2, label=f"{series}={label(labels, series, gv)}" if series else y)
    ax.set_xticks(range(len(levels)), [label(labels, x, v) for v in levels])
    ax.set_xlabel(x)
    ax.set_ylabel(spec.get("ylabel") or _metric_label(y))
    if spec.get("ylim"):
        ax.set_ylim(*spec["ylim"])
    ax.grid(alpha=0.3, axis="y")
    if series:
        ax.legend(fontsize=8)
    fig.suptitle(spec.get("title") or f"{_metric_label(y)} by {x}")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


CHARTS = {"line": chart_line, "heatmap": chart_heatmap, "bar": chart_bar}


def build_report(run_dir: str | Path) -> Path:
    """Write `report.md`, `summary.csv` and the charts of a sweep directory; returns the report."""
    run_dir = Path(run_dir)
    rows, manifest = read_runs(run_dir)
    spec = manifest["spec_full"].get("report") or {}
    factors = list(manifest.get("factors") or {})
    labels = spec.get("labels") or {}
    metrics = list(spec.get("metrics") or DEFAULT_METRICS)
    by = list(spec.get("group") or factors)
    for name in [*by, *metrics]:
        if name not in rows[0]:
            raise SystemExit(f"report: '{name}' is not a column of runs.csv (columns: {', '.join(rows[0])})")

    lines = [f"# {spec.get('title') or manifest.get('spec')}", "", str(manifest.get("description") or ""), "",
             f"{len(rows)} runs, {len(manifest['cases'])} cases x {len(manifest['seeds'])} seeds"
             f"{' (quick)' if manifest.get('quick') else ''}, wall {manifest.get('wall_s')} s, "
             f"git `{manifest.get('git')}`. Mean ± one standard deviation over the seeds.", ""]
    for note in spec.get("notes") or []:
        lines += [str(note), ""]
    for n, chart in enumerate(spec.get("charts") or [], 1):
        kind = chart.get("kind", "line")
        if kind not in CHARTS:
            raise SystemExit(f"report: unknown chart kind '{kind}' (known: {', '.join(CHARTS)})")
        name = f"chart_{n}_{chart.get('value') or chart.get('y')}.png"
        CHARTS[kind](rows, chart, labels, run_dir / name)
        lines += [f"![{name}]({name})", ""]
    for best in spec.get("best") or []:
        lines += [f"## Best {best['over']} by {best['metric']}", "", *best_table(rows, best, labels), ""]
    table, summary = main_table(rows, by, metrics, labels)
    lines += ["## All combinations", "", *table, ""]
    (run_dir / "report.md").write_text("\n".join(lines))
    with open(run_dir / "summary.csv", "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)
    print(f"[REPORT] {run_dir / 'report.md'}")
    return run_dir / "report.md"
