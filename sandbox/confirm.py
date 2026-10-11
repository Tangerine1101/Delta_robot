"""Select on one set of seeds, confirm on another, and say how many seeds the cell needs.

Choosing the best of many levels (a horizon W, a belt speed) on some seeds and then measuring that
level on the same seeds reports the luck of the choice together with its merit. This module
keeps the two apart:

* **select** — the seeds `--select-seeds` of the selection experiment give, per speed-law family
  (`predictive_rank` over `horizon_s`, `constant` over `static_mm_s`), the best level and a level
  well below and well above it (`--gap`: at least that many points of `pick_share` worse);
* **confirm** — on the seeds `--confirm-seeds` of the confirmation experiment, the chosen levels
  are compared by paired differences (the same seed, the same arrivals): the mean, the standard
  deviation of the difference `sd_d`, a 95 % interval, the number of seeds (at least 3) that would detect that
  difference on the cell (80 % power, two-sided 5 %, `sd_d` inflated by `--inflate` because the
  sandbox has no jitter), and the smallest difference 3, 5 or 10 cell seeds can detect.

    python3 -m sandbox confirm --select p1_select --confirm p1_confirm
    python3 -m sandbox confirm --select p1_regime --select-seeds 2001-2010 \\
        --confirm p1_regime --confirm-seeds 3001-3010 --out report/regime

An experiment argument is a result folder or the name of an experiment (its newest folder).
Writes `confirmation.md`, `contrasts.csv`, `chart_curves.png` and `chart_contrasts.png`.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path
from typing import Any

from sandbox.report import _plt, read_runs
from sandbox.sweep import RESULTS_DIR

Z_POWER = 1.959964 + 0.841621          # two-sided 5 %, 80 % power
T975 = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145,
        2.131, 2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048,
        2.045, 2.042]
FAMILIES = {"predictive_rank": ("W", "horizon_s"), "constant": ("c", "static_mm_s")}
INF_W = 1000.0
MIN_SEEDS = 3                          # the normal approximation is not trusted below this


def parse_seeds(text: str | None) -> set[int] | None:
    """`2001-2020`, `3001,3002` or a mix; None = every seed."""
    if not text:
        return None
    seeds: set[int] = set()
    for part in text.split(","):
        lo, _, hi = part.partition("-")
        seeds.update(range(int(lo), int(hi or lo) + 1))
    return seeds


def resolve(arg: str) -> Path:
    """A result folder, or the newest full (not `-quick`) folder of the experiment of that name."""
    path = Path(arg)
    if (path / "runs.csv").is_file():
        return path
    found = sorted(p for p in RESULTS_DIR.glob(f"{arg}-*") if (p / "runs.csv").is_file()
                   and p.name[len(arg) + 1:len(arg) + 2].isdigit())
    if not found:
        raise SystemExit(f"{arg}: no result folder with runs.csv (run the experiment first)")
    return found[-1]


def load(arg: str, seeds: set[int] | None) -> tuple[list[dict[str, Any]], Path]:
    folder = resolve(arg)
    rows, _ = read_runs(folder)
    return [r for r in rows if seeds is None or int(r["seed"]) in seeds], folder


def level_name(family: str, value: float) -> str:
    letter = FAMILIES[family][0]
    return f"{letter} = ∞" if family == "predictive_rank" and value >= INF_W else f"{letter} = {value:g}"


def family_rows(rows, planner: str, family: str, metric: str) -> dict[float, dict[int, float]]:
    """{level: {seed: metric}} of one planner's runs of one law."""
    out: dict[float, dict[int, float]] = {}
    column = FAMILIES[family][1]
    for r in rows:
        if r.get("law", family) != family or r.get("planner", planner) != planner:
            continue
        if not isinstance(r.get(metric), (int, float)) or not isinstance(r.get(column), (int, float)):
            continue
        out.setdefault(float(r[column]), {})[int(r["seed"])] = float(r[metric])
    return out


def _law_rows(rows, family):
    return [r for r in rows if r.get("law") == family]


def mean_of(values: dict[int, float]) -> float:
    return statistics.fmean(values.values()) if values else float("nan")


def choose(levels: dict[float, dict[int, float]], gap: float) -> dict[str, float]:
    """The best level by mean, and the nearest level on each side that is `gap` worse (the
    extreme level when none is)."""
    means = {level: mean_of(v) for level, v in levels.items()}
    ordered = sorted(means)
    best = max(ordered, key=lambda level: means[level])
    below = [lv for lv in ordered if lv < best]
    above = [lv for lv in ordered if lv > best]
    low = next((lv for lv in reversed(below) if means[lv] <= means[best] - gap), below[0] if below else best)
    high = next((lv for lv in above if means[lv] <= means[best] - gap), above[-1] if above else best)
    return {"low": low, "best": best, "high": high}


def paired(a: dict[int, float], b: dict[int, float], inflate: float) -> dict[str, Any]:
    seeds = sorted(set(a) & set(b))
    diff = [a[s] - b[s] for s in seeds]
    n = len(diff)
    mean = statistics.fmean(diff) if diff else float("nan")
    sd = statistics.stdev(diff) if n > 1 else float("nan")
    t = T975[n - 2] if 2 <= n <= 31 else 2.0
    half = t * sd / math.sqrt(n) if n > 1 else float("nan")
    needed = max(MIN_SEEDS, math.ceil((Z_POWER * sd * inflate / abs(mean)) ** 2)) if n > 1 and mean else float("inf")
    return {"n": n, "mean": mean, "sd_d": sd, "ci_lo": mean - half, "ci_hi": mean + half, "needed": needed,
            **{f"mdd_{k}": Z_POWER * sd * inflate / math.sqrt(k) for k in (3, 5, 10)}}


def run(select: str, confirm: str, *, select_seeds: str | None = None, confirm_seeds: str | None = None,
        planner: str = "dp", gap: float = 0.03, inflate: float = 1.5, metric: str = "pick_share",
        out: Path | None = None) -> Path:
    sel_rows, sel_dir = load(select, parse_seeds(select_seeds))
    con_rows, con_dir = load(confirm, parse_seeds(confirm_seeds))
    out = out or RESULTS_DIR / f"confirmation-{sel_dir.name}-{con_dir.name}"[:150]
    out.mkdir(parents=True, exist_ok=True)

    chosen: dict[str, dict[str, float]] = {}
    sel_levels: dict[str, dict[float, dict[int, float]]] = {}
    con_levels: dict[str, dict[float, dict[int, float]]] = {}
    for family in FAMILIES:
        sel_levels[family] = family_rows(_law_rows(sel_rows, family), planner, family, metric)
        con_levels[family] = family_rows(_law_rows(con_rows, family), planner, family, metric)
        if sel_levels[family]:
            chosen[family] = choose(sel_levels[family], gap)
    if "predictive_rank" not in chosen or "constant" not in chosen:
        raise SystemExit("the selection experiment needs both laws (law = predictive_rank and constant)")
    W, C = chosen["predictive_rank"], chosen["constant"]
    pr, cn = con_levels["predictive_rank"], con_levels["constant"]

    def at(family: str, level: float) -> dict[int, float]:
        return con_levels[family].get(level, {})

    contrasts: list[tuple[str, dict[str, Any]]] = []

    def add(name: str, a: dict[int, float], b: dict[int, float]) -> None:
        if a and b and a is not b and name not in {n for n, _ in contrasts}:
            contrasts.append((name, paired(a, b, inflate)))

    pw, cw = at("predictive_rank", W["best"]), at("constant", C["best"])
    add(f"predictive_rank {level_name('predictive_rank', W['best'])} − constant {level_name('constant', C['best'])}", pw, cw)
    for tag in ("low", "high"):
        if W[tag] != W["best"]:
            add(f"{level_name('predictive_rank', W['best'])} − {level_name('predictive_rank', W[tag])}", pw,
                at("predictive_rank", W[tag]))
    if INF_W in pr and W["best"] != INF_W:
        add(f"{level_name('predictive_rank', W['best'])} − {level_name('predictive_rank', INF_W)}", pw, pr[INF_W])
    for tag in ("low", "high"):
        if C[tag] != C["best"]:
            add(f"{level_name('constant', C['best'])} − {level_name('constant', C[tag])}", cw, at("constant", C[tag]))
    others = sorted({str(r["planner"]) for r in con_rows if r.get("planner") not in (None, "", planner)})
    for other in others:
        for family, level, label in (("predictive_rank", W["best"], "predictive_rank"), ("constant", C["best"], "constant")):
            theirs = family_rows(_law_rows(con_rows, family), other, family, metric).get(level, {})
            add(f"{planner} − {other}, {label} {level_name(family, level)}", at(family, level), theirs)

    lines = [f"# Select and confirm — {metric}", "",
             f"Selection: `{sel_dir.name}`, seeds {select_seeds or 'all'}, planner `{planner}`. "
             f"Confirmation: `{con_dir.name}`, seeds {confirm_seeds or 'all'}. A level counts as well below / above "
             f"the best when it is at least {gap:.3f} worse. `sd_d` is inflated by {inflate:g} for the cell's jitter.", "",
             "## Chosen levels (selection seeds)", "",
             "| law | well below | best | well above |", "|---|---|---|---|"]
    for family, levels in chosen.items():
        lines.append(f"| {family} | " + " | ".join(level_name(family, levels[k]) for k in ("low", "best", "high")) + " |")
    lines += ["", "## Every level, selection against confirmation seeds", "",
              "| law | level | selection mean ± sd | n | confirmation mean ± sd | n |", "|---|---|---|---|---|---|"]
    for family in FAMILIES:
        for level in sorted(sel_levels[family]):
            s_vals, c_vals = list(sel_levels[family][level].values()), list(con_levels[family].get(level, {}).values())
            fmt = lambda v: f"{statistics.fmean(v):.3f} ± {statistics.stdev(v) if len(v) > 1 else 0.0:.3f}" if v else "–"  # noqa: E731
            mark = " ◀" if level == chosen[family]["best"] else ""
            lines.append(f"| {family} | {level_name(family, level)}{mark} | {fmt(s_vals)} | {len(s_vals)} | {fmt(c_vals)} | {len(c_vals)} |")
    lines += ["", "## Paired differences on the confirmation seeds", "",
              "| a − b | seeds | mean | sd_d | 95 % interval | seeds for the cell | smallest detectable at 3 / 5 / 10 cell seeds |",
              "|---|---|---|---|---|---|---|"]
    for name, c in contrasts:
        needed = "–" if math.isinf(c["needed"]) else str(c["needed"])
        lines.append(f"| {name} | {c['n']} | {c['mean']:+.4f} | {c['sd_d']:.4f} | [{c['ci_lo']:+.4f}, {c['ci_hi']:+.4f}] | "
                     f"{needed} | {c['mdd_3']:.3f} / {c['mdd_5']:.3f} / {c['mdd_10']:.3f} |")
    lines += ["", "Seeds for the cell: `max(3, ceil((2.80 · sd_d · inflate / |mean|)²))`, the paired test at 80 % power and 5 % "
              "two-sided. A difference whose interval holds 0 here needs many seeds; do not chase it on the cell.", "",
              "![curves](chart_curves.png)", "", "![contrasts](chart_contrasts.png)", ""]
    (out / "confirmation.md").write_text("\n".join(lines))
    with open(out / "contrasts.csv", "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("contrast", "n", "mean", "sd_d", "ci_lo", "ci_hi", "seeds_for_cell", "mdd3", "mdd5", "mdd10"))
        writer.writerows((n, c["n"], c["mean"], c["sd_d"], c["ci_lo"], c["ci_hi"], c["needed"], c["mdd_3"], c["mdd_5"],
                          c["mdd_10"]) for n, c in contrasts)
    _charts(sel_levels, con_levels, chosen, contrasts, metric, out)
    print(f"[CONFIRM] {out / 'confirmation.md'}")
    return out / "confirmation.md"


def _charts(sel_levels, con_levels, chosen, contrasts, metric: str, out: Path) -> None:
    plt = _plt()
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8), sharey=True)
    for axis, family in zip(axes, FAMILIES):
        for levels, marker, name, shift in ((sel_levels[family], "o", "selection seeds", 0.0),
                                            (con_levels[family], "^", "confirmation seeds", 0.0)):
            xs = sorted(levels)
            ys = [mean_of(levels[x]) for x in xs]
            sd = [statistics.stdev(levels[x].values()) if len(levels[x]) > 1 else 0.0 for x in xs]
            if xs:
                plot_x = [min(x, 150.0) if family == "predictive_rank" else x for x in xs]
                axis.errorbar(plot_x, ys, sd, marker=marker, ms=4, lw=1.4, capsize=2, label=name)
        for tag, style in (("low", ":"), ("best", "--"), ("high", ":")):
            value = chosen[family][tag]
            axis.axvline(min(value, 150.0) if family == "predictive_rank" else value, color="0.5", ls=style, lw=1)
        axis.set(title=f"{family} ({FAMILIES[family][1]})", xlabel=FAMILIES[family][1] + (" (150 = ∞)" if family == "predictive_rank" else ""))
        axis.grid(alpha=0.3)
    axes[0].set_ylabel(metric)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "chart_curves.png", dpi=130)
    plt.close(fig)
    if contrasts:
        fig, axis = plt.subplots(figsize=(9, 0.5 * len(contrasts) + 1.2))
        for i, (name, c) in enumerate(reversed(contrasts)):
            axis.errorbar([c["mean"]], [i], xerr=[[c["mean"] - c["ci_lo"]], [c["ci_hi"] - c["mean"]]], fmt="o", capsize=3)
        axis.axvline(0, color="0.4", lw=1)
        axis.set_yticks(range(len(contrasts)), [n for n, _ in reversed(contrasts)], fontsize=8)
        axis.set_xlabel(f"paired difference of {metric} (mean, 95 % interval)")
        axis.grid(alpha=0.3, axis="x")
        fig.tight_layout()
        fig.savefig(out / "chart_contrasts.png", dpi=130)
        plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--select", required=True)
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--select-seeds")
    parser.add_argument("--confirm-seeds")
    parser.add_argument("--planner", default="dp")
    parser.add_argument("--gap", type=float, default=0.03)
    parser.add_argument("--inflate", type=float, default=1.5)
    parser.add_argument("--metric", default="pick_share")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    run(args.select, args.confirm, select_seeds=args.select_seeds, confirm_seeds=args.confirm_seeds,
        planner=args.planner, gap=args.gap, inflate=args.inflate, metric=args.metric, out=args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
