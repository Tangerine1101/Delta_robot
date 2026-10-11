"""Phase 0: does the sandbox reproduce the cell? Replay logged cell runs and compare.

A spec with `kind: replay` lists cell runs; each is replayed by the sandbox on the run's own
arrivals (`arrivals.csv`, the `script` feeder) under the run's own settings (the session's
`config.yaml` plus the run's command-line overrides), and the two part records are compared:

    kind: replay
    description: ...
    gates: {picks_rel: 0.05, belt_rel: 0.10, agree: 0.85}
    runs:
      - name: dp_w15
        cell_run: log/20261008-163559_console/run02_164527_simulate_feeder
        set: {sim.carry_scale: 0.8}     # optional overrides; with `set` the entry is a what-if,
                                        # not gated, unless it says `gated: true`
    quick: {duration_s: 900}

Parts are matched by arrival order (the cell calls them vf0001.., the sandbox p00000..); both
see the same arrivals, so part n is the same part. Compared: picks and belt speed over the
parts that arrived, the outcome of every part, and where the throughput dips. The cell's
latency, jitter and absolute clock are not modelled, so single parts diverge (the planner is
chaotic); the gates are on aggregates, `agree` on the share of parts with the same outcome.
"""

from __future__ import annotations

import csv
import json
import statistics
from datetime import datetime
from pathlib import Path
from typing import Any

from modules.config_io import read_config
from sandbox.cellrun import CellRun, plain
from sandbox.flow import find_dips, flow_series
from sandbox.sweep import RESULTS_DIR, _git_revision, simulate, write_run

CATEGORIES = ("picked", "covered", "grip_other", "late", "unfinished")
DEFAULT_GATES = {"picks_rel": 0.05, "belt_rel": 0.10, "agree": 0.85}


def category(row: dict[str, Any]) -> str:
    """The outcome of a part in the vocabulary both sides share."""
    outcome, reason = str(row.get("outcome", "")), str(row.get("reason", ""))
    if outcome == "picked":
        return "picked"
    if outcome == "miss_grip":
        return "covered" if reason == "covered" else "grip_other"
    if outcome in ("miss_late", "lost_track"):
        return "late"
    return "unfinished"


def _index(row: dict[str, Any]) -> int:
    return int("".join(ch for ch in str(row["part_id"]) if ch.isdigit()))


def match(cell_rows: list[dict[str, Any]], sim_rows: list[dict[str, Any]]) -> list[tuple[dict, dict]]:
    """Pairs by arrival order; the shorter record sets the length."""
    cell = sorted(cell_rows, key=_index)
    sim = sorted(sim_rows, key=_index)
    return list(zip(cell, sim))


def confusion(pairs: list[tuple[dict, dict]]) -> dict[tuple[str, str], int]:
    counts: dict[tuple[str, str], int] = {}
    for cell, sim in pairs:
        key = (category(cell), category(sim))
        counts[key] = counts.get(key, 0) + 1
    return counts


def _cell_belt(run: CellRun, until_s: float) -> float | None:
    """Mean belt speed over the flow.csv buckets of the cell run that end before `until_s`."""
    values = [float(r["mean_belt_mm_s"]) for r in run.flow
              if r.get("mean_belt_mm_s") and float(r["t_end"]) <= until_s]
    return statistics.fmean(values) if values else None


def _cycle(rows: list[dict[str, Any]]) -> tuple[float, float]:
    """(median, 10th percentile) of the time between consecutive picks, s: the arm's pace."""
    times = sorted(float(r["t_contact"]) for r in rows if r.get("outcome") == "picked" and r.get("t_contact"))
    gaps = sorted(b - a for a, b in zip(times, times[1:]))
    if not gaps:
        return float("nan"), float("nan")
    return statistics.median(gaps), gaps[len(gaps) // 10]


def _gaps(rows: list[dict[str, Any]]) -> list[float]:
    times = sorted(float(r["t_contact"]) for r in rows if r.get("outcome") == "picked" and r.get("t_contact"))
    return [b - a for a, b in zip(times, times[1:])]


def ks_statistic(a: list[float], b: list[float]) -> float:
    """Two-sample Kolmogorov-Smirnov distance: 0 for the same distribution, 1 for disjoint ones."""
    if not a or not b:
        return float("nan")
    a, b = sorted(a), sorted(b)
    worst, i, j = 0.0, 0, 0
    while i < len(a) and j < len(b):
        x = min(a[i], b[j])
        while i < len(a) and a[i] <= x:
            i += 1
        while j < len(b) and b[j] <= x:
            j += 1
        worst = max(worst, abs(i / len(a) - j / len(b)))
    return worst


def _rmse(pairs: list[tuple[float, float]]) -> float:
    return (sum((x - y) ** 2 for x, y in pairs) / len(pairs)) ** 0.5 if pairs else float("nan")


def fit_metrics(run: CellRun, sim, sim_rows: list[dict[str, Any]], horizon: float) -> dict[str, float]:
    """How closely the two runs follow each other in time (informational, not gated): the belt
    speed against the cell's 10 s buckets, the 60 s throughput curve, the distribution of the
    time between picks."""
    belt = []
    for r in run.flow:
        lo, hi = float(r["t_start"]), float(r["t_end"])
        if r.get("mean_belt_mm_s") and hi <= horizon:
            mine = [v for t, v in sim.belt_log if lo <= t < hi]
            if mine:
                belt.append((float(r["mean_belt_mm_s"]), statistics.fmean(mine)))
    as_text = lambda rows: [{k: str(v) for k, v in r.items()} for r in rows]  # noqa: E731
    cell_out = {round(t): o for t, _, o in flow_series(as_text(run.parts), step_s=5.0) if t >= 60.0}
    sim_out = {round(t): o for t, _, o in flow_series(as_text(sim_rows), step_s=5.0) if t >= 60.0}
    common = sorted(set(cell_out) & set(sim_out))
    cell_gaps, sim_gaps = _gaps(run.parts), _gaps(sim_rows)
    return {"belt_rmse": _rmse(belt), "throughput_rmse": _rmse([(cell_out[t], sim_out[t]) for t in common]),
            "interval_ks": ks_statistic(cell_gaps, sim_gaps),
            "interval_p90": (sorted(cell_gaps)[int(0.9 * len(cell_gaps))] if cell_gaps else float("nan"),
                             sorted(sim_gaps)[int(0.9 * len(sim_gaps))] if sim_gaps else float("nan"))}


def compare(run: CellRun, sim, gates: dict[str, float]) -> dict[str, Any]:
    """Every number of one comparison; `checks` is [(name, cell, sandbox, relative difference,
    limit, passed)]."""
    sim_rows = sim.records()
    pairs = [(c, s) for c, s in match(run.parts, sim_rows)]
    done = [(c, s) for c, s in pairs if category(s) != "unfinished" and category(c) != "unfinished"]
    cell_count = lambda name, side: sum(1 for p in done if category(p[side]) == name)  # noqa: E731
    horizon = sim.run.sim.duration_s
    belt_cell = _cell_belt(run, horizon)
    belt_sim = statistics.fmean(v for _, v in sim.belt_log) if sim.belt_log else None
    cell_dips = find_dips([{k: str(v) for k, v in r.items()} for r in run.parts])
    sim_dips = find_dips([{k: str(v) for k, v in r.items()} for r in sim_rows])
    agree = sum(1 for c, s in done if category(c) == category(s)) / len(done) if done else float("nan")
    fail_c = {_index(c) for c, _ in done if category(c) != "picked"}
    fail_s = {_index(c) for c, s in done if category(s) != "picked"}
    overlap = len(fail_c & fail_s) / len(fail_c | fail_s) if fail_c | fail_s else 1.0

    def rel(a, b):
        return abs(b - a) / a if a else float("nan")

    picks_c, picks_s = cell_count("picked", 0), cell_count("picked", 1)
    checks = [("picked", picks_c, picks_s, rel(picks_c, picks_s), gates["picks_rel"]),
              ("mean belt (mm/s)", belt_cell, belt_sim,
               rel(belt_cell, belt_sim) if belt_cell and belt_sim else float("nan"), gates["belt_rel"]),
              ("outcomes differing per part", 0.0, 1.0 - agree, 1.0 - agree, 1.0 - gates["agree"])]
    return {
        "parts": len(done), "unfinished": len(pairs) - len(done),
        "counts": {name: (cell_count(name, 0), cell_count(name, 1)) for name in CATEGORIES[:4]},
        "belt": (belt_cell, belt_sim), "cycle": (_cycle(run.parts), _cycle(sim_rows)), "agree": agree, "failure_overlap": overlap,
        "dips": (len(cell_dips), len(sim_dips)), "dip_s": (sum(d.length_s for d in cell_dips),
                                                          sum(d.length_s for d in sim_dips)),
        "confusion": confusion(done), "fit": fit_metrics(run, sim, sim_rows, horizon),
        "checks": [(n, a, b, r, lim, bool(r == r and r <= lim)) for n, a, b, r, lim in checks],
        "cell_rows": run.parts, "sim_rows": sim_rows,
    }


# ---- output --------------------------------------------------------------------------------
def _plot(name: str, run: CellRun, sim, result: dict[str, Any], path: Path) -> None:
    from sandbox.report import _plt

    plt = _plt()
    fig, (top, mid, bot) = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    as_text = lambda rows: [{k: str(v) for k, v in r.items()} for r in rows]  # noqa: E731
    cell = flow_series(as_text(result["cell_rows"]))
    box = flow_series(as_text(result["sim_rows"]))
    top.plot([t for t, _, _ in cell], [i for _, i, _ in cell], color="0.6", lw=1, label="input")
    top.plot([t for t, _, _ in cell], [o for _, _, o in cell], lw=1.4, label="cell output")
    top.plot([t for t, _, _ in box], [o for _, _, o in box], lw=1.4, label="sandbox output")
    top.set_ylabel("parts / min (60 s window)")
    top.legend(ncol=3, fontsize=8)
    cell_belt = [(float(r["t_start"]) + float(r["t_end"])) / 2 for r in run.flow if r.get("mean_belt_mm_s")]
    mid.plot(cell_belt, [float(r["mean_belt_mm_s"]) for r in run.flow if r.get("mean_belt_mm_s")], label="cell (10 s mean)")
    mid.plot([t for t, _ in sim.belt_log], [v for _, v in sim.belt_log], lw=0.8, label="sandbox")
    mid.set_ylabel("belt speed (mm/s)")
    mid.legend(fontsize=8)
    colours = {"covered": "tab:red", "late": "tab:orange", "grip_other": "tab:purple"}
    for row, (side, ys) in enumerate((("cell", 1.0), ("sandbox", 0.0))):
        rows = result["cell_rows"] if side == "cell" else result["sim_rows"]
        for cat, colour in colours.items():
            ts = [float(r["t_seen"]) for r in rows if category(r) == cat]
            bot.scatter(ts, [ys] * len(ts), marker="|", s=120, color=colour, label=cat if row == 0 else None)
    bot.set_yticks([0, 1], ["sandbox", "cell"])
    bot.set_ylim(-0.5, 1.5)
    bot.set_xlabel("arrival time (s)")
    bot.set_ylabel("failed parts")
    bot.legend(ncol=3, fontsize=8, loc="upper right")
    top.set_title(f"{name}: {run.label()}")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _plot_summary(entries: list[dict[str, Any]], path: Path) -> None:
    from sandbox.report import _plt

    plt = _plt()
    fig, (a, b) = plt.subplots(1, 2, figsize=(10, 3.8))
    x = range(len(entries))
    for ax, key, title in ((a, "picked", "picked parts"), (b, "belt", "mean belt speed (mm/s)")):
        cell = [e["result"]["counts"]["picked"][0] if key == "picked" else e["result"]["belt"][0] for e in entries]
        box = [e["result"]["counts"]["picked"][1] if key == "picked" else e["result"]["belt"][1] for e in entries]
        ax.bar([i - 0.2 for i in x], [c or 0 for c in cell], 0.4, label="cell")
        ax.bar([i + 0.2 for i in x], [c or 0 for c in box], 0.4, label="sandbox")
        ax.set_xticks(list(x), [e["name"] for e in entries], rotation=20, ha="right", fontsize=8)
        ax.set_title(title)
        ax.grid(alpha=0.3, axis="y")
    a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _fmt(value: Any) -> str:
    return "-" if value is None or value != value else (f"{value:.3f}" if isinstance(value, float) else str(value))


def write_report(out: Path, spec: dict[str, Any], entries: list[dict[str, Any]], gates: dict[str, float]) -> Path:
    lines = [f"# {spec.get('title') or 'Replay of cell runs'}", "", str(spec.get("description") or ""), "",
             f"Gates: picked within {gates['picks_rel']:.0%}, belt within {gates['belt_rel']:.0%}, "
             f"{gates['agree']:.0%} of the parts with the same outcome. Sandbox input = the cell run's "
             f"`arrivals.csv`; parts are matched by arrival order.", ""]
    verdict = []
    for entry in entries:
        r = entry["result"]
        what_if = "" if entry["gated"] else " (what-if: no cell counterpart under these settings)"
        lines += [f"## {entry['name']}{what_if}", "", f"Cell run `{entry['cell_run']}` — {entry['label']}.", "",
                  "| check | cell | sandbox | difference | limit | |", "|---|---|---|---|---|---|"]
        for n, a, b, rel, limit, ok in r["checks"]:
            lines.append(f"| {n} | {_fmt(a)} | {_fmt(b)} | {_fmt(rel)} | {_fmt(limit)} | "
                         f"{'pass' if ok else 'FAIL'} |")
        verdict.append((entry["name"], all(c[-1] for c in r["checks"]), not entry["gated"]))
        counts = r["counts"]
        lines += ["", "| outcome | cell | sandbox |", "|---|---|---|",
                  *(f"| {name} | {counts[name][0]} | {counts[name][1]} |" for name in counts),
                  f"| pick interval, median / fastest 10% (s) | {r['cycle'][0][0]:.2f} / {r['cycle'][0][1]:.2f} | "
                  f"{r['cycle'][1][0]:.2f} / {r['cycle'][1][1]:.2f} |",
                  f"| pick interval, 90th percentile (s) | {r['fit']['interval_p90'][0]:.2f} | {r['fit']['interval_p90'][1]:.2f} |",
                  f"| dips (count / seconds) | {r['dips'][0]} / {r['dip_s'][0]:.0f} | {r['dips'][1]} / {r['dip_s'][1]:.0f} |",
                  "", f"{r['parts']} parts compared ({r['unfinished']} unfinished on either side); "
                  f"failures shared: {r['failure_overlap']:.0%} of the parts that failed on either side.", "",
                  "Fit over time (not gated): belt-speed RMSE against the cell's 10 s buckets "
                  f"{_fmt(r['fit']['belt_rmse'])} mm/s; 60 s throughput RMSE {_fmt(r['fit']['throughput_rmse'])} parts/min; "
                  f"pick-interval distribution distance (KS) {_fmt(r['fit']['interval_ks'])}.", "",
                  "Rows: cell outcome, columns: sandbox outcome.", "",
                  "| cell \\ sandbox | " + " | ".join(CATEGORIES[:4]) + " |", "|---|" + "---|" * 4]
        for a in CATEGORIES[:4]:
            lines.append(f"| {a} | " + " | ".join(str(r["confusion"].get((a, b), 0)) for b in CATEGORIES[:4]) + " |")
        lines += ["", f"![{entry['name']}](replay_{entry['name']}.png)", ""]
    replayed = [v for v in verdict if not v[2]]
    lines += ["## Verdict", "",
              *(f"- {name}: {'pass' if ok else 'FAIL'}{' (what-if, not gated)' if whatif else ''}"
                for name, ok, whatif in verdict), ""]
    if replayed:
        lines += [f"**{'All replays pass the gates: the sandbox may be used for the next phase.' if all(v[1] for v in replayed) else 'Some replays fail: find out why (open-issues S3) before trusting sandbox sweeps.'}**", ""]
    lines += ["![summary](replay_summary.png)", ""]
    (out / "report.md").write_text("\n".join(lines))
    return out / "report.md"


def run_replay(spec_path: str | Path, *, quick: bool = False) -> Path:
    spec_path = Path(spec_path)
    spec = plain(read_config(spec_path))
    gates = {**DEFAULT_GATES, **(spec.get("gates") or {})}
    out = RESULTS_DIR / f"{spec_path.stem}{'-quick' if quick else ''}-{datetime.now():%Y%m%d-%H%M%S}"
    out.mkdir(parents=True)
    entries = []
    for item in spec["runs"]:
        run = CellRun.load(item["cell_run"])
        overrides = {**run.overrides(), **(item.get("set") or {})}
        if quick:
            overrides["sim.duration_s"] = float((spec.get("quick") or {}).get("duration_s", 900))
        name = str(item["name"])
        print(f"[REPLAY] {name}: {run.label()}  ({len(run.parts)} parts, {overrides['sim.duration_s']:.0f} s)", flush=True)
        sim = simulate(overrides)
        write_run(sim, overrides, out / name)
        result = compare(run, sim, gates)
        _plot(name, run, sim, result, out / f"replay_{name}.png")
        entries.append({"name": name, "cell_run": str(item["cell_run"]), "label": run.label(),
                        "set": item.get("set") or {}, "result": result,
                        "gated": bool(item.get("gated", not item.get("set")))})
        print("  " + "; ".join(f"{n}: {_fmt(a)} vs {_fmt(b)} {'ok' if ok else 'FAIL'}"
                               for n, a, b, _, _, ok in result["checks"]), flush=True)
    _plot_summary(entries, out / "replay_summary.png")
    report = write_report(out, spec, entries, gates)
    (out / "manifest.json").write_text(json.dumps({"spec": spec_path.name, "quick": quick, "git": _git_revision(),
                                                   "gates": gates, "spec_full": spec}, indent=2, default=str))
    with open(out / "replay.csv", "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("run", "check", "cell", "sandbox", "difference", "limit", "pass"))
        for e in entries:
            writer.writerows((e["name"], n, a, b, r, lim, ok) for n, a, b, r, lim, ok in e["result"]["checks"])
            fit = e["result"]["fit"]
            writer.writerows((e["name"], f"fit: {n}", "", v, "", "", "")
                             for n, v in (("belt RMSE (mm/s)", fit["belt_rmse"]),
                                          ("throughput RMSE (parts/min)", fit["throughput_rmse"]),
                                          ("pick-interval KS", fit["interval_ks"])))
    print(f"[REPLAY] {report}")
    return out
