"""Figures of the plant-identification report (doc/plant-identification.md).

Reads a console session (the measurements), the `replay.csv` / `report.md` of the replay
experiments (the validation) and the `summary.csv` of a W map, and writes up to seven PNG files (the seventh with `--noise`):

    .venv/bin/python -m sandbox.identification_figures log/<session> \\
        --replay sandbox/results/phase0_replay-<time> sandbox/results/phase0_replay_1001-<time> \\
        --w-map sandbox/results/w_map-<time> --noise sandbox/results/phase0_noise-<time> \\
        --out doc/figures/identification
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import numpy as np

from modules.config_io import read_config
from modules.settings import settings_from_dict
from modules.tools import identify_plant as ident
from modules.tools.log_timeline import BLUE, GRID, INK, MUTED, ORANGE, AQUA, load_matplotlib

# Cell runs of the 2026-10-10 session as (label, W) for the W-map overlay.
CELL_W_POINTS = {15.0: 943, 25.0: 958, 50.0: 846}
CELL_INPUT = 1000


def _style(plt):
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.titleweight": "bold", "axes.titlesize": 10})


def _grid(axis):
    axis.grid(color=GRID, lw=0.6)
    axis.set_axisbelow(True)
    for side in ("top", "right"):
        axis.spines[side].set_visible(False)


def session_data(session: Path, min_picked: int = 100):
    settings = settings_from_dict(read_config(session / "config.yaml"))
    windows = ident.run_windows(session)
    runs = {}
    for name, (start, end) in sorted(windows.items(), key=lambda item: item[1]):
        summary = json.loads((session / name / "summary.json").read_text())
        if summary.get("picked", 0) >= min_picked:
            runs[summary.get("run_name") or name] = (name, start, end)
    rows = ident.legs(session, runs, settings.robot.interpolator)
    cycles = [c for label in runs for c in ident.cycles([r for r in rows if r["run"] == label])]
    return runs, rows, cycles


def fig_legs(rows, out: Path, plt) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    colour = {"goto": BLUE, "pick": ORANGE}
    for kind in ("goto", "pick"):
        mine = [r for r in rows if r["kind"] == kind and np.isfinite(r["arrived"])]
        model = np.array([r["model_s"] for r in mine])
        measured = np.array([r["arrived"] - r["sent"] for r in mine])
        length = np.array([r["length_mm"] for r in mine])
        axes[0].scatter(model, measured, s=4, alpha=0.25, color=colour[kind], label=f"{kind} (n = {len(mine)})")
        axes[1].scatter(length, measured - model, s=4, alpha=0.2, color=colour[kind])
        edges = np.arange(100, 420, 20)
        mid, med = [], []
        for lo, hi in zip(edges[:-1], edges[1:]):
            mask = (length >= lo) & (length < hi)
            if mask.sum() >= 15:
                mid.append((lo + hi) / 2)
                med.append(np.median((measured - model)[mask]))
        axes[1].plot(mid, med, color=colour[kind], lw=2)
    lim = [0.3, 1.25]
    axes[0].plot(lim, lim, color=INK, lw=1)
    axes[0].set(xlim=lim, ylim=lim, xlabel="modelled trajectory time (s)", ylabel="measured send → arrival (s)",
                title="(a) Each leg against the model")
    axes[0].legend(loc="upper left", frameon=False)
    axes[1].axhline(0, color=INK, lw=1)
    axes[1].set(xlabel="path length (mm)", ylabel="measured − modelled (s)", ylim=(-0.08, 0.3),
                title="(b) The gap by path length (lines: median per 20 mm)")
    for axis in axes:
        _grid(axis)
    fig.tight_layout()
    fig.savefig(out / "fig1_legs.png", dpi=150)
    plt.close(fig)


def fig_cycle(cycles, replay_dirs, out: Path, plt) -> None:
    tight = [c for c in cycles if c["wait_s"] < ident._BACK_TO_BACK_S]
    parts = [("goto_s", "goto flight", BLUE), ("wait_s", "wait for the part", AQUA),
             ("pick_s", "pick flight", ORANGE), ("setup_s", "gap to next goto", MUTED)]
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6), gridspec_kw={"width_ratios": [1.5, 1]})
    left = 0.0
    for key, label, colour in parts:
        value = float(np.nanmedian([c[key] for c in tight]))
        axes[0].barh(0, value, left=left, color=colour, label=f"{label} {value:.2f} s")
        left += value
    axes[0].text(left + 0.03, 0, f"{left:.2f} s", va="center", color=INK)
    axes[0].set(yticks=[], xlim=(0, 2.9), xlabel="s",
                title=f"(a) Cell, back-to-back cycle (median, n = {len(tight)})")
    axes[0].legend(loc="lower center", bbox_to_anchor=(0.5, -0.65), ncol=2, frameon=False)
    intervals = {"cell": [], "uncalibrated": [], "identified": []}
    for directory in replay_dirs:
        text = (directory / "report.md").read_text()
        for name, body in re.findall(r"^## (\S+)[^\n]*\n(.*?)(?=^## |\Z)", text, flags=re.M | re.S):
            found = re.search(r"pick interval, median / fastest 10% \(s\) \| ([\d.]+) / [\d.]+ \| ([\d.]+) / [\d.]+", body)
            if not found:
                continue
            if name.endswith("_uncal") or name.endswith("_asis"):
                intervals["cell"].append(float(found.group(1)))
                intervals["uncalibrated"].append(float(found.group(2)))
            elif name.endswith("_cal"):
                intervals["identified"].append(float(found.group(2)))
    values = [np.mean(intervals[k]) for k in intervals]
    axes[1].bar(list(intervals), values, color=[BLUE, MUTED, AQUA])
    for i, value in enumerate(values):
        axes[1].text(i, value + 0.02, f"{value:.2f}", ha="center", color=INK)
    axes[1].set(ylim=(1.8, 2.7), ylabel="median pick interval (s)", title="(b) Pick interval, mean over the replays")
    for axis in axes:
        _grid(axis)
    fig.tight_layout()
    fig.savefig(out / "fig2_cycle.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def fig_belt(session: Path, out: Path, plt) -> None:
    status = np.genfromtxt(session / "status.csv", delimiter=",", skip_header=1, usecols=(0, 10), filling_values=np.nan)
    times, speed = status[:, 0], status[:, 1]
    commands = [c for c in (json.loads(line) for line in open(session / "commands.jsonl", encoding="utf-8"))
                if c["commandID"] == 8 and c.get("ok")]
    sent = np.array([c["t_mono_send"] for c in commands])
    setpoint = np.array([c["package"]["speed"] for c in commands])
    grid = np.arange(-0.2, 0.5, 0.005)
    curves = []
    for i in range(1, len(sent) - 1):
        step = setpoint[i] - setpoint[i - 1]
        if abs(step) < 4 or sent[i] - sent[i - 1] < 1.2 or sent[i + 1] - sent[i] < 1.2:
            continue
        before = (times > sent[i] - 0.6) & (times < sent[i])
        after = (times > sent[i] + 0.8) & (times < sent[i + 1] - 0.05)
        if before.sum() < 3 or after.sum() < 3:
            continue
        v0, v1 = np.median(speed[before]), np.median(speed[after])
        window = (times >= sent[i] - 0.3) & (times <= sent[i] + 0.6)
        curves.append((abs(step), np.interp(grid, times[window] - sent[i], (speed[window] - v0) / (v1 - v0))))
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8))
    for lo, hi, colour, label in ((4, 10, MUTED, "4–10 mm/s"), (10, 16, BLUE, "10–16 mm/s"), (16, 25, ORANGE, "16–25 mm/s")):
        mine = np.array([c for s, c in curves if lo <= s < hi])
        if len(mine):
            axes[0].plot(grid, np.mean(mine, axis=0), color=colour, lw=2, label=f"{label} (n = {len(mine)})")
    axes[0].axhline(0.5, color=GRID, lw=1)
    axes[0].set(xlabel="time from the command-8 send (s)", ylabel="fraction of the step",
                title="(a) Mean step response by step size", xlim=(-0.1, 0.4))
    axes[0].legend(frameon=False, loc="lower right")
    steps = ident.belt_steps(session)[0]
    t50 = [s["t50"] for s in steps if np.isfinite(s["t50"])]
    axes[1].hist(t50, bins=36, color=BLUE)
    axes[1].axvline(np.median(t50), color=ORANGE, lw=2)
    axes[1].text(np.median(t50) + 0.002, axes[1].get_ylim()[1] * 0.9, f"median {np.median(t50):.3f} s", color=ORANGE)
    axes[1].set(xlabel="time to 50 % of the step (s)", ylabel="steps", title=f"(b) Time to 50 % ({len(t50)} steps)")
    for axis in axes:
        _grid(axis)
    fig.tight_layout()
    fig.savefig(out / "fig3_belt.png", dpi=150)
    plt.close(fig)


def fig_error(session: Path, runs, out: Path, plt) -> None:
    errors = ident.pick_errors(session, [name for name, _, _ in runs.values()])
    fig, axis = plt.subplots(figsize=(6.4, 3.8))
    jitter = np.random.default_rng(1).normal(0, 0.4, len(errors))
    axis.scatter(errors[:, 2] + jitter, errors[:, 0], s=3, alpha=0.2, color=BLUE)
    slope, offset = np.linalg.lstsq(np.c_[errors[:, 2], np.ones(len(errors))], errors[:, 0], rcond=None)[0]
    xs = np.array([0, 60])
    axis.plot(xs, slope * xs + offset, color=ORANGE, lw=2, label=f"{slope:.4f} s × v + {offset:.2f} mm")
    axis.set(xlabel="belt speed at contact (mm/s)", ylabel="error along the belt (mm)", ylim=(-1, 6),
             title=f"Pick error of virtual parts (n = {len(errors)})")
    axis.legend(frameon=False)
    _grid(axis)
    fig.tight_layout()
    fig.savefig(out / "fig4_pick_error.png", dpi=150)
    plt.close(fig)


def _replay_rows(directories):
    table: dict[str, dict[str, tuple[float, float]]] = {}
    for directory in directories:
        with open(directory / "replay.csv", newline="") as handle:
            for row in csv.DictReader(handle):
                table.setdefault(row["run"], {})[row["check"]] = (float(row["cell"]), float(row["sandbox"]))
    return table


def fig_replay(directories, out: Path, plt) -> None:
    table = _replay_rows(directories)
    labels = [("dp_w15", "110 / W15 dp"), ("kim_w15", "110 / W15 KIM"), ("kim_w45", "110 / W45 KIM"),
              ("w15", "1001 / W15"), ("w25", "1001 / W25"), ("w50", "1001 / W50"), ("c20", "1001 / const 20")]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    x = np.arange(len(labels))
    for axis, check, title, ylabel in ((axes[0], "picked", "(a) Parts picked", "parts picked"),
                                       (axes[1], "mean belt (mm/s)", "(b) Mean belt speed", "mm/s")):
        cell = [table[f"{key}_{'cal' if f'{key}_cal' in table else 'x080'}"][check][0] for key, _ in labels]
        uncal = [table[f"{key}_{'uncal' if f'{key}_uncal' in table else 'asis'}"][check][1] for key, _ in labels]
        cal = [table[f"{key}_{'cal' if f'{key}_cal' in table else 'x080'}"][check][1] for key, _ in labels]
        axis.bar(x - 0.27, cell, 0.27, color=BLUE, label="cell")
        axis.bar(x, uncal, 0.27, color=MUTED, label="sandbox, command delay on each leg")
        axis.bar(x + 0.27, cal, 0.27, color=AQUA, label="sandbox, identified legs")
        axis.set_xticks(x, [name for _, name in labels], rotation=25, ha="right")
        axis.set(ylabel=ylabel, title=title)
        axis.set_ylim(0, max(cell + cal + uncal) * 1.12)
        for i, (c, u, k) in enumerate(zip(cell, uncal, cal)):
            axis.text(i, max(c, u, k) * 1.02, f"{100 * (k - c) / c:+.1f} %", ha="center", fontsize=7.5, color=AQUA)
        _grid(axis)
    handles, names = axes[0].get_legend_handles_labels()
    fig.legend(handles, names, loc="upper center", ncol=3, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(out / "fig5_replay.png", dpi=150)
    plt.close(fig)


def fig_wmap(directory: Path, out: Path, plt) -> None:
    rows = list(csv.DictReader(open(directory / "summary.csv", newline="")))
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8), sharey=True)
    for axis, rate in zip(axes, (20, 24, 28)):
        for aware, colour, label in (("True", BLUE, "stack-aware"), ("False", MUTED, "stack-blind")):
            mine = sorted((float(r["horizon_s"]), float(r["pick_share_mean"]), float(r["pick_share_sd"]))
                          for r in rows if float(r["rate_per_min"]) == rate and r["stack_aware"] == aware)
            ws = [min(w, 150.0) for w, _, _ in mine]
            axis.errorbar(ws, [m for _, m, _ in mine], [s for _, _, s in mine], color=colour, marker="o", ms=3,
                          lw=1.8, capsize=2, label=label)
        if rate == 24:
            axis.scatter(list(CELL_W_POINTS), [v / CELL_INPUT for v in CELL_W_POINTS.values()], marker="D", s=45,
                         color=ORANGE, zorder=5, label="cell, seed 1001 (23.4 / min)")
        axis.set(xscale="log", xlabel="W (s)", title=f"{rate} parts / min")
        axis.set_xticks([5, 10, 20, 45, 90, 150], ["5", "10", "20", "45", "90", "∞"])
        axis.minorticks_off()
        _grid(axis)
    axes[0].set_ylabel("share of arrived parts picked")
    axes[1].legend(frameon=False, loc="lower left", fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "fig6_w_map.png", dpi=150)
    plt.close(fig)


def fig_noise(directory: Path, out: Path, plt) -> None:
    """The cell's value against the spread of clock-shifted sandbox replays of the same run."""
    labels = [("w15", "1001 / W15"), ("w25", "1001 / W25"), ("w50", "1001 / W50"), ("c20", "1001 / const 20"),
              ("k45", "110 / W45 KIM")]
    values: dict[tuple[str, str], list[float]] = {}
    cell: dict[tuple[str, str], float] = {}
    with open(directory / "replay.csv", newline="") as handle:
        for row in csv.DictReader(handle):
            key = (row["run"].split("_")[0], row["check"])
            if row["check"] in ("picked", "mean belt (mm/s)"):
                values.setdefault(key, []).append(float(row["sandbox"]))
                cell[key] = float(row["cell"])
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8))
    for axis, check, title in ((axes[0], "picked", "(a) Parts picked"), (axes[1], "mean belt (mm/s)", "(b) Mean belt speed")):
        for i, (key, _) in enumerate(labels):
            mine = [100 * (v - cell[(key, check)]) / cell[(key, check)] for v in values.get((key, check), [])]
            if not mine:
                continue
            axis.plot([i, i], [min(mine), max(mine)], color=AQUA, lw=7, alpha=0.35, solid_capstyle="butt")
            axis.scatter([i] * len(mine), mine, s=14, color=AQUA, zorder=3,
                         label="sandbox, 7 clock-shifted replays" if i == 0 else None)
        axis.axhline(0, color=BLUE, lw=2, label="cell")
        limit = 5 if check == "picked" else 10
        axis.axhspan(-limit, limit, color=GRID, alpha=0.5, zorder=0, label=f"gate ±{limit} %")
        axis.set_xticks(range(len(labels)), [name for _, name in labels], rotation=20, ha="right")
        axis.set(ylabel="sandbox − cell (% of the cell)", title=title)
        _grid(axis)
    axes[1].legend(frameon=False, fontsize=8, loc="lower left")
    fig.tight_layout()
    fig.savefig(out / "fig7_noise_floor.png", dpi=150)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", type=Path)
    parser.add_argument("--replay", type=Path, nargs="+", required=True, help="replay result folders")
    parser.add_argument("--w-map", type=Path, required=True)
    parser.add_argument("--noise", type=Path, default=None, help="phase0_noise result folder (adds fig7)")
    parser.add_argument("--out", type=Path, default=Path("doc/figures/identification"))
    args = parser.parse_args(argv)
    plt = load_matplotlib()
    if plt is None:
        raise SystemExit("matplotlib is required")
    _style(plt)
    args.out.mkdir(parents=True, exist_ok=True)
    runs, rows, cycles = session_data(args.session)
    fig_legs(rows, args.out, plt)
    fig_cycle(cycles, args.replay, args.out, plt)
    fig_belt(args.session, args.out, plt)
    fig_error(args.session, runs, args.out, plt)
    fig_replay(args.replay, args.out, plt)
    fig_wmap(args.w_map, args.out, plt)
    if args.noise:
        fig_noise(args.noise, args.out, plt)
    print(f"[OK] wrote the figures to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
