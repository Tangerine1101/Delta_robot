"""System identification of the cell from a console session: `python3 -m modules.tools.identify_plant log/<session>`.

A console session logs, next to its run folders, the arm pose stream (`pose.csv`), every PLC
command (`commands.jsonl`) and the belt status (`status.csv`). From those this tool measures
the plant terms the scheduler and the sandbox assume, run by run (runs with fewer than
`--min-picked` picks, e.g. aborted starts, are skipped):

* **arm legs** — for every trajectory (command 3), send -> arrival at its last waypoint in the
  pose stream, against `core/motion.trajectory_time`; per leg kind (goto = ends above the part,
  pick = ends over the bin) the median gap and the fit by path length (open-issues C5, G8);
* **pick cycle** — goto send -> next goto send split into goto flight, wait for the part, pick
  flight and the gap to the next goto (open-issues C7, S3);
* **descent** — pick send -> lowest z, and how deep (T3);
* **belt servo** — each command-8 step against `speed_current` in `status.csv`: delay and 10-90 %
  time from the send, and the settled speed against the setpoint (C8 concerns what the
  feedback is *of*, not how it follows the command);
* **pick error** — `error_u_mm` of the picked parts against the belt speed at contact.

It writes `identification.md`, `legs.csv` and `identification.png` into `--out` (default
`sandbox/results/identify_<session>`). The sandbox plant constants it supports are
`sim.goto_extra_s` and `sim.pick_extra_s` (`sandbox/config.yaml`): the median gaps below.

    python3 -m modules.tools.identify_plant log/20261010-163210_exp_PR_DP_1010_console
    python3 -m modules.tools.identify_plant log/<session> --min-picked 300 --out report/
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path
from typing import Any

import numpy as np

from modules.config_io import read_config
from modules.core.motion import trajectory_time
from modules.runlog import REPO_ROOT
from modules.settings import settings_from_dict
from modules.tools.log_timeline import BLUE, GRID, INK, MUTED, load_matplotlib

# A leg whose end z is this close to the pre-pick height ends above the part; this close to the
# place height, over the bin (mm).
_Z_TOL_MM = 1.5
# The arm has left its start / reached its end within these (mm).
_MOVE_MM = 0.5
_ARRIVE_MM = 1.0
# A pick-to-pick gap shorter than this means the arm did not wait for its part (s).
_BACK_TO_BACK_S = 0.35
# A belt step needs this much quiet before and after to be measured (s), and this size (mm/s).
_STEP_QUIET_S = 1.2
_STEP_MIN_MM_S = 4.0

_ENDED = re.compile(r"scenario '[^']*' ended .*record in (\S+)")


def run_windows(session: Path) -> dict[str, tuple[float, float]]:
    """{run folder name: (start, end) in the logs' monotonic time}, from the console log's
    `scenario ... ended` lines (end) and each run's `duration_s` (start)."""
    ends: dict[str, float] = {}
    with open(session / "console.log", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if "' ended" not in line:
                continue
            found = _ENDED.search(line)
            if found:
                ends[Path(found.group(1)).name] = float(line.split(" ", 2)[1])
    windows = {}
    for name, end in ends.items():
        summary_path = session / name / "summary.json"
        if summary_path.is_file():
            windows[name] = (end - float(json.loads(summary_path.read_text())["duration_s"]), end)
    return windows


def _label(summary: dict[str, Any], name: str) -> str:
    return summary.get("run_name") or name


def legs(session: Path, runs: dict[str, tuple[str, float, float]], interp) -> list[dict[str, Any]]:
    """One row per trajectory: when it was sent, when the arm moved, arrived and reached its
    lowest z, and what the trajectory model says."""
    pose = np.loadtxt(session / "pose.csv", delimiter=",", skiprows=1, usecols=(0, 2, 3, 4))
    times, xyz = pose[:, 0], pose[:, 1:]
    commands = [c for c in (json.loads(line) for line in open(session / "commands.jsonl", encoding="utf-8"))
                if c["commandID"] == 3 and c.get("ok")]
    rows: list[dict[str, Any]] = []
    for index, command in enumerate(commands):
        sent = command["t_mono_send"]
        label = next((name for name, (_, start, end) in runs.items() if start - 1.0 < sent < end + 1.0), None)
        if label is None:
            continue
        package = command["package"]
        count = package["argument_number"]
        points = list(zip(package["argument_x"][:count], package["argument_y"][:count], package["argument_z"][:count]))
        last_z = points[-1][2]
        if abs(last_z - (-287.0)) < _Z_TOL_MM:
            kind = "goto"
        elif abs(last_z - (-280.0)) < _Z_TOL_MM:
            kind = "pick"
        else:
            continue
        until = min(commands[index + 1]["t_mono_send"] if index + 1 < len(commands) else sent + 8.0, sent + 6.0)
        lo, hi = np.searchsorted(times, sent), np.searchsorted(times, until)
        if hi - lo < 10:
            continue
        path, stamp = xyz[lo:hi], times[lo:hi]
        left = np.where(np.linalg.norm(path - path[0], axis=1) > _MOVE_MM)[0]
        target = np.array(points[-1])
        near = np.where(np.linalg.norm(path - target, axis=1) < _ARRIVE_MM)[0]
        if len(left):
            near = near[near > left[0]]
        arrived = stamp[near[0]] if len(near) else float("nan")
        bottom = stamp[np.where(path[:, 2] <= path[:, 2].min() + 0.5)[0][0]]
        length = sum(float(np.linalg.norm(np.subtract(points[i + 1], points[i]))) for i in range(len(points) - 1))
        rows.append({"run": label, "kind": kind, "sent": sent, "moved": stamp[left[0]] if len(left) else float("nan"),
                     "arrived": arrived, "bottom": bottom, "z_min": float(path[:, 2].min()),
                     "model_s": trajectory_time(points, interp), "length_mm": length,
                     "rtt_s": command["rtt_s"]})
    return rows


def cycles(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """goto -> pick -> goto triples of one run: flight, wait, flight and the gap to the next goto."""
    out = []
    for a, b, c in zip(rows, rows[1:], rows[2:]):
        if (a["kind"], b["kind"], c["kind"]) == ("goto", "pick", "goto") and a["run"] == c["run"]:
            out.append({"run": a["run"], "goto_s": a["arrived"] - a["sent"], "wait_s": b["sent"] - a["arrived"],
                        "pick_s": b["arrived"] - b["sent"], "setup_s": c["sent"] - b["arrived"],
                        "cycle_s": c["sent"] - a["sent"]})
    return out


def belt_steps(session: Path) -> tuple[list[dict[str, float]], list[tuple[float, float]]]:
    """Command-8 steps against the status stream: ([per-step delay and 10/50/90 % times from the
    send], [(setpoint, settled speed)])."""
    status = np.genfromtxt(session / "status.csv", delimiter=",", skip_header=1, usecols=(0, 10), filling_values=np.nan)
    times, speed = status[:, 0], status[:, 1]
    commands = [c for c in (json.loads(line) for line in open(session / "commands.jsonl", encoding="utf-8"))
                if c["commandID"] == 8 and c.get("ok")]
    sent = np.array([c["t_mono_send"] for c in commands])
    setpoint = np.array([c["package"]["speed"] for c in commands])
    steps, settled = [], []
    for i in range(1, len(sent) - 1):
        before = (times > sent[i] - 0.6) & (times < sent[i])
        after = (times > sent[i] + 0.8) & (times < sent[i + 1] - 0.05)
        if after.sum() > 5 and sent[i + 1] - sent[i] > 3.0:
            late = (times > sent[i] + 1.5) & (times < sent[i + 1] - 0.1)
            if late.sum() > 5:
                settled.append((float(setpoint[i]), float(np.median(speed[late]))))
        step = setpoint[i] - setpoint[i - 1]
        if abs(step) < _STEP_MIN_MM_S or sent[i] - sent[i - 1] < _STEP_QUIET_S or sent[i + 1] - sent[i] < _STEP_QUIET_S:
            continue
        if before.sum() < 3 or after.sum() < 3:
            continue
        v0, v1 = float(np.median(speed[before])), float(np.median(speed[after]))
        window = (times >= sent[i] - 0.1) & (times <= sent[i] + 1.0)
        tw, vw = times[window], speed[window]

        def crossing(fraction: float) -> float:
            level = v0 + fraction * (v1 - v0)
            hit = np.where((vw - level) * np.sign(step) >= 0)[0]
            if not len(hit) or hit[0] == 0:
                return float("nan")
            k = hit[0]
            return float(tw[k - 1] + (level - vw[k - 1]) / (vw[k] - vw[k - 1]) * (tw[k] - tw[k - 1]) - sent[i])

        steps.append({"step": float(abs(step)), "t10": crossing(0.1), "t50": crossing(0.5), "t90": crossing(0.9)})
    return steps, settled


def pick_errors(session: Path, names: list[str]) -> np.ndarray:
    """(error_u_mm, error_v_mm, belt mm/s at contact) of every picked part of the runs."""
    out = []
    for name in names:
        with open(session / name / "parts.csv", newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                if row["outcome"] == "picked" and row["error_u_mm"] and row["belt_mm_s_at_contact"]:
                    out.append((float(row["error_u_mm"]), float(row["error_v_mm"]), float(row["belt_mm_s_at_contact"])))
    return np.array(out).reshape(-1, 3)


def _stats(values: np.ndarray) -> str:
    values = values[np.isfinite(values)]
    if not len(values):
        return "| – | – | – | 0 |"
    p5, p50, p95 = np.percentile(values, [5, 50, 95])
    return f"| {p5:.3f} | {p50:.3f} | {p95:.3f} | {len(values)} |"


def build_report(session: Path, min_picked: int) -> tuple[str, list[dict[str, Any]], dict[str, Any]]:
    settings = settings_from_dict(read_config(session / "config.yaml"))
    windows = run_windows(session)
    runs: dict[str, tuple[str, float, float]] = {}
    skipped = []
    for name, (start, end) in sorted(windows.items(), key=lambda item: item[1]):
        summary = json.loads((session / name / "summary.json").read_text())
        if summary.get("picked", 0) < min_picked:
            skipped.append(f"{name} ({summary.get('picked', 0)} picked)")
            continue
        runs[_label(summary, name)] = (name, start, end)
    if not runs:
        raise SystemExit(f"{session}: no run with at least {min_picked} picks")
    rows = legs(session, runs, settings.robot.interpolator)
    rows_by_run = {label: [r for r in rows if r["run"] == label] for label in runs}
    cycle_rows = [c for label in runs for c in cycles(rows_by_run[label])]
    steps, settled = belt_steps(session)
    errors = pick_errors(session, [name for name, _, _ in runs.values()])

    arr = lambda key, kind=None, src=rows: np.array([r[key] for r in src if kind in (None, r["kind"])], dtype=float)
    gap = lambda kind: arr("arrived", kind) - arr("sent", kind) - arr("model_s", kind)
    lines = [f"# Plant identification — {session.name}", "",
             f"Runs used ({len(runs)}, at least {min_picked} picks): " + ", ".join(runs) + ".",
             "Skipped: " + (", ".join(skipped) if skipped else "none") + ".", "",
             "Interpolator of the session config: " + ", ".join(
                 f"{k} = {v}" for k, v in vars(settings.robot.interpolator).items()) + ".", "",
             "## Arm legs: send -> arrival against the trajectory model", "",
             "Gap = measured send -> arrival minus `trajectory_time` (s). Sandbox constants: `sim.goto_extra_s`, `sim.pick_extra_s`.", "",
             "| leg | gap p5 | gap median | gap p95 | n |", "|---|---|---|---|---|"]
    for kind in ("goto", "pick"):
        lines.append(f"| {kind} " + _stats(gap(kind)))
    lines += ["", "Gap by path length (median, s):", "", "| path length (mm) | goto n | goto gap | pick n | pick gap |", "|---|---|---|---|---|"]
    for lo, hi in ((0, 120), (120, 160), (160, 200), (200, 240), (240, 280), (280, 400)):
        cells = []
        for kind in ("goto", "pick"):
            length = arr("length_mm", kind)
            mask = (length >= lo) & (length < hi) & np.isfinite(gap(kind))
            cells += [str(int(mask.sum())), f"{np.median(gap(kind)[mask]):+.3f}" if mask.sum() > 5 else "–"]
        lines.append(f"| {lo}–{hi} | " + " | ".join(cells) + " |")
    rtt = arr("rtt_s")
    moved_lag = arr("moved") - arr("sent")
    lines += ["", f"Command write (rtt) median {np.nanmedian(rtt):.3f} s; send -> first motion median {np.nanmedian(moved_lag):.3f} s.", "",
              "## Descent (pick leg)", "",
              "| quantity | p5 | median | p95 | n |", "|---|---|---|---|---|",
              "| send -> lowest z (s) " + _stats(arr("bottom", "pick") - arr("sent", "pick")),
              "| lowest z (mm) " + _stats(arr("z_min", "pick")), "",
              "## Pick cycle", "",
              "| part of the cycle (s) | p5 | median | p95 | n |", "|---|---|---|---|---|"]
    for key, label in (("goto_s", "goto flight"), ("wait_s", "wait for the part"), ("pick_s", "pick flight"),
                       ("setup_s", "pick arrival -> next goto"), ("cycle_s", "goto send -> next goto send")):
        lines.append(f"| {label} " + _stats(np.array([c[key] for c in cycle_rows])))
    lines += ["", "Back-to-back cycles (wait below %.2f s) per run:" % _BACK_TO_BACK_S, "",
              "| run | cycles | back-to-back | cycle median (s) | back-to-back cycle median (s) |", "|---|---|---|---|---|"]
    for label in runs:
        mine = [c for c in cycle_rows if c["run"] == label]
        tight = [c for c in mine if c["wait_s"] < _BACK_TO_BACK_S]
        lines.append(f"| {label} | {len(mine)} | {len(tight)} | {np.nanmedian([c['cycle_s'] for c in mine]):.3f} | "
                     f"{np.nanmedian([c['cycle_s'] for c in tight]) if tight else float('nan'):.3f} |")
    lines += ["", "## Belt servo (command 8 against `speed_current`)", "", f"{len(steps)} steps of at least {_STEP_MIN_MM_S:.0f} mm/s, "
              "times from the command send (s):", "", "| point of the step | p5 | median | p95 | n |", "|---|---|---|---|---|"]
    for key, label in (("t10", "10 %"), ("t50", "50 %"), ("t90", "90 %")):
        lines.append(f"| {label} " + _stats(np.array([s[key] for s in steps])))
    lines += ["", "| setpoint band (mm/s) | settled steps | median speed − setpoint | median speed / setpoint |", "|---|---|---|---|"]
    settled_arr = np.array(settled).reshape(-1, 2)
    for lo, hi in ((0, 15), (15, 25), (25, 40), (40, 60), (60, 120)):
        mask = (settled_arr[:, 0] >= max(lo, 1.0)) & (settled_arr[:, 0] < hi)
        if mask.sum():
            diff = settled_arr[mask, 1] - settled_arr[mask, 0]
            lines.append(f"| {lo}–{hi} | {int(mask.sum())} | {np.median(diff):+.3f} | {np.median(settled_arr[mask, 1] / settled_arr[mask, 0]):.4f} |")
    lines += ["", "## Pick error along the belt (picked parts)", ""]
    if len(errors):
        slope, offset = np.linalg.lstsq(np.c_[errors[:, 2], np.ones(len(errors))], errors[:, 0], rcond=None)[0]
        lines += [f"{len(errors)} picks. `error_u_mm` = {slope:.4f} · belt speed + {offset:.3f} mm (the slope is a time: "
                  f"{slope * 1000:.0f} ms late at contact).", "",
                  "| belt speed (mm/s) | picks | median error_u (mm) | sd (mm) |", "|---|---|---|---|"]
        for lo, hi in ((0, 15), (15, 25), (25, 40), (40, 60), (60, 120)):
            mask = (errors[:, 2] >= lo) & (errors[:, 2] < hi)
            if mask.sum() > 10:
                lines.append(f"| {lo}–{hi} | {int(mask.sum())} | {np.median(errors[mask, 0]):+.2f} | {errors[mask, 0].std():.2f} |")
    return "\n".join(lines) + "\n", rows, {"steps": steps, "settled": settled, "errors": errors, "cycles": cycle_rows}


def draw(rows: list[dict[str, Any]], extra: dict[str, Any], path: Path) -> None:
    plt = load_matplotlib()
    if plt is None:
        return
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for kind, colour in (("goto", BLUE), ("pick", "#eb6834")):
        pick_rows = [r for r in rows if r["kind"] == kind and np.isfinite(r["arrived"])]
        axes[0][0].scatter([r["model_s"] for r in pick_rows], [r["arrived"] - r["sent"] for r in pick_rows], s=4, alpha=0.3,
                           color=colour, label=kind)
    lim = axes[0][0].get_xlim()
    axes[0][0].plot(lim, lim, color=MUTED, lw=1)
    axes[0][0].set(xlabel="trajectory_time (s)", ylabel="measured send -> arrival (s)", title="Arm legs")
    axes[0][0].legend()
    parts = ("goto_s", "wait_s", "pick_s", "setup_s")
    medians = [np.nanmedian([c[k] for c in extra["cycles"]]) for k in parts]
    left = 0.0
    for key, value in zip(parts, medians):
        axes[0][1].barh(0, value, left=left, label=f"{key[:-2]} {value:.2f}")
        left += value
    axes[0][1].set(yticks=[], xlabel="s", title="Median pick cycle")
    axes[0][1].legend(fontsize=8)
    steps = extra["steps"]
    axes[1][0].hist([s["t50"] for s in steps if np.isfinite(s["t50"])], bins=30, color=BLUE)
    axes[1][0].set(xlabel="50 % of a belt step, from the command send (s)", ylabel="steps", title="Belt servo")
    errors = extra["errors"]
    if len(errors):
        axes[1][1].scatter(errors[:, 2], errors[:, 0], s=3, alpha=0.2, color=BLUE)
    axes[1][1].set(xlabel="belt speed at contact (mm/s)", ylabel="error_u (mm)", title="Pick error along the belt")
    for axis in axes.flat:
        axis.grid(color=GRID, lw=0.6)
        axis.title.set_color(INK)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", type=Path, help="a console session folder under log/")
    parser.add_argument("--min-picked", type=int, default=100, help="skip runs with fewer picks (default 100)")
    parser.add_argument("--out", type=Path, default=None, help="output folder (default sandbox/results/identify_<session>)")
    args = parser.parse_args(argv)
    session = args.session if args.session.is_absolute() else Path.cwd() / args.session
    if not session.is_dir():
        session = REPO_ROOT / args.session
    for name in ("pose.csv", "commands.jsonl", "status.csv", "console.log", "config.yaml"):
        if not (session / name).is_file():
            raise SystemExit(f"{session}: not a console session (missing {name})")
    out = args.out or REPO_ROOT / "sandbox" / "results" / f"identify_{session.name}"
    out.mkdir(parents=True, exist_ok=True)
    text, rows, extra = build_report(session, args.min_picked)
    (out / "identification.md").write_text(text, encoding="utf-8")
    with open(out / "legs.csv", "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    draw(rows, extra, out / "identification.png")
    print(text)
    print(f"[OK] wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
