"""Pick timing and belt scale from a run log: the two measurements the pick gate rests on.

1. **Gate -> contact latency** (open-issues C2, T3). For every pick-phase command 3 in
   `commands.jsonl`, the UDP pose stream (`pose.csv`, ~12 ms) gives when the arm starts to
   move and when it reaches its lowest z. Their median is what `pick_gate.ethernet_delay_s +
   robot_movement_delay_s` must cover (plus the few ms from the gate to the send). The
   executor's own `[GATE] dispatch_to_contact_s` is quantised by its poll period; this is not.
2. **Camera vs belt encoder scale** (open-issues C8). For every part tracked across the
   camera window, the distance the camera saw it travel divided by the encoder's
   displacement over the same time. 1.0 means `vision.pixels_per_mm` and the belt feedback
   agree; otherwise one of them is off by that factor, and only a tape measurement says which.
3. **Arm cycle** (open-issues C7): per pick, goto send -> parked -> pick send -> contact ->
   over the bin -> next goto send. Without the wait for the part, grab = goto + contact and
   occupancy = goto + pick-to-bin + setup (the back-to-back bin -> next goto gap): their maxima
   are `scheduling.arm_cycle.grab_worst_s` / `occupancy_worst_s`, the gap's median
   `scheduling.setup_time_s`.
4. **Unmodelled camera latency**, at every belt start or stop while a part sits in the camera
   window. A tracked part's u is its camera position plus the belt travel since the frame's
   capture stamp, so `u - p` is continuous across a speed change only if the stamp is right.
   A stamp late by Δ makes `u - p` jump by -Δ·(v_after - v_before) at the change; the tool
   fits a line to `u - p` on each side, extrapolates both to the change and reports Δ.
   The `test_camera_latency` scenario produces exactly such a log (and prints the result
   itself); any run where the belt starts or stops with parts in view works too.

    python3 -m modules.tools.pick_timing log/20261002-171446_console
    python3 -m modules.tools.pick_timing                # newest folder under log/
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
from pathlib import Path

from modules.core.latency import change_lags, median_lag_s
from modules.runlog import REPO_ROOT

# A command 3 whose lowest point is below this is a pick phase (mm, robot frame).
_PICK_Z_MAX = -295.0
# A pose this far below the starting z means the arm has started to move (mm).
_MOTION_START_MM = 0.5
# The bottom is the first sample within this of the lowest z (mm).
_BOTTOM_BAND_MM = 0.5
# Window searched after each send (s).
_WINDOW_S = 1.0
# A track must cover at least this much encoder travel to give a scale sample (mm).
_MIN_TRAVEL_MM = 20.0
# Arrival at a waypoint: the arm first leaves it by this much, then comes within the second (mm).
_ARRIVAL_LEAVE_MM = 2.0
_ARRIVAL_MM = 1.0
# A bin -> next goto gap shorter than this means the next part was already waiting (s).
_BACK_TO_BACK_S = 0.5
# Longer than this between two sightings ends a track's continuous stretch (s).
_MAX_GAP_S = 0.5


def _rows(path: Path) -> list[dict[str, str]]:
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def pick_latencies(log_dir: Path) -> list[dict[str, float]]:
    poses = [(float(r["t_mono"]), float(r["z"])) for r in _rows(log_dir / "pose.csv") if r.get("z")]
    times = [t for t, _ in poses]
    out = []
    with open(log_dir / "commands.jsonl", encoding="utf-8") as handle:
        for line in handle:
            cmd = json.loads(line)
            pkg = cmd.get("package") or {}
            if cmd.get("commandID") != 3 or not cmd.get("ok"):
                continue
            n = max(int(pkg.get("argument_number") or 0), 1)
            if min(pkg.get("argument_z", [0.0])[:n]) > _PICK_Z_MAX:
                continue
            t_send = float(cmd["t_mono_send"])
            i = bisect.bisect_left(times, t_send)
            window = [(t, z) for t, z in poses[i:] if t <= t_send + _WINDOW_S]
            if len(window) < 5:
                continue
            z0 = window[0][1]
            z_min = min(z for _, z in window)
            t_move = next((t for t, z in window if z < z0 - _MOTION_START_MM), None)
            t_bottom = next(t for t, z in window if z <= z_min + _BOTTOM_BAND_MM)
            if t_move is None:
                continue
            out.append({"t_send": t_send, "rtt_s": float(cmd["rtt_s"]), "send_to_motion_s": t_move - t_send,
                        "send_to_bottom_s": t_bottom - t_send, "z_min": z_min})
    return out


class _Belt:
    """Belt position and speed from status.csv, interpolated in time."""

    def __init__(self, log_dir: Path) -> None:
        rows = [r for r in _rows(log_dir / "status.csv") if r.get("conveyor_position")]
        self.times = [float(r["t_mono"]) for r in rows]
        self.positions = [float(r["conveyor_position"]) for r in rows]
        self.speeds = [float(r["speed_current"] or 0.0) for r in rows]

    def __len__(self) -> int:
        return len(self.times)

    def at(self, t: float) -> float:
        times, positions = self.times, self.positions
        i = min(max(bisect.bisect_left(times, t), 1), len(times) - 1)
        t0, t1 = times[i - 1], times[i]
        return positions[i - 1] + (positions[i] - positions[i - 1]) * ((t - t0) / (t1 - t0) if t1 > t0 else 0.0)

    def samples(self) -> list[tuple[float, float, float]]:
        return list(zip(self.times, self.positions, self.speeds))


def _roi_tracks(log_dir: Path) -> dict[tuple[str, int], list[tuple[float, float]]]:
    """(t_mono, u) of every part while the camera sees it, keyed by (id, 100 s bucket):
    ids restart with each scenario, the bucket keeps reused ids apart."""
    tracks: dict[tuple[str, int], list[tuple[float, float]]] = {}
    with open(log_dir / "console.log", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if "[DETECT]" not in line or not line[:1].isdigit():
                continue
            try:
                t_mono = float(line.split()[1])
                objects = json.loads(line.split("[DETECT] ", 1)[1]).get("objects", [])
            except (ValueError, IndexError):
                continue                  # a line interleaved with another thread's output
            for obj in objects:
                if obj.get("zone") == "ROI":
                    tracks.setdefault((obj["id"], int(t_mono // 100)), []).append((t_mono, obj["u"]))
    return tracks


def camera_lags(log_dir: Path) -> list[dict[str, object]]:
    belt = _Belt(log_dir)
    if len(belt) < 2:
        return []
    offsets = {key: [(t, u - belt.at(t)) for t, u in samples]
               for key, samples in _roi_tracks(log_dir).items()}
    return change_lags(belt.samples(), offsets)


def arm_cycles(log_dir: Path) -> list[dict[str, float]]:
    """Milestones of every goto + pick pair, from commands.jsonl and the pose stream."""
    poses = [(float(r["t_mono"]), float(r["x"]), float(r["y"]), float(r["z"]))
             for r in _rows(log_dir / "pose.csv") if r.get("z")]
    times = [p[0] for p in poses]
    with open(log_dir / "commands.jsonl", encoding="utf-8") as handle:
        cmds = [c for c in map(json.loads, handle) if c.get("commandID") == 3 and c.get("ok")]

    def points(cmd: dict) -> list[tuple[float, float, float]]:
        pkg = cmd["package"]
        n = max(int(pkg.get("argument_number") or 0), 1)
        return list(zip(pkg["argument_x"][:n], pkg["argument_y"][:n], pkg["argument_z"][:n]))

    def arrival(t0: float, target: tuple[float, float, float], t_end: float) -> float | None:
        left = False
        for t, x, y, z in poses[bisect.bisect_left(times, t0):]:
            if t > t_end:
                return None
            d = ((x - target[0]) ** 2 + (y - target[1]) ** 2 + (z - target[2]) ** 2) ** 0.5
            left = left or d > _ARRIVAL_LEAVE_MM
            if left and d < _ARRIVAL_MM:
                return t
        return None

    out = []
    for i in range(1, len(cmds)):
        goto, pick = points(cmds[i - 1]), points(cmds[i])
        if min(p[2] for p in pick) > _PICK_Z_MAX or min(p[2] for p in goto) <= _PICK_Z_MAX:
            continue
        t_goto, t_pick = cmds[i - 1]["t_mono_send"], cmds[i]["t_mono_send"]
        t_next = cmds[i + 1]["t_mono_send"] if i + 1 < len(cmds) else None
        parked = arrival(t_goto, goto[-1], t_pick + 0.5)
        at_bin = arrival(t_pick, pick[-1], (t_next or t_pick + 5.0) + 0.5)
        window = [(t, z) for t, _, _, z in poses[bisect.bisect_left(times, t_pick):]
                  if t <= t_pick + _WINDOW_S]
        if parked is None or at_bin is None or not window:
            continue
        z_min = min(z for _, z in window)
        contact = next(t for t, z in window if z <= z_min + _BOTTOM_BAND_MM)
        out.append({"goto_s": parked - t_goto, "contact_s": contact - t_pick, "to_bin_s": at_bin - t_pick,
                    "gap_s": (t_next - at_bin) if t_next is not None else math.inf})
    return out


def belt_scale_ratios(log_dir: Path) -> list[float]:
    belt = _Belt(log_dir)
    if len(belt) < 2:
        return []
    tracks = _roi_tracks(log_dir)
    ratios = []
    for samples in tracks.values():
        # A gap means the id was lost or reused: only the first continuous stretch counts.
        end = next((i for i in range(1, len(samples)) if samples[i][0] - samples[i - 1][0] > _MAX_GAP_S),
                   len(samples))
        (t0, u0), (t1, u1) = samples[0], samples[end - 1]
        travel = belt.at(t1) - belt.at(t0)
        if travel >= _MIN_TRAVEL_MM:
            ratios.append((u1 - u0) / travel)
    return ratios


def _newest_log() -> Path:
    folders = sorted(p for p in (REPO_ROOT / "log").iterdir() if p.is_dir())
    if not folders:
        raise SystemExit("no folder under log/")
    return folders[-1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("log_dir", nargs="?", type=Path, help="run-log folder (default: newest)")
    args = parser.parse_args()
    log_dir = args.log_dir or _newest_log()
    print(f"[PICK-TIMING] {log_dir}")

    picks = pick_latencies(log_dir)
    if picks:
        def med(key: str) -> float:
            return statistics.median(p[key] for p in picks)
        bottoms = [p["send_to_bottom_s"] for p in picks]
        print(f"  pick phases: {len(picks)}")
        print(f"  send -> motion   median {med('send_to_motion_s'):.3f} s")
        print(f"  send -> bottom   median {med('send_to_bottom_s'):.3f} s "
              f"(min {min(bottoms):.3f}, max {max(bottoms):.3f})")
        print(f"  command rtt      median {med('rtt_s'):.3f} s (write + the worker's status read)")
        print(f"  lowest z         median {med('z_min'):.1f} mm")
    else:
        print("  no pick phase with pose samples (pose stream off, or no picks)")

    cycles = arm_cycles(log_dir)
    gaps = [c["gap_s"] for c in cycles if c["gap_s"] < _BACK_TO_BACK_S]
    if cycles and gaps:
        setup = statistics.median(gaps)
        grab = [c["goto_s"] + c["contact_s"] for c in cycles]
        occupancy = [c["goto_s"] + c["to_bin_s"] + setup for c in cycles]
        print(f"  arm cycles: {len(cycles)} (no wait for the part)")
        print(f"  setup (bin -> next goto, {len(gaps)} back-to-back)  median {setup:.3f} s")
        print(f"  grab       median {statistics.median(grab):.3f} s  max {max(grab):.3f} s")
        print(f"  occupancy  median {statistics.median(occupancy):.3f} s  max {max(occupancy):.3f} s")
    elif cycles:
        print(f"  arm cycles: {len(cycles)}, none back-to-back: setup and occupancy need a busier run")

    ratios = belt_scale_ratios(log_dir)
    if ratios:
        ratio = statistics.median(ratios)
        spread = statistics.pstdev(ratios) if len(ratios) > 1 else 0.0
        print(f"  camera / encoder travel: median {ratio:.3f} (sd {spread:.3f}, {len(ratios)} tracks)")
        if abs(ratio - 1.0) > 0.02:
            print(f"    either vision.pixels_per_mm should be multiplied by {ratio:.3f}, or the belt "
                  f"feedback is short by that factor (Sysmac mm/rev, or conveyor.position_scale_mm "
                  f"x{ratio:.3f}); a tape measurement decides.")
    else:
        print("  no camera track long enough for a scale sample")

    lags = camera_lags(log_dir)
    if lags:
        for lag in lags:
            print(f"  belt {lag['v_before']:.0f} -> {lag['v_after']:.0f} mm/s at {lag['t']:.2f}, "
                  f"{lag['part'][0]}: u - p jumps {lag['jump_mm']:+.2f} mm -> unmodelled camera lag "
                  f"{float(lag['lag_s']) * 1000:+.0f} ms")
        print(f"  unmodelled camera lag: median {median_lag_s(lags) * 1000:+.0f} ms "
              f"({len(lags)} samples; > 0 = detections are stamped later than the true capture)")
    else:
        print("  no belt start/stop with a part in the camera window (see the docstring for the procedure)")


if __name__ == "__main__":
    main()
