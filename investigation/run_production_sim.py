"""Drive the real `production` loop against the virtual cell and measure it.

This is the dry-run `open-issues.md` L8 says does not exist.  Everything on the
control side is the shipped code -- `_run_realtime_pick_loop`, the perception
thread, `RealtimePickExecutor`, `_object_pick_gate_status`, the adaptive speed
law.  Only the hardware is fake.

    python3 -m investigation.run_production_sim --belt 120 --duration 60

Reported per run:

  contacts        how many grabs were actually commanded
  hit rate        fraction where the cup came down within the grip tolerance
  along-belt err  signed, positive = the board was downstream of the cup, i.e.
                  the grab fired late.  This is the number the whole
                  investigation is about.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import re
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field, replace
from typing import Any

from investigation.virtual_cell import (
    PlantProfile,
    Spawner,
    VirtualCamera,
    VirtualCell,
)

import modules.scheduler as RS
from modules.conveyor import BeltPositionTracker, BeltTracker, ConveyorFrame
from modules.EthernetCom import load_config


@dataclass
class RunResult:
    label: str
    belt_mm_s: float
    duration_s: float
    spawned: int
    contacts: int
    hits: int
    along_errors: list[float] = field(default_factory=list)
    cross_errors: list[float] = field(default_factory=list)
    total_errors: list[float] = field(default_factory=list)
    gate_lines: list[dict] = field(default_factory=list)
    warnings: dict[str, int] = field(default_factory=dict)
    plans: int = 0
    log_path: str | None = None
    park_offsets_mm: list[float] = field(default_factory=list)
    gate_overshoots_mm: list[float] = field(default_factory=list)

    @property
    def hit_rate(self) -> float:
        return self.hits / self.contacts if self.contacts else 0.0

    def summary(self) -> str:
        def stat(values: list[float]) -> str:
            if not values:
                return "n/a"
            return (f"mean {statistics.fmean(values):+7.2f}  "
                    f"median {statistics.median(values):+7.2f}  "
                    f"min {min(values):+7.2f}  max {max(values):+7.2f}")

        warn = ", ".join(f"{k}={v}" for k, v in sorted(self.warnings.items())) or "none"
        return (
            f"\n=== {self.label} ===\n"
            f"  belt setpoint       : {self.belt_mm_s:.0f} mm/s over {self.duration_s:.0f}s\n"
            f"  boards fed / planned: {self.spawned} / {self.plans}\n"
            f"  grabs commanded     : {self.contacts}\n"
            f"  landed on the board : {self.hits}  ({self.hit_rate * 100:.0f}%)\n"
            f"  along-belt error mm : {stat(self.along_errors)}\n"
            f"  cross-belt error mm : {stat(self.cross_errors)}\n"
            f"  radial   error mm   : {stat(self.total_errors)}\n"
            f"  park offset at grab : {stat(self.park_offsets_mm)}\n"
            f"  board past cup @grab: {stat(self.gate_overshoots_mm)}\n"
            f"  warnings            : {warn}\n"
        )


_WARN_RE = re.compile(r"^\[WARN\] (\{.*\})$")
_GATE_RE = re.compile(r"^\[GATE\] (\{.*\})$")
_PLAN_RE = re.compile(r"^\[PLAN\] (\{.*\})$")


def _parse_log(text: str) -> tuple[list[dict], dict[str, int], int]:
    gates: list[dict] = []
    warnings: dict[str, int] = {}
    plans = 0
    for line in text.splitlines():
        m = _GATE_RE.match(line)
        if m:
            with contextlib.suppress(json.JSONDecodeError):
                gates.append(json.loads(m.group(1)))
            continue
        m = _WARN_RE.match(line)
        if m:
            with contextlib.suppress(json.JSONDecodeError):
                event = json.loads(m.group(1)).get("event", "unknown")
                warnings[event] = warnings.get(event, 0) + 1
            continue
        if _PLAN_RE.match(line):
            plans += 1
    return gates, warnings, plans


def run_once(
    *,
    label: str,
    duration_s: float = 45.0,
    belt_static_mm_s: float | None = None,
    adaptive: bool | None = None,
    oblique: bool | None = None,
    plant: PlantProfile | None = None,
    settings_overrides: dict[str, Any] | None = None,
    arrival_tolerance_mm: float = 15.0,
    arrival_tolerance_max_mm: float = 50.0,
    feed_interval_s: float = 2.5,
    camera_latency_s: float = 0.090,
    grip_tolerance_mm: float = 12.7,
    log_path: str | None = None,
    quiet: bool = True,
) -> RunResult:
    settings = RS.SchedulerSettings.from_config(load_config())
    overrides: dict[str, Any] = dict(settings_overrides or {})
    if belt_static_mm_s is not None:
        overrides["belt_speed_static_mm_s"] = belt_static_mm_s
    if adaptive is not None:
        overrides["adaptive_speed_enabled"] = adaptive
    if oblique is not None:
        overrides["oblique_descent_enabled"] = oblique
    if overrides:
        settings = replace(settings, **overrides)

    frame = ConveyorFrame()
    cell = VirtualCell(
        settings, frame,
        plant=plant or PlantProfile(),
        camera_latency_s=camera_latency_s,
        grip_tolerance_mm=grip_tolerance_mm,
    )
    camera = VirtualCamera(cell, settings)
    spawner = Spawner(cell, interval_s=feed_interval_s)

    executor = RS.RealtimePickExecutor(
        cell.dispatch,
        cell.request_status,
        interpolar_points=7,
        wait_margin_s=settings.execution_margin_s,
        status_poll_interval_s=settings.poll_interval_s,
        position_tolerance_mm=arrival_tolerance_mm,
        position_tolerance_max_mm=arrival_tolerance_max_mm,
        tolerance_speed_min_mm_s=settings.belt_speed_min_mm_s,
        tolerance_speed_max_mm_s=settings.belt_speed_max_mm_s,
        rotate_home_tolerance_deg=settings.rotate_home_tolerance_deg,
        rotate_offset_rad=settings.rotate_offset_rad,
        rotate_sign=settings.rotate_sign,
        rotate_refresh_max_delta_deg=15.0,
    )

    tracker = BeltTracker(
        frame,
        workspace_window_uv=settings.workspace_window_uv,
        stale_timeout_s=settings.stale_timeout_s,
        camera_window_uv=settings.camera_window_uv,
    )
    decoder = BeltPositionTracker(velocity_ema_alpha=settings.belt_velocity_ema_alpha)
    speed_source = RS.ConveyorSpeedSource(
        executor.request_status, frame, decoder, "production",
        position_scale_mm=settings.conveyor_position_scale_mm,
    )
    scheduler = RS.PickScheduler(settings, 7, frame, tracker, "production")

    def event_sink(kind: str, payload: dict) -> None:
        # The loop announces every commitment here; the plant needs to know
        # which board a grab was aimed at in order to score it.
        if kind == "plan":
            cell.current_plan_object_id = payload.get("object_id")

    cell.start()
    stop_feed = threading.Event()

    def feeder() -> None:
        while not stop_feed.is_set():
            spawner.tick(time.monotonic())
            time.sleep(0.02)

    feed_thread = threading.Thread(target=feeder, daemon=True)
    feed_thread.start()

    buffer = io.StringIO()
    start = time.monotonic()
    try:
        if quiet:
            with contextlib.redirect_stdout(buffer):
                RS._run_realtime_pick_loop(
                    "production", settings, 7, executor, camera, frame, scheduler,
                    speed_source, start, duration_s, event_sink,
                )
        else:
            RS._run_realtime_pick_loop(
                "production", settings, 7, executor, camera, frame, scheduler,
                speed_source, start, duration_s, event_sink,
            )
    finally:
        stop_feed.set()
        feed_thread.join(timeout=1.0)
        cell.stop()

    text = buffer.getvalue()
    if log_path:
        with open(log_path, "w", encoding="utf-8") as handle:
            handle.write(text)
    gates, warnings, plans = _parse_log(text)

    scored = [c for c in cell.contacts if c.along_belt_error_mm is not None]
    result = RunResult(
        label=label,
        belt_mm_s=belt_static_mm_s if belt_static_mm_s is not None
        else settings.belt_speed_static_mm_s,
        duration_s=duration_s,
        spawned=cell.spawned,
        contacts=len(cell.contacts),
        hits=sum(1 for c in cell.contacts if c.hit),
        along_errors=[c.along_belt_error_mm for c in scored],
        cross_errors=[c.cross_belt_error_mm for c in scored],
        total_errors=[c.error_mm for c in scored],
        gate_lines=gates,
        warnings=warnings,
        plans=plans,
        log_path=log_path,
        park_offsets_mm=[d[0] for d in cell.grab_dispatches],
        gate_overshoots_mm=[d[1] for d in cell.grab_dispatches],
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--belt", type=float, default=None,
                        help="static belt speed (mm/s); default = config")
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--feed", type=float, default=2.5, help="seconds between boards")
    parser.add_argument("--adaptive", action="store_true")
    parser.add_argument("--no-adaptive", dest="adaptive", action="store_false")
    parser.add_argument("--oblique", action="store_true", default=None)
    parser.add_argument("--command-delay", type=float, default=0.10,
                        help="TRUE dispatch->motion latency of the plant (s)")
    parser.add_argument("--servo-lag", type=float, default=0.0,
                        help="extra time past State 10 before the cup is at Pos[0]")
    parser.add_argument("--log", default=None)
    parser.add_argument("--verbose", action="store_true")
    parser.set_defaults(adaptive=False)
    args = parser.parse_args()

    plant = PlantProfile(command_delay_s=args.command_delay, servo_lag_s=args.servo_lag)
    result = run_once(
        label=f"belt={args.belt} adaptive={args.adaptive} oblique={args.oblique}",
        duration_s=args.duration,
        belt_static_mm_s=args.belt,
        adaptive=args.adaptive,
        oblique=args.oblique,
        plant=plant,
        feed_interval_s=args.feed,
        log_path=args.log,
        quiet=not args.verbose,
    )
    print(result.summary())


if __name__ == "__main__":
    main()
