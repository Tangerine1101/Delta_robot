"""Virtual parts for the `simulate_feeder` scenario: a camera stand-in fed from a seed.

The arrival schedule (time, type, lane, heading) is drawn once, before the run, by a feeder of
`modules/core/feeders.py` — the same functions the sandbox uses — from `feeder.kind`,
`feeder.seed` and `feeder.kinds.<kind>`. A part lands at the camera origin (u = camera
`u_min`) at its arrival time and then rides the belt encoder, so on the cell the real belt
carries it and the real arm picks at it; nothing is on the belt physically.

`poll(now)` answers like the vision pipeline: one `ObjectDetection` per part inside the camera
window, in belt coordinates, stamped with its capture time. The same seed gives the same
arrivals, so two runs (two planners, two speed laws) see the same input.
"""

from __future__ import annotations

import csv
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from modules.core.feeders import FEEDERS, SpawnEvent, build_events
from modules.core.tracking import ObjectDetection
from modules.settings import Settings, SettingsError


@dataclass
class VirtualPart:
    part_id: str
    event: SpawnEvent
    t_spawn: float                  # run time it landed
    belt_at_spawn: float            # belt position when it landed at u0
    u0: float

    def u(self, belt_position: float) -> float:
        return self.u0 + belt_position - self.belt_at_spawn


def arrival_schedule(settings: Settings, horizon_s: float) -> list[SpawnEvent]:
    """The run's arrivals, deterministic in `feeder.seed`."""
    cfg = settings.feeder
    if cfg.kind not in FEEDERS:
        raise SettingsError(f"feeder.kind: unknown feeder '{cfg.kind}'. Known: {', '.join(sorted(FEEDERS))}")
    events = build_events(cfg.kind, dict(cfg.kinds.get(cfg.kind) or {}), random.Random(cfg.seed), horizon_s)
    unknown = sorted({e.part_type for e in events} - set(settings.object_types))
    if unknown:
        raise SettingsError(f"feeder.kinds.{cfg.kind}.types: {', '.join(unknown)} not in object_types")
    return events[:cfg.max_parts] if cfg.max_parts > 0 else events


class VirtualFeed:
    def __init__(self, settings: Settings, start_time: float,
                 position_at: Callable[[float], float | None], *, duration_s: float | None = None) -> None:
        cfg = settings.feeder
        self.start_time = start_time
        self.position_at = position_at
        self.events = arrival_schedule(settings, duration_s if duration_s else cfg.schedule_s)
        self.camera_window_uv = settings.conveyor.camera_window_uv
        self.period_s = max(cfg.detection_period_s, 1e-3)
        self.latency_s = max(cfg.detection_latency_s, 0.0)
        self.noise_mm = max(cfg.position_noise_mm, 0.0)
        self._noise = random.Random(cfg.seed + 7919)
        self._next_event = 0
        self._next_frame = 0.0
        self._visible: list[VirtualPart] = []
        self.spawned: list[VirtualPart] = []

    @property
    def exhausted(self) -> bool:
        """Every scheduled part has landed and left the camera view."""
        return self._next_event >= len(self.events) and not self._visible

    def poll(self, now: float) -> list[ObjectDetection]:
        belt_now = self.position_at(now)
        if belt_now is None:
            return []           # no belt sample yet: hold the schedule
        u_min, u_max, v_min, v_max = self.camera_window_uv
        t_run = now - self.start_time
        while self._next_event < len(self.events) and self.events[self._next_event].time_s <= t_run:
            event = self.events[self._next_event]
            self._next_event += 1
            landed = self.position_at(self.start_time + event.time_s)
            part = VirtualPart(f"vf{self._next_event:04d}", event, round(event.time_s, 3),
                               belt_now if landed is None else landed, u_min)
            self._visible.append(part)
            self.spawned.append(part)
        if now < self._next_frame:
            return []
        self._next_frame = now + self.period_s
        capture_t = now - self.latency_s
        belt = self.position_at(capture_t)
        if belt is None:
            belt = belt_now
        self._visible = [p for p in self._visible if p.u(belt) <= u_max]
        out = []
        for part in self._visible:
            u, v = part.u(belt), part.event.v_mm
            if not (v_min <= v <= v_max):
                continue
            if self.noise_mm:
                u += self._noise.gauss(0.0, self.noise_mm)
                v += self._noise.gauss(0.0, self.noise_mm)
            out.append(ObjectDetection(object_id=part.part_id, x=u, y=v, object_type=part.event.part_type,
                                       timestamp=capture_t, confidence=1.0,
                                       angle_deg=part.event.heading_deg))
        return out

    def write_arrivals(self, path: Path) -> None:
        """The arrivals that landed: replayable as a sandbox `script` feeder."""
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("part_id", "t_s", "type", "v_mm", "heading_deg", "regime"))
            for part in self.spawned:
                e = part.event
                writer.writerow((part.part_id, part.t_spawn, e.part_type, round(e.v_mm, 2),
                                 round(e.heading_deg, 2), e.regime))

    def describe(self) -> dict[str, Any]:
        return {"scheduled": len(self.events), "landed": len(self.spawned)}

    # The dashboard draws virtual parts in its cell view; there is no camera image.
    def jpeg_frame(self) -> bytes | None:
        return None

    def stop(self) -> None:
        pass
