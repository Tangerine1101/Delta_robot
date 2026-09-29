"""The shared cell state of a realtime run.

The perception thread writes the belt sample, the robot pose and the tracker; the main thread
(decision loop and pick executor) reads them. Every access goes through `state_lock`; PLC I/O
goes through `ipc_lock` and is never done while holding `state_lock`, so the two locks are
never nested.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any

from modules.core.frames import ConveyorFrame
from modules.core.tracking import BeltTracker
from modules.settings import Point


@dataclass(frozen=True)
class SpeedSample:
    vx: float
    vy: float
    timestamp: float
    position_mm: float = 0.0        # belt position along +u (mm)
    speed_uv: float = 0.0           # belt speed along +u (mm/s)


@dataclass
class RealtimeState:
    tracker: BeltTracker
    frame: ConveyorFrame
    ipc_lock: threading.Lock = field(default_factory=threading.Lock)
    state_lock: threading.Lock = field(default_factory=threading.Lock)
    stop_event: threading.Event = field(default_factory=threading.Event)
    latest_speed: SpeedSample | None = None
    belt_position_mm: float = 0.0
    belt_speed_mm_s: float = 0.0
    # Live PLC speed_current feedback (mm/s); None until a Siemens status arrives.
    belt_speed_measured_mm_s: float | None = None
    robot_pose: Point | None = None
    end_effector: int | None = None
    claimed_object_ids: set[str] = field(default_factory=set)
    # True from goto dispatch until cup contact (or abort): the belt setpoint must not move,
    # so the belt is steady from plan through gate lead to grip (basis-theory §6.5, L9).
    pick_committed: bool = False
    # Rolling average pick-cycle wall time (s) for the dashboard's Performance card.
    recent_pick_cycle_s: float = 0.0
    # Live Siemens suction-cup angle (R-frame DEGREES); None until the first Siemens status
    # arrives. Read by the [ROTATE] calibration log and the home-tolerance warning.
    rotate_current_deg: float | None = None


@dataclass
class SchedulerMetrics:
    total_detections: int = 0
    planned_picks: int = 0
    completed_picks: int = 0
    stale_drops: int = 0
    skipped_unknown_type: int = 0
    skipped_outside_workspace: int = 0
    total_planning_latency: float = 0.0
    planning_events: int = 0
    queue_peak: int = 0

    def as_dict(self) -> dict[str, Any]:
        average_latency = (
            self.total_planning_latency / self.planning_events if self.planning_events else 0.0
        )
        return {
            "total_detections": self.total_detections,
            "planned_picks": self.planned_picks,
            "completed_picks": self.completed_picks,
            "stale_drops": self.stale_drops,
            "skipped_unknown_type": self.skipped_unknown_type,
            "skipped_outside_workspace": self.skipped_outside_workspace,
            "average_planning_latency_s": round(average_latency, 4),
            "queue_peak": self.queue_peak,
        }
