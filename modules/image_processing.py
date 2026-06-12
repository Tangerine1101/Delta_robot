"""
Phase 1 image-processing stub.

Detection coordinates `(x, y)` produced by this module are interpreted as
C-frame `(u, v)` by `modules.conveyor.BeltTracker` and the scheduler. This is
a deliberate shortcut for Phase 1 wiring so the rest of the pipeline can be
exercised end to end. Phase 3 will replace this whole module with a real
YOLO + tracker that yields pixel-space detections and routes them through
`modules.conveyor.CameraFrame` to obtain C-frame coordinates.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ObjectDetection:
    object_id: str
    x: float
    y: float
    object_type: str
    timestamp: float
    confidence: float = 1.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "object_id": self.object_id,
            "x": self.x,
            "y": self.y,
            "object_type": self.object_type,
            "timestamp": self.timestamp,
            "confidence": self.confidence,
        }


class SimulatedImageProcessing:
    """Deterministic fake object stream for scheduler development.

    Phase 1: emits detections at C-frame `(u, v)` coordinates.
    - `throughput_spawn_y` is reused as the upstream `u_spawn`.
    - `throughput_lanes` is reused as the per-object `v_lane` values.
    - `accuracy_points` are read as `(u, v, _)` triples.
    """

    def __init__(self, scenario_name: str, config: dict[str, Any], start_time: float) -> None:
        self.scenario_name = scenario_name
        self.config = config
        self.start_time = start_time
        self.next_emit_at = start_time
        self.counter = 0
        self.throughput_types = list(config.get("throughput_object_types", ["pcb1", "pcb2"]))
        self.throughput_lanes = list(config.get("throughput_lanes", [-40.0, 0.0, 40.0]))
        self.u_spawn = float(config.get("throughput_spawn_y", -50.0))
        self.throughput_emit_interval_s = float(config.get("throughput_emit_interval_s", 0.35))
        self.accuracy_emit_interval_s = float(config.get("accuracy_emit_interval_s", 0.8))
        raw_points = config.get(
            "accuracy_points",
            [
                [40.0, -60.0, -300.0],
                [0.0, 0.0, -300.0],
                [-40.0, 60.0, -300.0],
            ],
        )
        self.accuracy_points = [(float(point[0]), float(point[1])) for point in raw_points]

    def poll(self, now: float) -> list[ObjectDetection]:
        detections: list[ObjectDetection] = []
        interval = self._scenario_interval()
        while now >= self.next_emit_at:
            detections.append(self._build_detection(self.next_emit_at))
            self.next_emit_at += interval
        return detections

    def _scenario_interval(self) -> float:
        if self.scenario_name == "test_accuracy":
            return self.accuracy_emit_interval_s
        return self.throughput_emit_interval_s

    def _build_detection(self, timestamp: float) -> ObjectDetection:
        self.counter += 1
        if self.scenario_name == "test_accuracy":
            u, v = self.accuracy_points[(self.counter - 1) % len(self.accuracy_points)]
            object_type = self.throughput_types[(self.counter - 1) % len(self.throughput_types)]
            return ObjectDetection(
                object_id=f"accuracy-{self.counter:06d}",
                x=u,
                y=v,
                object_type=object_type,
                timestamp=timestamp,
            )

        v_lane = self.throughput_lanes[(self.counter - 1) % len(self.throughput_lanes)]
        object_type = self.throughput_types[(self.counter - 1) % len(self.throughput_types)]
        return ObjectDetection(
            object_id=f"throughput-{self.counter:06d}",
            x=self.u_spawn,
            y=float(v_lane),
            object_type=object_type,
            timestamp=timestamp,
        )
