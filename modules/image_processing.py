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
    """Deterministic fake object stream for scheduler development."""

    def __init__(self, scenario_name: str, config: dict[str, Any], start_time: float) -> None:
        self.scenario_name = scenario_name
        self.config = config
        self.start_time = start_time
        self.next_emit_at = start_time
        self.counter = 0
        self.throughput_types = list(config.get("throughput_object_types", ["pcb1", "pcb2"]))
        self.throughput_lanes = list(config.get("throughput_lanes", [-60.0, 0.0, 60.0]))
        self.throughput_spawn_y = float(config.get("throughput_spawn_y", -180.0))
        self.throughput_spawn_x = float(config.get("throughput_spawn_x", -180.0))
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
            x, y = self.accuracy_points[(self.counter - 1) % len(self.accuracy_points)]
            object_type = self.throughput_types[(self.counter - 1) % len(self.throughput_types)]
            return ObjectDetection(
                object_id=f"accuracy-{self.counter:06d}",
                x=x,
                y=y,
                object_type=object_type,
                timestamp=timestamp,
            )

        lane = self.throughput_lanes[(self.counter - 1) % len(self.throughput_lanes)]
        object_type = self.throughput_types[(self.counter - 1) % len(self.throughput_types)]
        return ObjectDetection(
            object_id=f"throughput-{self.counter:06d}",
            x=float(lane),
            y=self.throughput_spawn_y,
            object_type=object_type,
            timestamp=timestamp,
        )
