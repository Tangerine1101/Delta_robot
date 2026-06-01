import math
import unittest

from modules.image_processing import SimulatedImageProcessing
from modules.scheduler import (
    SchedulerSettings,
    _build_goto_geometry,
    _build_pick_geometry,
    _segment_duration,
)


def _settings(**overrides):
    values = {
        "home_position": (0.0, 0.0, -290.0),
        "clearance_height": -290.0,
        "slope_transition_height": -295.0,
        "pickup_height": -310.0,
        "pre_pick_height": -300.0,
        "place_height": -290.0,
        "corner_blend_xy": 35.0,
        "intercept_lead_time_s": 0.14,
        "release_descent_time_s": 0.14,
        "nominal_xy_speed": 50.0,
        "nominal_z_speed": 50.0,
        "stale_timeout_s": 5.0,
        "speed_timeout_s": 1.0,
        "poll_interval_s": 0.05,
        "default_speed": (0.0, 80.0),
        "robot_movement_delay_s": 0.05,
        "ethernet_delay_s": 0.002,
        "pickup_window_x": (-120.0, 50.0),
        "pickup_window_y": (-60.0, 60.0),
        "accuracy_points": [
            (40.0, -60.0, -300.0),
            (0.0, 0.0, -290.0),
            (-40.0, 60.0, -300.0),
        ],
        "log_path": "data.log",
        "object_type_map": {"object_A": "object_A"},
        "object_thickness_mm": {"object_A": 0.0},
        "sorting_positions": {"object_A": (0.0, 90.0, -290.0)},
        "throughput_object_types": ["object_A"],
        "throughput_lanes": [-50.0, 0.0, 50.0],
        "throughput_spawn_x": -180.0,
        "throughput_spawn_y": -180.0,
        "throughput_emit_interval_s": 0.35,
        "accuracy_emit_interval_s": 0.8,
        "execution_margin_s": 1.0,
    }
    values.update(overrides)
    return SchedulerSettings(**values)


def _dxy(start, end):
    return math.hypot(end[0] - start[0], end[1] - start[1])


class TrajectoryGeometryTests(unittest.TestCase):
    def test_goto_and_pick_include_mandatory_3d_slopes(self):
        settings = _settings()
        start = (0.0, 0.0, -290.0)
        pick = (40.0, -60.0, -310.0)
        place = (0.0, 90.0, -290.0)

        goto_points = _build_goto_geometry(start, pick, settings)
        pick_points = _build_pick_geometry(pick, place, settings, goto_points)

        # Goto has 7 points
        self.assertEqual(len(goto_points), 7)
        # P1 -> P2 is diagonal slope up (XY moves, Z goes up to clearance)
        self.assertGreater(_dxy(goto_points[0], goto_points[1]), 0.0)
        self.assertGreater(goto_points[1][2] - goto_points[0][2], 0.0)

        # Pick has 7 points
        self.assertEqual(len(pick_points), 7)
        # P2 -> P3 is diagonal slope up (XY moves, Z goes up to clearance)
        self.assertGreater(_dxy(pick_points[1], pick_points[2]), 0.0)
        self.assertGreater(pick_points[2][2] - pick_points[1][2], 0.0)

        # pre_pick is higher than pickup (less negative)
        self.assertGreater(goto_points[6][2], pick_points[0][2])
        # slope transition height after pickup is higher than pickup
        self.assertGreater(pick_points[1][2], pick_points[0][2])

    def test_segment_duration_uses_slowest_axis_not_axis_sum(self):
        settings = _settings(nominal_xy_speed=50.0, nominal_z_speed=10.0)
        duration = _segment_duration((0.0, 0.0, -300.0), (30.0, 40.0, -320.0), settings)
        self.assertAlmostEqual(duration, 2.0)
        self.assertNotAlmostEqual(duration, 3.0)

    def test_segment_duration_keeps_minimum_time(self):
        self.assertAlmostEqual(
            _segment_duration((0.0, 0.0, -300.0), (0.0, 0.0, -300.0), _settings()),
            0.08,
        )


class SimulatedPerceptionTests(unittest.TestCase):
    def test_accuracy_points_cycle_in_order(self):
        start = 1000.0
        sim = SimulatedImageProcessing(
            "test_accuracy",
            {
                "throughput_object_types": ["object_A"],
                "accuracy_emit_interval_s": 0.8,
                "accuracy_points": [
                    (40.0, -60.0, -300.0),
                    (0.0, 0.0, -290.0),
                    (-40.0, 60.0, -300.0),
                ],
            },
            start,
        )

        detections = []
        for index in range(4):
            detections.extend(sim.poll(start + index * 0.8))

        self.assertEqual(
            [(d.x, d.y) for d in detections],
            [(40.0, -60.0), (0.0, 0.0), (-40.0, 60.0), (40.0, -60.0)],
        )

    def test_throughput_spawns_lanes_on_x_and_moves_along_y(self):
        sim = SimulatedImageProcessing(
            "test_throughput",
            {
                "throughput_object_types": ["object_A"],
                "throughput_lanes": [-50.0, 0.0, 50.0],
                "throughput_spawn_y": -180.0,
                "throughput_emit_interval_s": 0.35,
            },
            1000.0,
        )

        detections = sim.poll(1000.7)
        self.assertEqual(
            [(d.x, d.y) for d in detections],
            [(-50.0, -180.0), (0.0, -180.0), (50.0, -180.0)],
        )


if __name__ == "__main__":
    unittest.main()
