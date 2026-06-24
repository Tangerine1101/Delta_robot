import math
import unittest

from modules.image_processing import SimulatedImageProcessing
from modules.scheduler import (
    SchedulerSettings,
    _build_goto_geometry,
    _build_pick_geometry,
    _segment_duration,
    _segment_profile_time,
    _trajectory_total_time,
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
        "workspace_window_uv": (275.0, 400.0, 10.0, 120.0),
        "camera_window_uv": (50.0, 250.0, -75.0, 75.0),
        "conveyor_length_mm": 800.0,
        "conveyor_position_scale_mm": 1.0,
        "object_dimensions": {"object_A": (30.0, 40.0)},
        "accuracy_points": [
            (40.0, -60.0, -300.0),
            (0.0, 0.0, -290.0),
            (-40.0, 60.0, -300.0),
        ],
        "accuracy_points_uv": [],
        "accuracy_spawn_uv": [(300.0, 40.0)],
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


class InterpolatorTimingTests(unittest.TestCase):
    """Port of the PLC MC_Inter_Curve_Vel timing model (doc/PLC_Program_description)."""

    def test_scurve_with_cruise_matches_closed_form(self):
        # 200 mm, V=300, A=D=1000: t_acc=t_dec=0.45 s, s_acc=s_dec=67.5 mm,
        # s_run=65 mm -> t_run=0.21667 s, total=1.11667 s.
        t = _segment_profile_time(200.0, 0.0, 0.0, 300.0, 1000.0, 1000.0)
        self.assertAlmostEqual(t, 0.45 + 65.0 / 300.0 + 0.45, places=4)

    def test_short_segment_uses_triangular_fallback(self):
        # 50 mm < l_min(135 mm) -> v_peak < v_max, no cruise.
        t = _segment_profile_time(50.0, 0.0, 0.0, 300.0, 1000.0, 1000.0)
        v_peak = math.sqrt(50.0 / (0.75 * (2.0 / 1000.0)))
        self.assertAlmostEqual(t, 2.0 * (1.5 * v_peak / 1000.0), places=4)

    def test_zero_length_segment_is_zero(self):
        self.assertEqual(_segment_profile_time(0.0, 0.0, 0.0, 300.0, 1000.0, 1000.0), 0.0)

    def test_trajectory_time_includes_soft_start_and_is_positive(self):
        settings = _settings()
        start = (0.0, 0.0, -290.0)
        pick = (120.0, 60.0, -310.0)
        goto = _build_goto_geometry(start, pick, settings)
        total = _trajectory_total_time(goto, settings)
        # Strictly greater than the lone 80 ms soft-start, and faster than the same
        # path run as independent stop-and-go segments (blending saves time).
        self.assertGreater(total, settings.interp_soft_start_s)
        stop_and_go = settings.interp_soft_start_s + sum(
            _segment_profile_time(
                math.dist(goto[i], goto[i + 1]), 0.0, 0.0,
                settings.interp_v_max, settings.interp_a_max, settings.interp_d_max,
            )
            for i in range(len(goto) - 1)
        )
        self.assertLessEqual(total, stop_and_go + 1e-9)


class SimulatedPerceptionTests(unittest.TestCase):
    def test_accuracy_points_cycle_in_order(self):
        start = 1000.0
        # test_accuracy now spawns objects at C-frame (u, v) from accuracy_spawn_uv;
        # detection.x = u, detection.y = v.
        sim = SimulatedImageProcessing(
            "test_accuracy",
            {
                "throughput_object_types": ["object_A"],
                "accuracy_emit_interval_s": 0.8,
                "accuracy_spawn_uv": [
                    (40.0, -60.0),
                    (0.0, 0.0),
                    (-40.0, 60.0),
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

    def test_throughput_spawns_at_u_spawn_across_v_lanes(self):
        # C-frame convention: throughput_spawn_y is the upstream u_spawn (shared by
        # every object) and throughput_lanes are the per-object v lanes, so
        # detection.x = u_spawn and detection.y = lane.
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
            [(-180.0, -50.0), (-180.0, 0.0), (-180.0, 50.0)],
        )


if __name__ == "__main__":
    unittest.main()
