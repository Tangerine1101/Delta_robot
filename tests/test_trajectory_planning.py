import math
import unittest

from modules.conveyor import ConveyorFrame
from modules.image_processing import SimulatedImageProcessing
from modules.scheduler import (
    PickPlan,
    RealRobotExecutor,
    SchedulerSettings,
    SpeedSample,
    TrajectoryPoint,
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


class _StubSpeedSource:
    """Minimal speed_source for exercising the dispatch-time re-prediction."""

    def __init__(self, position_mm, speed_uv):
        self._position_mm = position_mm
        self._speed_uv = speed_uv

    def sample(self, now):
        return SpeedSample(
            vx=0.0,
            vy=self._speed_uv,
            timestamp=now,
            position_mm=self._position_mm,
            speed_uv=self._speed_uv,
        )


class RepredictionLeadTests(unittest.TestCase):
    """Layer-2 fix: the dispatch-time re-prediction must budget the full pick-phase
    approach flight (parked hover -> descent traverse), not just command_delay +
    soft_start. See doc/bug_report_final.md and doc/video_analysis_report.md."""

    def _executor(self, settings, frame, speed_source):
        executor = RealRobotExecutor(
            dispatch=lambda packet: None,
            request_status=lambda: {},
            interpolar_points=7,
            wait_margin_s=1.0,
            status_poll_interval_s=0.05,
        )
        executor.frame = frame
        executor.settings = settings
        executor.speed_source = speed_source
        return executor

    def _plan(self, settings, hover_xy, u_anchor, v_anchor, belt_pos, speed_uv):
        last_goto = TrajectoryPoint(hover_xy[0], hover_xy[1], settings.pre_pick_height, 0, 0.1)
        return PickPlan(
            plan_id="plan-test",
            object_id="obj-1",
            object_type="object_A",
            detected_at=0.0,
            source_position_2d=(u_anchor, v_anchor),
            cycle_start_position=settings.home_position,
            assumed_speed=(0.0, speed_uv),
            predicted_pick_time=0.0,
            pick_dispatch_time=0.0,
            predicted_pick_position_2d=(hover_xy[0], hover_xy[1], settings.pickup_height),
            sorting_position=settings.sorting_positions["object_A"],
            trajectory_goto=[last_goto],
            trajectory_pick=[last_goto],
            object_uv_anchor=(u_anchor, v_anchor),
            belt_pos_anchor=belt_pos,
        )

    def test_lead_includes_pick_traverse_when_object_drifted(self):
        settings = _settings()
        frame = ConveyorFrame()
        speed_uv, v_anchor, u_now, belt_pos = 50.0, 65.0, 290.0, 1000.0
        speed_source = _StubSpeedSource(belt_pos, speed_uv)
        executor = self._executor(settings, frame, speed_source)

        # Park the hover ~100 mm downstream of the object so the pick phase needs a
        # real horizontal traverse — the multi-object failure case.
        hover_xy = frame.to_robot(390.0, v_anchor)
        plan = self._plan(settings, hover_xy, u_now, v_anchor, belt_pos, speed_uv)

        packet = executor._repredicted_pick_packet(plan, _trajectory_packet_dummy())
        self.assertIsNotNone(packet)

        new_u, _ = frame.to_conveyor(packet["argument_x"][0], packet["argument_y"][0])
        command_delay = settings.robot_movement_delay_s + settings.ethernet_delay_s
        old_u_contact = u_now + speed_uv * (command_delay + settings.interp_soft_start_s)

        # Corrected lead places the descent further downstream than the old
        # soft-start-only budget, and still inside the workspace window.
        self.assertGreater(new_u, old_u_contact + 1.0)
        u_min, u_max = settings.workspace_window_uv[0], settings.workspace_window_uv[1]
        self.assertTrue(u_min <= new_u <= u_max)

    def test_lead_degrades_to_soft_start_when_arm_already_above_object(self):
        settings = _settings()
        frame = ConveyorFrame()
        speed_uv, v_anchor, u_now, belt_pos = 50.0, 65.0, 290.0, 1000.0
        speed_source = _StubSpeedSource(belt_pos, speed_uv)
        executor = self._executor(settings, frame, speed_source)

        # Park the hover exactly where the old soft-start-only budget predicts the
        # descent: zero traverse -> the new lead must collapse to the old value.
        command_delay = settings.robot_movement_delay_s + settings.ethernet_delay_s
        u_expected = u_now + speed_uv * (command_delay + settings.interp_soft_start_s)
        hover_xy = frame.to_robot(u_expected, v_anchor)
        plan = self._plan(settings, hover_xy, u_now, v_anchor, belt_pos, speed_uv)
        # pre_pick hover z differs from pickup z; align so the traverse distance is ~0.
        plan.trajectory_goto = [
            TrajectoryPoint(hover_xy[0], hover_xy[1], settings.pickup_height, 0, 0.1)
        ]

        packet = executor._repredicted_pick_packet(plan, _trajectory_packet_dummy())
        self.assertIsNotNone(packet)
        new_u, _ = frame.to_conveyor(packet["argument_x"][0], packet["argument_y"][0])
        self.assertAlmostEqual(new_u, u_expected, delta=1.0)


def _trajectory_packet_dummy():
    return {"commandID": 3, "argument_number": 0, "argument_x": [], "argument_y": [],
            "argument_z": [], "argument_e": [], "argument_time": []}


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
