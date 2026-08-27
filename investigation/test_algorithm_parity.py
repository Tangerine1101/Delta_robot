"""Algorithm parity between the delta-robot repo and the scheduling sandbox.

Both repositories implement the same seven pieces of logic.  This file runs the
two implementations against identical inputs and reports where they agree and
where they do not.  It is read-only with respect to both trees.

Run:  python3 -m investigation.test_algorithm_parity
"""

from __future__ import annotations

import math
import unittest

from investigation._bridge import (
    robot_conveyor,
    robot_scheduler,
    robot_settings,
    sandbox_config,
    sandbox_conveyor,
    sandbox_kinetic,
    sandbox_scheduler,
)

RS = robot_scheduler
SK = sandbox_kinetic


def _sandbox_profile(which: str = "model") -> "SK.MotionProfile":
    return SK.MotionProfile.from_config(sandbox_config(), which)


def _robot_points_goto(settings, start, pick):
    return RS._build_goto_geometry(start, pick, settings)


def _robot_points_pick(settings, contact, sort_pos):
    return RS._build_pick_geometry(contact, sort_pos, settings, [])


class CoordinateFrameParity(unittest.TestCase):
    """C-frame <-> R-frame transform, and the belt-direction unit vector."""

    def setUp(self) -> None:
        self.robot_frame = robot_conveyor.ConveyorFrame()
        cfg = sandbox_config()
        self.sandbox_frame = sandbox_conveyor.ConveyorFrame.from_config(cfg["conveyor"]["frame"])

    def test_forward_transform_matches(self):
        for u, v in [(188.0, 0.0), (275.5, 62.5), (363.0, 125.0), (0.0, 88.0)]:
            with self.subTest(u=u, v=v):
                rx, ry = self.robot_frame.to_robot(u, v)
                sx, sy = self.sandbox_frame.to_robot(u, v)
                self.assertAlmostEqual(rx, sx, places=6)
                self.assertAlmostEqual(ry, sy, places=6)

    def test_inverse_transform_matches(self):
        for x, y in [(0.0, 0.0), (100.0, -50.0), (-60.0, 120.0)]:
            with self.subTest(x=x, y=y):
                ru, rv = self.robot_frame.to_conveyor(x, y)
                su, sv = self.sandbox_frame.to_conveyor(x, y)
                self.assertAlmostEqual(ru, su, places=6)
                self.assertAlmostEqual(rv, sv, places=6)

    def test_belt_direction_matches(self):
        sx, sy = self.sandbox_frame.belt_direction_robot()
        self.assertAlmostEqual(self.robot_frame.u_hat[0], sx, places=9)
        self.assertAlmostEqual(self.robot_frame.u_hat[1], sy, places=9)

    def test_transform_is_a_reflection_in_both(self):
        """Both blocks have determinant -1, not +1.

        Documented as 'a plain 2-D rotation' in both repos.  It is not: the
        (u,v)->(x,y) block is a rotation composed with a reflection, so a
        heading measured CCW in the C-frame comes out reversed in the R-frame.
        Position mapping is unaffected; only angle mapping is.  Recorded here so
        the angle chain is not reasoned about as if it were a pure rotation.
        """
        F = self.robot_frame.F
        det = F[0][0] * F[1][1] - F[0][1] * F[1][0]
        self.assertAlmostEqual(det, -1.0, places=9)
        s, c = math.sin(self.sandbox_frame.theta_rad), math.cos(self.sandbox_frame.theta_rad)
        self.assertAlmostEqual((-s) * s - c * c, -1.0, places=9)


class SegmentProfileParity(unittest.TestCase):
    """Single-segment velocity profile: robot `_segment_profile_time` vs
    sandbox `segment_time`.  Same formulas, same constants -> must be identical."""

    def test_identical_over_a_grid(self):
        settings = robot_settings()
        prof = _sandbox_profile("model")
        worst = 0.0
        for length in (0.0, 1.0, 5.0, 16.0, 40.0, 135.0, 200.0, 400.0):
            for v_start in (0.0, 50.0, 150.0, 300.0):
                for v_end in (0.0, 50.0, 150.0, 300.0):
                    a = RS._segment_profile_time(
                        length, v_start, v_end,
                        settings.interp_v_max, settings.interp_a_max,
                        settings.interp_d_max, settings.interp_scurve_shape_factor,
                    )
                    b = SK.segment_time(length, v_start, v_end, prof)
                    worst = max(worst, abs(a - b))
        self.assertLess(worst, 1e-9, f"segment profile diverges by {worst:.6f}s")


class CornerVelocityParity(unittest.TestCase):
    """Corner blending.  The sandbox's `corner_velocity` is the *pure geometric*
    limit; the robot's `_corner_v_end` folds the forward reachability clamp into
    the same call.  They therefore differ by construction -- what matters is
    whether the composed schedules agree (see TrajectoryScheduleParity)."""

    def test_geometric_term_matches(self):
        settings = robot_settings()
        prof = _sandbox_profile("model")
        pts = [(0.0, 0.0, -275.0), (25.0, 0.0, -260.0), (120.0, 40.0, -260.0)]
        geometric = SK.corner_velocity(pts[0], pts[1], pts[2], prof)
        # A huge entry speed makes the robot's v_reach clamp inactive, leaving
        # only the geometric term.
        composed = RS._corner_v_end(pts[0], pts[1], pts[2], 1e6, settings.interp_v_max,
                                    settings.interp_a_max)
        self.assertAlmostEqual(geometric, composed, places=9)

    def test_robot_lacks_the_stop_distance_clamp(self):
        """The robot's corner velocity never asks 'can I still stop in time?'."""
        settings = robot_settings()
        # Entry velocity is irrelevant to the sandbox's geometric term but the
        # sandbox applies a *separate* backward pass; the robot has no equivalent.
        self.assertFalse(
            hasattr(RS, "_segment_schedule"),
            "robot repo grew a segment scheduler -- re-check the backward pass",
        )
        source = RS._trajectory_total_time.__doc__ or ""
        self.assertNotIn("backward", source.lower())


class TrajectoryTemplateParity(unittest.TestCase):
    """The 7-point goto / pick waypoint templates."""

    def setUp(self) -> None:
        self.settings = robot_settings()
        self.heights = SK.Heights.from_config(sandbox_config())
        self.blend = self.settings.corner_blend_xy
        self.start = (0.0, 0.0, -290.0)
        self.pick = (60.0, -120.0, self.settings.pickup_height)
        self.sort = (149.65, -13.07, -280.0)

    def test_goto_template_identical(self):
        robot_pts = _robot_points_goto(self.settings, self.start, self.pick)
        park = (self.pick[0], self.pick[1], self.heights.pre_pick)
        sandbox_pts = SK.end_effector("goto", self.start, park, self.heights, self.blend)
        self.assertEqual(len(robot_pts), len(sandbox_pts))
        for i, (a, b) in enumerate(zip(robot_pts, sandbox_pts)):
            with self.subTest(waypoint=i):
                self.assertAlmostEqual(math.dist(a, b), 0.0, places=6)

    def test_pick_template_identical(self):
        robot_pts = _robot_points_pick(self.settings, self.pick, self.sort)
        sandbox_pts = SK.end_effector("pick", self.pick, self.sort, self.heights, self.blend)
        self.assertEqual(len(robot_pts), len(sandbox_pts))
        for i, (a, b) in enumerate(zip(robot_pts, sandbox_pts)):
            with self.subTest(waypoint=i):
                self.assertAlmostEqual(math.dist(a, b), 0.0, places=6)


class TrajectoryScheduleParity(unittest.TestCase):
    """Multi-segment timing.  Two structural divergences are expected here:

    1. the sandbox runs a *backward* pass capping every corner at the speed from
       which the arm can still stop by the next waypoint; the robot does not;
    2. the sandbox times the segment from the arm's current pose to Pos[0]; the
       robot deliberately omits it, claiming the PLC soft-start absorbs it.
    """

    def setUp(self) -> None:
        self.settings = robot_settings()
        self.prof = _sandbox_profile("model")
        self.heights = SK.Heights.from_config(sandbox_config())

    def _pair(self, start, points):
        robot_t = RS._trajectory_total_time(points, self.settings)
        sandbox_same_points = self.prof.soft_start_s + sum(
            SK.segment_schedule(list(points), self.prof)
        )
        sandbox_with_bridge = SK.trajectory_time([start, *points], self.prof)
        return robot_t, sandbox_same_points, sandbox_with_bridge

    def test_goto_phase_timing(self):
        """Forward-only scheduling is *slower on paper* than the two-pass one.

        Counter-intuitive but correct: without the backward pass the arm enters
        the short final segment far too fast, and `_segment_profile_time` then
        bills a full `v_in / d_max` deceleration that does not fit in the
        segment.  The two-pass schedule slows the corners a little and saves far
        more on the last leg.  The robot repo therefore *over*-estimates flight
        time -- a conservative bias for the gate, a pessimistic one for
        feasibility and throughput.
        """
        start = (149.65, -13.07, -280.0)          # arm resting in the QFP bin
        pick = (60.0, -120.0, self.settings.pickup_height)
        points = _robot_points_goto(self.settings, start, pick)
        robot_t, same_pts, with_bridge = self._pair(start, points)
        print(f"\n  [goto ] robot={robot_t:.4f}s  sandbox(same pts)={same_pts:.4f}s "
              f"sandbox(+bridge)={with_bridge:.4f}s  "
              f"over-estimate={robot_t - same_pts:+.4f}s")
        self.assertGreater(robot_t, same_pts)

    def test_pick_phase_timing_and_the_missing_descent(self):
        """The pick packet's Pos[0] IS the contact point, so the segment the
        robot omits is the whole park->contact descent."""
        park = (60.0, -120.0, self.settings.pre_pick_height)
        contact = (60.0, -120.0, self.settings.pickup_height)
        sort_pos = (149.65, -13.07, -280.0)
        points = _robot_points_pick(self.settings, contact, sort_pos)
        robot_t, same_pts, with_bridge = self._pair(park, points)
        print(f"  [pick ] robot={robot_t:.4f}s  sandbox(same pts)={same_pts:.4f}s "
              f"sandbox(+bridge from park)={with_bridge:.4f}s")
        print(f"  [pick ] omitted park->contact descent = "
              f"{with_bridge - same_pts:.4f}s")
        self.assertGreater(with_bridge - same_pts, 0.2,
                           "the omitted first segment is not negligible")

    def test_backward_pass_matters_most_on_the_short_final_segment(self):
        """Forward-only scheduling enters the last waypoint above the speed it
        can bleed off inside that segment."""
        park = (60.0, -120.0, self.settings.pre_pick_height)
        contact = (60.0, -120.0, self.settings.pickup_height)
        sort_pos = (149.65, -13.07, -280.0)
        points = _robot_points_pick(self.settings, contact, sort_pos)

        v = 0.0
        entry_speeds = []
        for i in range(len(points) - 1):
            a, b = points[i], points[i + 1]
            if i == len(points) - 2:
                v_end = 0.0
            else:
                v_end = RS._corner_v_end(a, b, points[i + 2], v,
                                         self.settings.interp_v_max,
                                         self.settings.interp_a_max)
            entry_speeds.append((i, math.dist(a, b), v, v_end))
            v = v_end
        last_i, last_len, v_in, _ = entry_speeds[-1]
        stop_distance = v_in ** 2 / (2.0 * self.settings.interp_d_max)
        print(f"  [last ] segment len={last_len:.1f}mm  entry v={v_in:.1f}mm/s  "
              f"stop distance needed={stop_distance:.1f}mm")
        self.assertGreater(stop_distance, last_len,
                           "expected the forward-only pass to over-run the final segment")


class ContactDelayBudget(unittest.TestCase):
    """The single number pick accuracy hangs on: how much dead time sits between
    'send the grab command' and 'the cup touches the board', and how much of it
    the pick gate actually leads by."""

    def setUp(self) -> None:
        self.settings = robot_settings()
        cfg = sandbox_config()
        self.sandbox_sched = sandbox_scheduler.Scheduler.from_config(cfg)

    def test_sandbox_budget_terms(self):
        s = self.sandbox_sched
        drop = abs(s.heights.pre_pick - s.heights.pickup)
        descent = SK.segment_time(drop, 0.0, 0.0, s.model)
        print(f"\n  [sandbox] command_delay={s.model.command_delay_s:.4f}s "
              f"soft_start={s.model.soft_start_s:.4f}s descent={descent:.4f}s "
              f"-> contact_delay={s.contact_delay_s:.4f}s")
        self.assertAlmostEqual(
            s.contact_delay_s,
            s.model.command_delay_s + s.model.soft_start_s + descent,
            places=9,
        )

    def test_robot_gate_lead_omits_soft_start_and_descent(self):
        st = self.settings
        gate_sampling = st.poll_interval_s / 2.0 + 0.0125
        gate_lead_s = st.robot_movement_delay_s + st.ethernet_delay_s + gate_sampling
        t_d = RS._descent_time_s(st, 0.0)      # soft_start + descent, belt at rest
        print(f"  [robot  ] gate lead covers {gate_lead_s:.4f}s "
              f"(move {st.robot_movement_delay_s} + eth {st.ethernet_delay_s} "
              f"+ sampling {gate_sampling:.4f})")
        print(f"  [robot  ] modelled park->contact t_d = {t_d:.4f}s "
              f"(soft_start {st.interp_soft_start_s} + descent {t_d - st.interp_soft_start_s:.4f})")
        print(f"  [robot  ] oblique_descent_enabled = {st.oblique_descent_enabled}")
        self.assertFalse(
            st.oblique_descent_enabled,
            "config changed: with the oblique descent ON, t_d is compensated by "
            "the downstream contact shift instead",
        )
        for belt in (30.0, 60.0, 100.0, 120.0, 150.0):
            uncompensated_mm = belt * t_d
            print(f"           belt {belt:6.1f} mm/s -> board travels "
                  f"{uncompensated_mm:6.2f} mm during the descent, and nothing "
                  f"in the pipeline leads by it")
        self.assertGreater(t_d, 0.3)

    def test_descend_time_is_inflated_when_the_oblique_descent_is_off(self):
        """`plan.descend_time_s` folds the belt slant in unconditionally.

        `_contact_position` computes `t_d = _descent_time_s(settings, v_belt)`
        *before* checking `oblique_descent_enabled`, so with the flag off the
        stored `descend_time_s` describes a slanted descent the arm never makes.
        That value is what the per-pick `[GATE]` line publishes as
        `t_d_model_s`, and the documented calibration is
        `robot_movement_delay_s = dispatch_to_contact_s - t_d_model_s`.
        Over-stating `t_d_model_s` therefore *under*-estimates the very latency
        the gate lead is built from.
        """
        st = self.settings
        frame = robot_conveyor.ConveyorFrame()
        pick = (60.0, -120.0, st.pickup_height)
        vertical_truth = RS._descent_time_s(st, 0.0)
        print("\n  descend_time_s reported with oblique OFF vs. the vertical "
              "descent the arm actually performs:")
        for belt in (0.0, 60.0, 100.0, 120.0):
            _, reported = RS._contact_position(frame, st, pick, belt)
            print(f"           belt {belt:6.1f} mm/s -> reported t_d="
                  f"{reported:.4f}s, true vertical t_d={vertical_truth:.4f}s, "
                  f"calibration bias={vertical_truth - reported:+.4f}s")
            if belt > 0.0:
                self.assertGreater(reported, vertical_truth)

    def test_oblique_shift_equals_the_missing_term(self):
        """Turning the oblique descent on shifts contact downstream by exactly
        the belt travel the gate does not lead by -- confirming that the flag is
        the *only* compensation for t_d in the whole pipeline."""
        st = self.settings
        frame = robot_conveyor.ConveyorFrame()
        pick = (60.0, -120.0, st.pickup_height)
        for belt in (60.0, 120.0):
            off_pos, t_d = RS._contact_position(frame, st, pick, belt)
            on_settings = RS.replace(st, oblique_descent_enabled=True)
            on_pos, t_d_on = RS._contact_position(frame, on_settings, pick, belt)
            shift = math.dist(off_pos, on_pos)
            with self.subTest(belt=belt):
                self.assertAlmostEqual(t_d, t_d_on, places=9)
                self.assertAlmostEqual(shift, belt * t_d, places=6)
                self.assertAlmostEqual(math.dist(off_pos, pick), 0.0, places=9)


class PickSolverParity(unittest.TestCase):
    """Fixed-point pick-time solvers.  Same shape; the sandbox folds the whole
    contact delay into the convergence condition, the robot folds in only the
    command delay plus one soft start."""

    def test_robot_solver_convergence_condition(self):
        st = robot_settings()
        src = RS.PickScheduler._predict_pick_position.__doc__ or ""
        self.assertIn("u_anchor", src)
        # The robot's iteration target, spelled out:
        #   t = now + command_delay + goto_total + interp_soft_start_s
        # The sandbox's:
        #   t = max(t_arrive + contact_delay, now + lead)
        #     = now + command_delay + goto + (command_delay + soft_start + descent)
        # Difference = one extra command delay + the descent.
        sandbox = sandbox_scheduler.Scheduler.from_config(sandbox_config())
        robot_extra = st.interp_soft_start_s
        sandbox_extra = sandbox.contact_delay_s
        print(f"\n  [solver] robot adds {robot_extra:.4f}s after arrival, "
              f"sandbox adds {sandbox_extra:.4f}s")
        self.assertGreater(sandbox_extra - robot_extra, 0.3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
