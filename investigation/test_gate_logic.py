"""Unit-level probes of the pick gate and the intercept predictor.

These do not simulate anything.  Each one drives a single shipped function with
a hand-built state and asserts what it does at a boundary the production loop
can reach.  They are the deterministic counterpart to
`investigation.experiments`, which measures the same effects end to end.

Run:  python3 -m investigation.test_gate_logic
"""

from __future__ import annotations

import math
import unittest

import modules.scheduler as RS
from modules.conveyor import BeltTracker, ConveyorFrame, TrackedObject
from modules.EthernetCom import load_config


def _settings(**overrides):
    settings = RS.SchedulerSettings.from_config(load_config())
    return RS.replace(settings, **overrides) if overrides else settings


def _state(settings, frame, tracker, belt_position, belt_speed) -> RS.RealtimeState:
    return RS.RealtimeState(
        tracker=tracker,
        frame=frame,
        belt_position_mm=belt_position,
        belt_speed_mm_s=belt_speed,
        command_delay_s=settings.robot_movement_delay_s + settings.ethernet_delay_s,
        gate_sampling_latency_s=settings.poll_interval_s / 2.0 + 0.0125,
    )


def _plan_at(frame, settings, u_pick, v_pick, object_id="obj-1") -> RS.PickPlan:
    x, y = frame.to_robot(u_pick, v_pick)
    return RS.PickPlan(
        plan_id="plan-test",
        object_id=object_id,
        object_type="TQFP",
        detected_at=0.0,
        source_position_2d=(u_pick, v_pick),
        cycle_start_position=settings.home_position,
        assumed_speed=(0.0, 0.0),
        predicted_pick_time=0.0,
        pick_dispatch_time=0.0,
        predicted_pick_position_2d=(x, y, settings.pickup_height),
        sorting_position=(149.65, -13.07, -280.0),
        trajectory_goto=[],
        trajectory_pick=[],
    )


def _tracked(object_id, u, v, belt_position) -> TrackedObject:
    return TrackedObject(
        object_id=object_id,
        object_type="TQFP",
        conveyor_uv=(u, v),
        belt_pos_anchor=belt_position,
    )


class GateThresholdShape(unittest.TestCase):
    """`_object_pick_gate_status` -- what the gate does and does not check."""

    def setUp(self) -> None:
        self.settings = _settings()
        self.frame = ConveyorFrame()
        self.tracker = BeltTracker(self.frame,
                                   workspace_window_uv=self.settings.workspace_window_uv,
                                   camera_window_uv=self.settings.camera_window_uv)
        self.u_pick = 300.0
        self.v_pick = 60.0
        self.plan = _plan_at(self.frame, self.settings, self.u_pick, self.v_pick)

    def _gate(self, u_object, belt_speed):
        self.tracker._objects.clear()
        obj = _tracked("obj-1", u_object, self.v_pick, 0.0)
        self.tracker._objects[obj.object_id] = obj
        state = _state(self.settings, self.frame, self.tracker, 0.0, belt_speed)
        return RS._object_pick_gate_status(state, self.plan)

    def test_lead_is_linear_in_belt_speed(self):
        st = self.settings
        expected_lead_s = (st.robot_movement_delay_s + st.ethernet_delay_s
                           + st.poll_interval_s / 2.0 + 0.0125)
        print(f"\n  gate lead budget = {expected_lead_s:.4f}s")
        for belt in (0.0, 60.0, 120.0, 200.0):
            gate = self._gate(0.0, belt)
            lead_mm = gate["pick_u"] - gate["threshold_u"]
            print(f"    belt {belt:6.1f} mm/s -> fires {lead_mm:6.2f} mm early")
            self.assertAlmostEqual(lead_mm, belt * expected_lead_s, places=6)

    def test_gate_is_one_sided_and_has_no_late_bound(self):
        """The only question the gate asks is 'has it got here yet?'.

        There is no upper bound: an object that is already far downstream of the
        park point -- because the arm arrived late, or the goto flight was
        under-modelled -- reads `reached = True` just the same, and the executor
        dispatches the grab into empty belt.  Nothing between the gate and the
        `pick` dispatch re-checks how far past the park point the board is.
        """
        for overshoot in (0.0, 20.0, 60.0, 200.0):
            gate = self._gate(self.u_pick + overshoot, 120.0)
            with self.subTest(overshoot=overshoot):
                self.assertTrue(gate["reached"])
        # ...and past u_max too, which is outside the reachable window entirely.
        u_max = self.settings.workspace_window_uv[1]
        gate = self._gate(u_max + 50.0, 120.0)
        self.assertTrue(gate["reached"],
                        "an object 50 mm past u_max still opens the gate")

    def test_gate_uses_the_smoothed_belt_speed(self):
        """The lead is `v * T`, and `v` is the EMA-filtered encoder derivative.

        During a commanded ramp the filtered value lags the true speed, so the
        lead is wrong by `(v_true - v_filtered) * T`.  `basis-theory.md` 6.5
        answers this by promising the belt is steady whenever a gate fires;
        `execute()` then commits a new setpoint at the grip instant, which is
        inside the descent the lead does not cover.
        """
        st = self.settings
        lead_s = (st.robot_movement_delay_s + st.ethernet_delay_s
                  + st.poll_interval_s / 2.0 + 0.0125)
        # A 20 mm/s step (belt_speed_max_step_mm_s) mis-estimated during the ramp.
        error_mm = st.belt_speed_max_step_mm_s * lead_s
        print(f"  a {st.belt_speed_max_step_mm_s:.0f} mm/s speed-estimate error "
              f"moves the gate by {error_mm:.2f} mm")
        self.assertGreater(error_mm, 4.0)


class InterceptFeasibility(unittest.TestCase):
    """`_predict_realtime_pick_position` -- how much margin a plan is given."""

    def setUp(self) -> None:
        self.settings = _settings()
        self.frame = ConveyorFrame()
        self.tracker = BeltTracker(self.frame,
                                   workspace_window_uv=self.settings.workspace_window_uv,
                                   camera_window_uv=self.settings.camera_window_uv)
        self.scheduler = RS.PickScheduler(self.settings, 7, self.frame, self.tracker,
                                          "production")
        self.scheduler.current_position = (149.65, -13.07, -280.0)

    def _predict(self, u_now, belt_speed, now=1000.0):
        obj = _tracked("obj-1", u_now, 60.0, 0.0)
        sample = RS.SpeedSample(vx=0.0, vy=0.0, timestamp=now,
                                position_mm=0.0, speed_uv=belt_speed)
        self.scheduler.latest_speed = sample
        return RS._predict_realtime_pick_position(self.scheduler, obj, sample, now)

    def test_accepted_plans_have_zero_time_margin(self):
        """The feasibility test is `arm_arrival <= pick_time`, with no margin.

        Every millisecond by which the real flight exceeds the modelled one is
        therefore a millisecond the board spends travelling past the park point
        before the arm is there -- and the gate, being one-sided, fires anyway.
        The sandbox names the same lever `pick_margin_mm`, and records that it
        too defaults to 0.
        """
        st = self.settings
        command_delay = st.robot_movement_delay_s + st.ethernet_delay_s
        rows = []
        for belt in (60.0, 90.0, 120.0, 150.0):
            for u_now in (150.0, 200.0, 260.0, 300.0):
                prediction = self._predict(u_now, belt)
                if prediction is None:
                    rows.append((belt, u_now, None, None))
                    continue
                pick_time, _, pick_position = prediction
                goto = RS._build_goto_geometry(self.scheduler.current_position,
                                               pick_position, st)
                arrival = 1000.0 + command_delay + RS._trajectory_total_time(goto, st)
                margin = pick_time - arrival
                u_pick = self.frame.to_conveyor(pick_position[0], pick_position[1])[0]
                rows.append((belt, u_now, margin, u_pick))
        print("\n   belt   u_now   park margin (s)   u_pick")
        for belt, u_now, margin, u_pick in rows:
            if margin is None:
                print(f"  {belt:5.0f}  {u_now:6.0f}   rejected")
            else:
                print(f"  {belt:5.0f}  {u_now:6.0f}   {margin:+13.4f}   {u_pick:6.1f}")
        margins = [m for _, _, m, _ in rows if m is not None]
        self.assertTrue(margins, "no plan was accepted at all")
        self.assertLess(min(margins), 0.15,
                        "expected at least one plan parked with almost no spare time")

    def test_intercept_lead_time_only_binds_when_the_flight_is_short(self):
        """`intercept_lead_time_s = 0.8` is the promised parked-and-waiting time.

        It is a `max()` against the arrival-driven solution, so it binds only
        when the whole flight fits inside 0.8 s.  Compare the sandbox's
        `known-issues.md` 2, which measured the same lever never binding.
        """
        st = self.settings
        command_delay = st.robot_movement_delay_s + st.ethernet_delay_s
        pick = (60.0, -120.0, st.pickup_height)
        goto = RS._build_goto_geometry(self.scheduler.current_position, pick, st)
        flight = RS._trajectory_total_time(goto, st)
        print(f"\n  modelled goto flight from the bin = {flight:.4f}s, "
              f"+ command delay {command_delay:.4f}s = "
              f"{flight + command_delay:.4f}s vs lead {st.intercept_lead_time_s:.2f}s")
        self.assertGreater(flight + command_delay, st.intercept_lead_time_s,
                           "the minimum lead would bind -- re-read this test")

    def test_the_plannable_window_collapses_with_belt_speed(self):
        """How far upstream must a board still be for a plan to be accepted?

        `_predict_pick_position` returns None as soon as the projected intercept
        passes `u_max`, so the u_max clamp inside `_predict_realtime_pick_position`
        is nearly unreachable: an object that would need clamping is rejected one
        call earlier.  What is left is a hard cut-off that marches upstream as the
        belt speeds up.  Past it the loop has nothing to plan, and the boards it
        does plan are the ones it barely caught.
        """
        st = self.settings
        u_min, u_max, _, _ = st.workspace_window_uv
        print(f"\n  workspace = [{u_min:.0f}, {u_max:.0f}]; camera sees "
              f"[{st.camera_window_uv[0]:.0f}, {st.camera_window_uv[1]:.0f}]")
        print("   belt   latest u_now still plannable   headroom past u_min")
        cutoffs = {}
        for belt in (60.0, 90.0, 120.0, 150.0, 180.0):
            latest = None
            for u_now in range(0, int(u_max) + 1, 2):
                if self._predict(float(u_now), belt) is not None:
                    latest = float(u_now)
            cutoffs[belt] = latest
            if latest is None:
                print(f"  {belt:5.0f}   {'nothing is plannable':>30}")
            else:
                print(f"  {belt:5.0f}   {latest:>30.0f}   {latest - u_min:+.0f} mm")
        finite = [v for v in cutoffs.values() if v is not None]
        self.assertTrue(finite)
        # The cut-off must be monotone in belt speed: a faster belt leaves less
        # time to fly, so the board has to be caught further upstream.
        ordered = [cutoffs[b] for b in sorted(cutoffs) if cutoffs[b] is not None]
        self.assertEqual(ordered, sorted(ordered, reverse=True))


class AdaptiveSpeedLaw(unittest.TestCase):
    """The deployed belt-speed law, checked against its own configuration."""

    def setUp(self) -> None:
        self.settings = _settings()

    def test_static_seed_sits_above_the_adaptive_ceiling(self):
        """Open issue G1, confirmed numerically: startup commands a speed the
        controller is not allowed to ask for."""
        st = self.settings
        v_cap = RS._adaptive_belt_speed(0, st)
        print(f"\n  startup seed {st.belt_speed_static_mm_s:.0f} mm/s vs "
              f"adaptive ceiling {v_cap:.1f} mm/s")
        self.assertGreater(st.belt_speed_static_mm_s, v_cap)

    def test_density_law_is_hyperbolic_and_steps_hard_at_small_n(self):
        st = self.settings
        print("   N  ->  v_target")
        previous = None
        for n in range(0, 7):
            v = RS._adaptive_belt_speed(n, st)
            jump = "" if previous is None else f"   (step {v - previous:+.1f})"
            print(f"  {n:2d}  ->  {v:7.1f} mm/s{jump}")
            previous = v
        v0 = RS._adaptive_belt_speed(0, st)
        v1 = RS._adaptive_belt_speed(1, st)
        v2 = RS._adaptive_belt_speed(2, st)
        # Bang-bang at small N: one commit step is 20 mm/s, so the empty->one
        # transition alone needs three commits to walk.
        self.assertGreater(abs(v0 - v1), st.belt_speed_max_step_mm_s)
        # And from two objects on, the law is saturated at the floor: with
        # lambda_nom = headroom / pick_cycle_s over L_meas = u_max,
        # the interior of the band is only ever reached at N = 1.
        self.assertAlmostEqual(v2, st.belt_speed_min_mm_s, places=6)
        self.assertAlmostEqual(RS._adaptive_belt_speed(20, st), st.belt_speed_min_mm_s,
                               places=6)
        print(f"  the regulated interior of the law is reached only at N = 1; "
              f"N >= 2 pins v_min = {st.belt_speed_min_mm_s:.0f} mm/s")

    def test_density_region_starts_upstream_of_the_workspace(self):
        """Open issue G3: `belt_density_length_mm = 0` derives L_meas = u_max,
        which begins at the C-frame origin, not at `u_min`."""
        st = self.settings
        u_min, u_max, _, _ = st.workspace_window_uv
        l_meas = st.belt_density_length_mm if st.belt_density_length_mm > 0 else u_max
        print(f"  regulated window = [{u_min:.0f}, {u_max:.0f}] "
              f"({u_max - u_min:.0f} mm); density measured over "
              f"[0, {l_meas:.0f}] ({l_meas:.0f} mm)")
        self.assertGreater(l_meas, u_max - u_min)


class ContactPointConstruction(unittest.TestCase):
    """Where the pick packet's Pos[0] ends up, and what the PLC does with it."""

    def test_pick_packet_pos0_is_the_contact_point(self):
        st = self.settings = _settings()
        contact = (60.0, -120.0, st.pickup_height)
        points = RS._build_pick_geometry(contact, (149.65, -13.07, -280.0), st, [])
        self.assertAlmostEqual(math.dist(points[0], contact), 0.0, places=9)
        self.assertAlmostEqual(points[0][2], st.pickup_height, places=9)

    def test_the_park_to_contact_move_is_not_a_packet_segment(self):
        """The arm is parked at `pre_pick_height` when the pick packet is sent,
        but the packet's first *segment* is Pos[0] -> Pos[1], i.e. the lift back
        up.  The descent onto the board is whatever the PLC does to get from the
        measured pose to Pos[0] -- State 10, an 80 ms linear setpoint ramp
        (`doc/PLC_Program_description/MC_inter_curve_vel.md`).  The repo's own
        `_descent_time_s` instead models it as a full S-curve segment.
        """
        st = _settings()
        drop = abs(st.pre_pick_height - st.pickup_height)
        s_curve = RS._segment_profile_time(
            drop, 0.0, 0.0, st.interp_v_max, st.interp_a_max, st.interp_d_max,
            st.interp_scurve_shape_factor)
        print(f"\n  park->contact drop = {drop:.1f} mm")
        print(f"    modelled by _descent_time_s as an S-curve segment: {s_curve:.4f}s "
              f"(+{st.interp_soft_start_s:.2f}s soft start)")
        print(f"    performed by the PLC as State 10             : "
              f"{st.interp_soft_start_s:.4f}s linear setpoint ramp")
        print(f"    implied mean cup speed during State 10       : "
              f"{drop / st.interp_soft_start_s:.0f} mm/s, from and to a full stop")
        self.assertGreater(s_curve, st.interp_soft_start_s)


if __name__ == "__main__":
    unittest.main(verbosity=2)
