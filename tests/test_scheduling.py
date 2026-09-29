"""Pins the scheduling framework: forecast, intercept solver, planners, speed laws, gates,
the commit policy and the registry.

Run:  python3 -m unittest tests.test_scheduling
"""

from __future__ import annotations

import itertools
import math
import random
import unittest

from modules.core.delta import DeltaArm
from modules.core.forecast import BeltForecast, live_forecast, steady
from modules.core.frames import ConveyorFrame
from modules.scheduling import gates
from modules.scheduling.commit import RESYNC_AFTER_S, CommitPolicy, commit_step
from modules.scheduling.registry import (
    PLANNERS, SPEED_LAWS, describe, get_planner, get_speed_law, validate_plugin_config,
)
from modules.scheduling.speed_laws import PredictiveRankConfig, predictive_rank, queue_bounds
from modules.scheduling.types import SpeedView
from modules.settings import ArmCycleSettings, SettingsError
from tests.helpers import settings, toy_jobs


class Forecast(unittest.TestCase):
    CASES = [BeltForecast(40.0, 40.0), BeltForecast(20.0, 80.0, 22.31), BeltForecast(90.0, 30.0, 22.31),
             BeltForecast(0.0, 50.0, 10.0, 40.0), BeltForecast(50.0, 50.0, 22.31)]

    def test_time_inverts_travel(self):
        for forecast in self.CASES:
            for dt in (0.05, 0.5, 1.7, 4.0, 12.0):
                distance = forecast.travel_in(dt)
                self.assertAlmostEqual(forecast.time_to_travel(distance), dt, places=6, msg=forecast)

    def test_stopping_belt_never_arrives(self):
        forecast = BeltForecast(30.0, 0.0, 10.0)
        self.assertTrue(math.isinf(forecast.time_to_travel(forecast.travel_in(10.0) + 1.0)))

    def test_at_composes(self):
        forecast = BeltForecast(20.0, 80.0, 22.31)
        for a, b in ((0.3, 1.0), (1.0, 3.0), (3.0, 0.5)):
            self.assertAlmostEqual(forecast.travel_in(a + b),
                                   forecast.travel_in(a) + forecast.at(a).travel_in(b), places=9)

    def test_live_forecast_settled_band(self):
        self.assertEqual(live_forecast(48.0, 50.0, 22.31, 5.0), steady(48.0))
        self.assertEqual(live_forecast(30.0, 50.0, 22.31, 5.0).target, 50.0)


def legacy_realtime_intercept(arm: DeltaArm, u: float, v: float, start, now: float, speed: float):
    """The realtime predictor the loop used before the job pipeline (frozen reference): the
    earliest reachable intercept with no lead, then pushed downstream until the arm is parked
    a full gate lead before contact."""
    gate = arm.gate
    window = arm.workspace_window_uv
    u_min, u_max, v_min, v_max = window
    command_delay_s = gate.command_delay_s
    soft_start_s = arm.robot.interpolator.soft_start_s

    def goto_total(pick):
        return arm.trajectory_time(arm.goto_points(start, pick))

    # Stage (a): earliest reachable intercept, lead 0.
    guess = now + max(0.0, command_delay_s)
    t_enter = now
    if speed > 0.001 and u < u_min:
        t_enter = now + (u_min - u) / speed
        guess = max(guess, t_enter)
    if v < v_min or v > v_max:
        return None
    for _ in range(6):
        u_pick = u + speed * max(0.0, guess - now)
        if u_pick > u_max:
            return None
        new_guess = max(now + command_delay_s + goto_total(arm.pick_point(u_pick, v)) + soft_start_s, t_enter)
        if abs(new_guess - guess) < 0.01:
            guess = new_guess
            break
        guess = new_guess
    u_pick = u + speed * max(0.0, guess - now)
    if not ConveyorFrame.is_in_window_uv(u_pick, v, window):
        return None

    # Stage (b): the caller's lead and the park-before-gate condition.
    final = max(guess, now + gate.intercept_lead_time_s)
    if u > u_max:
        return None
    for _ in range(4):
        u_pick = u + speed * max(0.0, final - now)
        clamped = False
        if u_pick > u_max:
            if speed <= 0.0:
                return None
            u_pick = u_max
            final = now + max(0.0, (u_pick - u) / speed)
            clamped = True
        if not ConveyorFrame.is_in_window_uv(u_pick, v, window):
            return None
        pick = arm.pick_point(u_pick, v)
        required = now + command_delay_s + goto_total(pick) + arm.gate_lead_s
        if required <= final + 1e-3:
            return final, final - arm.gate_lead_s, pick
        if clamped:
            return None
        final = required
    return None


class InterceptParity(unittest.TestCase):
    """DeltaArm.predict on a steady belt == the legacy realtime predictor, over a grid."""

    def test_grid(self):
        cfg = settings()
        arm = DeltaArm.from_settings(cfg)
        starts = [cfg.robot.home_position] + [arm.place_position(spec.bin) for spec in cfg.object_types.values()]
        now = 100.0
        feasible = 0
        for speed, u, v, start in itertools.product((0.0, 30.0, 100.0), (-50.0, 100.0, 188.0, 300.0, 360.0),
                                                   (-5.0, 20.0, 60.0, 110.0, 130.0), starts):
            expected = legacy_realtime_intercept(arm, u, v, start, now, speed)
            got = arm.predict(u, v, start, now, steady(speed), now)
            if expected is None:
                self.assertIsNone(got, (speed, u, v, start))
                continue
            feasible += 1
            self.assertIsNotNone(got, (speed, u, v, start))
            self.assertAlmostEqual(got.pick_time, expected[0], places=9)
            self.assertAlmostEqual(got.gate_fire_time, expected[1], places=9)
            for a, b in zip(got.pick_position, expected[2]):
                self.assertAlmostEqual(a, b, places=9)
        self.assertGreater(feasible, 20)


def best_count(order, processing, deadlines) -> int:
    best = 0
    for mask in range(1 << len(order)):
        t, ok, n = 0.0, True, 0
        for i, name in enumerate(order):
            if mask >> i & 1:
                t += processing[name]
                ok &= t <= deadlines[name]
                n += 1
        if ok:
            best = max(best, n)
    return best


def random_instance(rng: random.Random):
    n = rng.randint(1, 7)
    order = [f"j{i}" for i in range(n)]
    processing = {name: rng.uniform(0.5, 3.0) for name in order}
    deadlines, d = {}, 0.0
    for name in order:            # release order == deadline order on one belt
        d += rng.uniform(0.2, 2.5)
        deadlines[name] = d
    return order, processing, deadlines


class Planners(unittest.TestCase):
    # Exact when jobs are walked in an order agreeable with the deadlines (belt order here);
    # the release-order variants walk r_j = a_j - g_j, which need not be agreeable.
    EXACT_ON_AGREEABLE = ("kim", "drop_longest", "cardinality_arrival", "dp")

    def test_drops_one_job(self):
        order = ["a", "b", "c"]
        processing = {"a": 1.0, "b": 3.0, "c": 1.0}
        deadlines = {"a": 2.0, "b": 4.5, "c": 4.0}
        for name in ("kim", "drop_longest"):
            jobs, ctx = toy_jobs(order, processing, deadlines)
            schedule = get_planner(name)(jobs, ctx)
            self.assertEqual([p.job.obj.object_id for p in schedule], ["a", "c"], name)

    def test_optimal_on_random_agreeable_instances(self):
        rng = random.Random(7)
        for _ in range(200):
            order, processing, deadlines = random_instance(rng)
            best = best_count(order, processing, deadlines)
            for name in self.EXACT_ON_AGREEABLE:
                jobs, ctx = toy_jobs(order, processing, deadlines)
                schedule = get_planner(name)(jobs, ctx)
                self.assertTrue(all(not p.late for p in schedule), name)
                self.assertEqual(len(schedule), best, (name, order, processing, deadlines))
            for name in ("kim_release", "cardinality_release"):
                jobs, ctx = toy_jobs(order, processing, deadlines)
                schedule = get_planner(name)(jobs, ctx)
                self.assertTrue(all(not p.late for p in schedule), name)
                self.assertLessEqual(len(schedule), best, name)

    def test_rollouts_are_on_time(self):
        rng = random.Random(3)
        for _ in range(50):
            order, processing, deadlines = random_instance(rng)
            for rule in ("edd", "fifo", "spt", "least_slack"):
                jobs, ctx = toy_jobs(order, processing, deadlines)
                schedule = get_planner(f"{rule}_rollout")(jobs, ctx)
                self.assertTrue(all(not p.late for p in schedule), rule)
                ids = [p.job.obj.object_id for p in schedule]
                self.assertEqual(len(ids), len(set(ids)))

    def test_rule_becomes_full_ranking(self):
        order = ["a", "b", "c"]
        jobs, ctx = toy_jobs(order, {"a": 1.0, "b": 1.0, "c": 1.0}, {"a": 9.0, "b": 3.0, "c": 5.0})
        ranking = get_planner("edd")(jobs, ctx)
        self.assertEqual([p.job.obj.object_id for p in ranking], ["b", "c", "a"])


class Registry(unittest.TestCase):
    def test_every_sandbox_algorithm_is_registered(self):
        for name in ("spt", "edd", "fifo", "least_slack", "kim", "kim_release", "cardinality_arrival",
                     "cardinality_release", "dp", "drop_longest", "edd_rollout", "fifo_rollout",
                     "spt_rollout", "least_slack_rollout"):
            self.assertIn(name, PLANNERS)
        for name in ("constant", "inverse_density", "predictive_rank", "predictive_rank_hysteresis",
                     "bound_only", "backlog_patience", "min_slack", "rate_schedule", "periodic_two_speed"):
            self.assertIn(name, SPEED_LAWS)
        self.assertIn("predictive_rank", describe())

    def test_unknown_names_and_bad_config(self):
        with self.assertRaises(SettingsError):
            get_planner("nope")
        with self.assertRaises(SettingsError):
            get_speed_law("nope")
        with self.assertRaises(SettingsError):
            get_speed_law("constant", gate="nope")
        with self.assertRaises(SettingsError):
            get_speed_law("inverse_density", {"inverse_density": {"headrom": 1.0}})
        with self.assertRaises(SettingsError):
            get_speed_law("constant", {"constant": {"x": 1}})

    def test_repository_config_is_valid(self):
        validate_plugin_config(settings())

    def test_default_gates(self):
        self.assertFalse(get_speed_law("constant").adaptive)
        self.assertEqual(get_speed_law("inverse_density").gate.name, "at_contact")
        self.assertEqual(get_speed_law("predictive_rank").gate.name, "arm_free")
        self.assertEqual(get_speed_law("predictive_rank", gate="periodic").gate.name, "periodic")


class Gates(unittest.TestCase):
    def test_semantics(self):
        self.assertFalse(gates.never("arm_free", 99.0, 1.0))
        self.assertTrue(gates.arm_free("arm_free", 0.0, 1.0))
        self.assertFalse(gates.arm_free("idle", 0.5, 1.0))
        self.assertTrue(gates.arm_free("idle", 1.0, 1.0))
        self.assertFalse(gates.arm_free("contact", 5.0, 1.0))
        self.assertTrue(gates.at_contact("contact", 0.0, 1.0))
        self.assertFalse(gates.at_contact("busy", 5.0, 1.0))
        self.assertTrue(gates.periodic("busy", 1.0, 1.0))
        self.assertTrue(gates.always("busy", 0.0, 1.0))


class Commit(unittest.TestCase):
    POLICY = CommitPolicy(band=(30.0, 100.0), deadband_mm_s=5.0, max_step_mm_s=20.0)

    def test_step_band_deadband(self):
        self.assertEqual(commit_step(90.0, 40.0, 40.0, 0.0, self.POLICY).send, 60.0)
        self.assertEqual(commit_step(150.0, 90.0, 90.0, 0.0, self.POLICY).send, 100.0)
        self.assertEqual(commit_step(10.0, 40.0, 40.0, 0.0, self.POLICY).send, 30.0)
        self.assertIsNone(commit_step(43.0, 40.0, 40.0, 0.0, self.POLICY).send)
        self.assertEqual(commit_step(43.0, 40.0, 40.0, 0.0, self.POLICY, use_deadband=False).send, 43.0)
        self.assertIsNone(commit_step(0.0, 40.0, 40.0, 0.0, self.POLICY).send)

    def test_resync(self):
        self.assertIsNone(commit_step(40.0, 40.0, 20.0, RESYNC_AFTER_S - 0.1, self.POLICY).send)
        step = commit_step(40.0, 40.0, 20.0, RESYNC_AFTER_S, self.POLICY)
        self.assertEqual((step.send, step.resync), (40.0, True))
        self.assertIsNone(commit_step(40.0, 40.0, 35.0, 10.0, self.POLICY).send)


def speed_view(setpoint: float, score, *, objects=(), band=(10.0, 100.0)) -> SpeedView:
    return SpeedView(
        now=0.0, setpoint_mm_s=setpoint, measured_mm_s=setpoint, band=band, max_step_mm_s=20.0,
        static_mm_s=30.0, workspace_window_uv=(188.0, 363.0, 0.0, 125.0), objects_u=tuple(objects),
        arm_cycle=ArmCycleSettings(cycle_s=2.0, occupancy_worst_s=1.0, grab_worst_s=0.75), event="arm_free",
        forecast_for=lambda v: BeltForecast(setpoint, v, 22.31),
        schedule_for=lambda forecast, planner=None: score(forecast.target),
    )


class PredictiveRank(unittest.TestCase):
    CFG = PredictiveRankConfig(horizon_s=45.0, queue_depth_k=1.0, candidate_step_mm_s=5.0)

    @staticmethod
    def picks(n):
        """A fake schedule of n picks, all inside the horizon."""
        from modules.scheduling.types import ScheduledPick
        return [ScheduledPick(None, 0.0, 1.0, 1.0, 1.0) for _ in range(n)]

    def test_bounds(self):
        self.assertEqual(queue_bounds(speed_view(50.0, lambda v: []), 1.0), (10.0, 100.0))  # 175 / 1.75

    def test_candidates_within_band_and_step(self):
        seen = []

        def score(v):
            seen.append(v)
            return self.picks(int(v))

        decision = predictive_rank(speed_view(50.0, score), self.CFG)
        self.assertTrue(all(10.0 <= v <= 100.0 and abs(v - 50.0) <= 20.0 + 1e-9 for v in seen))
        self.assertEqual(decision.target_mm_s, 70.0)

    def test_setpoint_defended_and_not_reused(self):
        decision = predictive_rank(speed_view(52.0, lambda v: self.picks(5 if v == 52.0 else 3)), self.CFG)
        self.assertEqual(decision.target_mm_s, 52.0)
        self.assertIsNone(decision.schedule)

    def test_tie_goes_faster(self):
        decision = predictive_rank(speed_view(50.0, lambda v: self.picks(4)), self.CFG)
        self.assertEqual(decision.target_mm_s, 70.0)
        self.assertEqual(len(decision.schedule), 4)

    def test_inadmissible_setpoint_abandoned(self):
        decision = predictive_rank(speed_view(150.0, lambda v: self.picks(1)), self.CFG)
        self.assertLessEqual(decision.target_mm_s, 100.0)


class InverseDensity(unittest.TestCase):
    def law(self, objects):
        return get_speed_law("inverse_density", {"inverse_density": {"headroom": 0.75, "transit_min_s": 2.0}})(
            speed_view(50.0, lambda v: [], objects=objects, band=(30.0, 100.0)))

    def test_regimes(self):
        self.assertAlmostEqual(self.law([]).target_mm_s, 87.5)          # sparse: L / transit
        dense = self.law([200.0 + 10.0 * i for i in range(12)])
        self.assertEqual(dense.target_mm_s, 30.0)                        # dense and tight: floor
        spread = self.law([350.0, 250.0])
        self.assertAlmostEqual(spread.target_mm_s, min(0.375 * 363.0 / 2, 100.0 / 2.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
