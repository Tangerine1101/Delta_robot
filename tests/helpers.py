"""Shared fixtures: the live config, and a toy arm with sequence-independent processing."""

from __future__ import annotations

from dataclasses import dataclass

from modules.core.arm_model import Intercept
from modules.core.forecast import steady
from modules.scheduling.jobs import build_jobs
from modules.scheduling.types import ObjectSnapshot, PlanContext
from modules.settings import Settings, load_settings, with_overrides

WINDOW = (188.0, 363.0, 0.0, 125.0)


def settings(**overrides) -> Settings:
    """The repository config with dotted-path overrides."""
    return with_overrides(load_settings(), overrides) if overrides else load_settings()


@dataclass
class ToyArm:
    """grab = start + p, finish = start + p: the textbook 1 | r_j | sum U_j instance."""

    processing: dict[str, float]
    home: tuple[float, float, float] = (0.0, 0.0, -290.0)
    gate_lead_s: float = 0.0

    def predict(self, u, v, start, start_time, forecast, anchor_time):
        object_id = self._by_u[round(u, 6)]
        t = start_time + self.processing[object_id]
        return Intercept(t, t, (0.0, 0.0, -300.0), u, 0.0)

    def costs(self, start, start_time, intercept, bin_position, belt_speed_at_pick, setup_s):
        p = intercept.pick_time - start_time
        return p, p

    def place_position(self, bin_position):
        return bin_position

    def goto_time_s(self, start, park):
        return 0.0

    def contact_delay_s(self, belt_speed):
        return 0.0

    def pick_time_s(self, park, bin_position, belt_speed):
        return 0.0


def toy_jobs(release_order: list[str], processing: dict[str, float], deadlines: dict[str, float],
             now: float = 0.0):
    """Jobs in `release_order` (belt order) with the given processing times and deadlines."""
    objects = tuple(ObjectSnapshot(name, "QFP", 350.0 - i, 50.0, (0.0, 0.0, -280.0), float(i))
                    for i, name in enumerate(release_order))
    arm = ToyArm(processing)
    arm._by_u = {round(obj.u, 6): obj.object_id for obj in objects}
    ctx = PlanContext(now, arm.home, steady(0.0), arm, WINDOW)
    jobs = build_jobs(objects, ctx)
    for job in jobs:
        job.deadline = deadlines[job.obj.object_id]
        job.arrival = float(release_order.index(job.obj.object_id))
    return jobs, ctx
