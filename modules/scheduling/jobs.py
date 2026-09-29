"""From parts on the belt to scheduling jobs, and the steps every planner shares.

A planner never computes an intercept itself: `build_jobs` costs every part once from the
arm's current position (release r_j, deadline d_j, processing p_j, intercept), and a planner
that walks a sequence re-costs a part from its predecessor with `extend`. Belt speed reaches a
planner only through those numbers (doc/basis-theory.md §7.1).
"""

from __future__ import annotations

import math

from modules.scheduling.types import Job, ObjectSnapshot, PlanContext, Point, ScheduledPick


def make_job(obj: ObjectSnapshot, ctx: PlanContext, position: Point | None = None,
             start: float | None = None) -> Job:
    """Cost one part as if the arm left `position` (default: where it is) at `start`
    (default: now). Deadline and arrival come from the forecast at `ctx.now`, so they do not
    depend on `start`."""
    position = ctx.arm_position if position is None else position
    start = ctx.now if start is None else start
    u_min, u_max = ctx.workspace_window_uv[0], ctx.workspace_window_uv[1]
    deadline = ctx.now + ctx.forecast.time_to_travel(u_max - obj.u)
    arrival = ctx.now + ctx.forecast.time_to_travel(max(0.0, u_min - obj.u))
    intercept = ctx.predict(obj, position, start)
    processing = math.inf
    path_mm = math.inf
    if intercept is not None:
        processing = ctx.costs(obj, position, start, intercept)[1]
        path_mm = math.dist(position, intercept.pick_position) + math.dist(intercept.pick_position, obj.bin)
    return Job(
        obj=obj,
        now=start,
        release=obj.detected_at,
        deadline=deadline,
        processing=processing,
        feasible=intercept is not None,
        intercept=intercept,
        reachable_at=start,
        path_mm=path_mm,
        safety_margin_s=ctx.safety_margin_s,
        arrival=arrival,
    )


def build_jobs(objects: tuple[ObjectSnapshot, ...] | list[ObjectSnapshot], ctx: PlanContext) -> list[Job]:
    """One job per part, in belt order (most downstream first). A planner that sorts stably on
    a key shared by all jobs therefore sees them in release order."""
    return [make_job(obj, ctx) for obj in sorted(objects, key=lambda o: o.u, reverse=True)]


def extend(job: Job, position: Point, free_at: float, taken: frozenset[str],
           ctx: PlanContext) -> tuple[float, Point, ScheduledPick] | None:
    """Append one pick to a partial schedule: the arm leaves `position` once both it (at
    `free_at`) and the job allow. Returns (arm free again, arm position, the pick), or None
    when the part is blocked or its contact misses the margined deadline. Processing time is
    re-costed here, against the predecessor actually retained, because the arm departs from
    that part's bin."""
    if not job.blockers <= taken:
        return None
    start = max(free_at, job.reachable_at)
    intercept = ctx.predict(job.obj, position, start)
    if intercept is None or intercept.pick_time > job.deadline_safe:
        return None
    _, occupancy = ctx.costs(job.obj, position, start, intercept)
    finish = start + occupancy
    pick = ScheduledPick(job, start, intercept.pick_time, finish, occupancy, intercept)
    return finish, ctx.next_position(job.obj), pick


def precedence_order(jobs: list[Job]) -> list[Job]:
    """Pull every blocker forward to just before the job it covers (no-op without stacking)."""
    by_id = {job.obj.object_id: job for job in jobs}
    placed: set[str] = set()
    order: list[Job] = []

    def place(job: Job) -> None:
        if job.obj.object_id in placed:
            return
        placed.add(job.obj.object_id)
        for object_id in sorted(job.blockers):
            if object_id in by_id:
                place(by_id[object_id])
        order.append(job)

    for job in jobs:
        place(job)
    return order


def release_estimate(job: Job) -> float:
    """r_j = a_j - g_j with g_j flown from where the arm is now: when the arm would have to
    leave for this part. Only an ordering key; a part the arm cannot reach sorts last."""
    if job.intercept is None:
        return math.inf
    return job.arrival - (job.intercept.pick_time - job.reachable_at)


def picks_within(schedule: list[ScheduledPick], until: float) -> int:
    """How many picks of a schedule make contact by `until` — the score of a speed candidate."""
    return sum(1 for pick in schedule if pick.grab_s <= until)
