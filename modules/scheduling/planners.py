"""Sequence planners: the retained jobs in pick order, with predicted times.

All solve 1 | r_j, d_j, online | sum U_j on the same jobs (doc/basis-theory.md §7). The loop
commits the first entry that still validates against the live tracker and re-plans when the
arm is free again. Belt speed reaches a planner only through the jobs' r_j, d_j and p_j.

Processing and grab times depend on the predecessor (the arm starts from the previous bin), so
every planner re-costs a job against the job actually before it (`jobs.extend`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from modules.scheduling.jobs import extend, make_job, precedence_order, release_estimate
from modules.scheduling.registry import PLANNERS, planner
from modules.scheduling.types import Job, PlanContext, Point, ScheduledPick

State = tuple[float, Point, frozenset[str], list[ScheduledPick]]   # free at, arm at, taken, picks


def _start(ctx: PlanContext) -> State:
    return ctx.now, ctx.arm_position, frozenset(), []


def _append(state: State, job: Job, ctx: PlanContext) -> State | None:
    free, position, taken, picks = state
    result = extend(job, position, free, taken, ctx)
    if result is None:
        return None
    finish, place, pick = result
    return finish, place, taken | {job.obj.object_id}, picks + [pick]


def _arrival_order(jobs: list[Job]) -> list[Job]:
    return sorted(jobs, key=lambda job: (job.arrival, job.deadline, job.obj.object_id))


def _release_order(jobs: list[Job]) -> list[Job]:
    return sorted(jobs, key=lambda job: (release_estimate(job), job.arrival, job.obj.object_id))


# ---------------------------------------------------------------------------
# Kise-Ibaraki-Mine
# ---------------------------------------------------------------------------


def _kim(order: list[Job], ctx: PlanContext) -> list[ScheduledPick]:
    """Kise, Ibaraki and Mine's algorithm for 1 | r_j | sum U_j (1978, eqs. 3-4), walked in
    the given order.

    Add the jobs one at a time. If the retained set plus the new job is on time, keep it.
    Otherwise remove exactly one job: the one whose removal leaves an on-time set that frees
    the arm earliest (ties remove the job latest in the order, the new job first). Removing
    the new job always restores the previous, on-time set, so one removal suffices.

    Exact only for the paper's problem (fixed r_j and p_j, deadline tested at completion,
    agreeable order); here p_j and g_j depend on the predecessor and the deadline is tested at
    contact, so it is a heuristic. O(n^3) transitions per decision."""
    start = _start(ctx)
    retained: list[Job] = []
    schedule = start

    for job in precedence_order(order):
        trial = retained + [job]
        prefixes = [start]                 # prefixes[i]: the first i jobs of trial
        for member in trial:
            state = _append(prefixes[-1], member, ctx)
            if state is None:
                break
            prefixes.append(state)
        if len(prefixes) == len(trial) + 1:
            retained, schedule = trial, prefixes[-1]
            continue

        best, best_h = prefixes[len(retained)], len(trial) - 1   # remove the new job
        for h in range(len(trial) - 2, -1, -1):
            state = prefixes[h]
            for member in trial[h + 1:]:
                state = _append(state, member, ctx)
                if state is None or state[0] >= best[0]:
                    state = None
                    break
            if state is not None:
                best, best_h = state, h
        retained = trial[:best_h] + trial[best_h + 1:]
        schedule = best

    return schedule[3]


@planner("kim")
def kim(jobs: list[Job], ctx: PlanContext) -> list[ScheduledPick]:
    """Kise-Ibaraki-Mine, walked in arrival order (belt order, agreeable with the deadlines)."""
    return _kim(_arrival_order(jobs), ctx)


@planner("kim_release")
def kim_release(jobs: list[Job], ctx: PlanContext) -> list[ScheduledPick]:
    """Kise-Ibaraki-Mine, walked in estimated release order r_j = a_j - g_j."""
    return _kim(_release_order(jobs), ctx)


# ---------------------------------------------------------------------------
# Lawler's cardinality recurrence
# ---------------------------------------------------------------------------


def _cardinality(order: list[Job], ctx: PlanContext) -> list[ScheduledPick]:
    """Lawler's cardinality recurrence for 1 | r_j | sum U_j (1983, eq. 2.1), walked in the
    given order: for every k keep the on-time schedule of k jobs that frees the arm earliest;
    each job extends each of them. The largest k is the schedule. A heuristic here (p_j
    depends on the predecessor), kept beside KIM for comparison. O(n^2) transitions."""
    best: list[State] = [_start(ctx)]
    for job in precedence_order(order):
        for k in range(len(best) - 1, -1, -1):      # downwards: the job is used once
            state = _append(best[k], job, ctx)
            if state is None:
                continue
            if k + 1 < len(best):
                if state[0] >= best[k + 1][0]:
                    continue                         # ties keep the incumbent
                best[k + 1] = state
            else:
                best.append(state)
    return best[-1][3]


@planner("cardinality_arrival")
def cardinality_arrival(jobs: list[Job], ctx: PlanContext) -> list[ScheduledPick]:
    """Lawler's cardinality recurrence, walked in arrival order (a heuristic, not KIM)."""
    return _cardinality(_arrival_order(jobs), ctx)


@planner("cardinality_release")
def cardinality_release(jobs: list[Job], ctx: PlanContext) -> list[ScheduledPick]:
    """Lawler's cardinality recurrence, walked in estimated release order."""
    return _cardinality(_release_order(jobs), ctx)


# ---------------------------------------------------------------------------
# Drop-longest (the robot's planner before 2026-09, kept for comparison)
# ---------------------------------------------------------------------------


def _walk(order: list[Job], ctx: PlanContext) -> tuple[bool, list[ScheduledPick]]:
    """Cost `order` back to back from the arm's position; unreachable or late entries make
    the sequence infeasible but stay in it, so the caller can choose what to drop."""
    schedule: list[ScheduledPick] = []
    free_at = ctx.now
    position = ctx.arm_position
    feasible = True
    taken: set[str] = set()

    for job in order:
        start = max(free_at, job.reachable_at)
        intercept = ctx.predict(job.obj, position, start) if job.blockers <= taken else None
        if intercept is None:
            schedule.append(ScheduledPick(job, start, math.inf, math.inf, math.inf, None))
            feasible = False
            continue
        _, processing = ctx.costs(job.obj, position, start, intercept)
        entry = ScheduledPick(job, start, intercept.pick_time, start + processing, processing, intercept)
        schedule.append(entry)
        if entry.late:
            feasible = False
        free_at = entry.finish_s
        position = ctx.next_position(job.obj)
        taken.add(job.obj.object_id)

    return feasible, schedule


@planner("drop_longest")
def drop_longest(jobs: list[Job], ctx: PlanContext) -> list[ScheduledPick]:
    """Belt order; while late, drop the longest job (the robot's `kim` before 2026-09).

    Moore-Hodgson style rather than Kise-Ibaraki-Mine: several drops per insertion can happen
    because p_j depends on the predecessor. Kept so results measured with it stay comparable."""
    order = precedence_order(sorted(jobs, key=lambda job: job.reachable_at))
    retained: list[Job] = []
    schedule: list[ScheduledPick] = []

    for job in order:
        trial = retained + [job]
        while trial:
            ok, attempt = _walk(trial, ctx)
            if ok:
                retained, schedule = trial, attempt
                break
            victim = max(attempt, key=lambda entry: entry.processing).job
            trial = [candidate for candidate in trial if candidate is not victim]

    return schedule


# ---------------------------------------------------------------------------
# Exact search
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ExactConfig:
    max_jobs: int = 8               # the most urgent parts searched; the rest are left out


def exact_schedule(jobs: list[Job], ctx: PlanContext) -> list[ScheduledPick]:
    """The largest set of parts that can all be picked, in the order that frees the arm
    earliest — exact for the given jobs. Layer k holds every (set of k parts, last part) the
    arm can reach with the earliest instant it is free there; the earlier can always wait to
    imitate the later, so only it is kept. O(2^n n^2) transitions."""
    layer: dict[tuple[frozenset[str], str | None], State] = {(frozenset(), None): _start(ctx)}
    best: list[ScheduledPick] = []
    while layer:
        following: dict[tuple[frozenset[str], str | None], State] = {}
        for (taken, _), state in layer.items():
            for job in jobs:
                object_id = job.obj.object_id
                if object_id in taken:
                    continue
                extended = _append(state, job, ctx)
                if extended is None:
                    continue
                key = (taken | {object_id}, object_id)
                if key not in following or extended[0] < following[key][0]:
                    following[key] = extended
        if following:
            best = min(following.values(), key=lambda s: s[0])[3]
        layer = following
    return best


@planner("dp", config=ExactConfig)
def dp(jobs: list[Job], ctx: PlanContext, cfg: ExactConfig) -> list[ScheduledPick]:
    """Exact search over the `max_jobs` most urgent parts (and whatever lies on them)."""
    if len(jobs) > cfg.max_jobs:
        urgent = sorted(jobs, key=lambda job: (job.deadline, job.obj.object_id))[:cfg.max_jobs]
        by_id = {job.obj.object_id: job for job in jobs}
        keep: set[str] = set()
        pending = [job.obj.object_id for job in urgent]
        while pending:                   # a blocker's own blockers go first too
            object_id = pending.pop()
            if object_id in keep or object_id not in by_id:
                continue
            keep.add(object_id)
            pending.extend(by_id[object_id].blockers)
        jobs = [job for job in jobs if job.obj.object_id in keep]
    return exact_schedule(jobs, ctx)


# ---------------------------------------------------------------------------
# Rollouts: a dispatch rule run forward in time
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RolloutConfig:
    horizon_s: float = 45.0         # stop choosing once the clock passes now + horizon


def _rollout(rule_name: str, jobs: list[Job], ctx: PlanContext, cfg: RolloutConfig) -> list[ScheduledPick]:
    """Ask the rule to choose repeatedly against a simulated clock: pick, book the arm for
    p_j, move it to the bin, re-cost what is left from there, ask again. The result has the
    shape a sequence planner returns, so a rule can be scored by a speed law like KIM. The
    predictor already waits for a part still upstream, so when nothing is on time from the
    current clock nothing will be later, and the rollout ends."""
    rule = PLANNERS[rule_name].fn
    remaining = [job.obj for job in jobs]
    schedule: list[ScheduledPick] = []
    clock = ctx.now
    position = ctx.arm_position
    end = ctx.now + cfg.horizon_s

    while remaining and clock <= end:
        snapshot = [make_job(obj, ctx, position, clock) for obj in remaining]
        ready = [job for job in snapshot if job.feasible and job.intercept.pick_time <= job.deadline_safe]
        if not ready:
            break
        chosen = rule(ready, ctx)
        if chosen is None:
            break
        schedule.append(ScheduledPick(chosen, clock, chosen.intercept.pick_time,
                                      clock + chosen.processing, chosen.processing, chosen.intercept))
        remaining = [obj for obj in remaining if obj is not chosen.obj]
        clock += chosen.processing
        position = ctx.next_position(chosen.obj)
    return schedule


def _make_rollout(rule_name: str) -> None:
    def rollout(jobs: list[Job], ctx: PlanContext, cfg: RolloutConfig) -> list[ScheduledPick]:
        return _rollout(rule_name, jobs, ctx, cfg)

    rollout.__doc__ = f"The `{rule_name}` rule rolled forward over the visible parts."
    planner(f"{rule_name}_rollout", config=RolloutConfig)(rollout)


for _rule in ("edd", "fifo", "spt", "least_slack"):
    _make_rollout(_rule)
