"""Dispatch rules: pick ONE job from the feasible ones.

The simplest plugin, and where a newcomer starts (README.md). A rule only chooses; the jobs are
already costed and filtered to the parts the arm can reach. The registry turns every rule into
a planner (it re-applies the rule to the remaining jobs to get a full ranking), so
`scheduling.planner: <rule>` works, and `<rule>_rollout` (planners.py) runs it forward in time.

On one belt at one speed, EDD and FIFO make the same choices: every deadline is
(u_max - u) / v with a shared v, and the belt order never changes. A rule that differs from
EDD has to look at something besides the deadline: path, processing time or slack.
"""

from __future__ import annotations

from modules.scheduling.registry import dispatch_rule
from modules.scheduling.types import Job, PlanContext


@dispatch_rule("spt")
def spt(jobs: list[Job], ctx: PlanContext) -> Job | None:
    """Shortest path first: the start -> pick -> bin path stands in for processing time."""
    return min(jobs, key=lambda job: job.path_mm, default=None)


@dispatch_rule("edd")
def edd(jobs: list[Job], ctx: PlanContext) -> Job | None:
    """Earliest due date: the part that leaves the workspace first."""
    return min(jobs, key=lambda job: job.deadline, default=None)


@dispatch_rule("fifo")
def fifo(jobs: list[Job], ctx: PlanContext) -> Job | None:
    """First in, first out: the part the camera saw first (a control: equals EDD here)."""
    return min(jobs, key=lambda job: job.release, default=None)


@dispatch_rule("least_slack")
def least_slack(jobs: list[Job], ctx: PlanContext) -> Job | None:
    """Least slack: deadline - now - processing, smallest first."""
    return min(jobs, key=lambda job: job.deadline - job.now - job.processing, default=None)
