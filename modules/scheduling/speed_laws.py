"""Belt speed laws: which setpoint the belt should run at.

A law reads a `SpeedView` and returns a `SpeedDecision`; it never sends anything. The
controller around it (modules/scheduling/commit.py) owns the invariants every law shares: the
setpoint never moves between goto dispatch and cup contact (open-issues L9), every commit is
clamped to `speed.band` and the hardware maximum, limited to `speed.commit.max_step_mm_s` per
step, and held inside `speed.commit.deadband_mm_s`. A law only proposes a target.

*When* a law runs is its setpoint gate (gates.py): each law names a default, and
`speed.setpoint_gate` overrides it.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from modules.scheduling.jobs import picks_within
from modules.scheduling.registry import speed_law
from modules.scheduling.types import SpeedDecision, SpeedView

_GRID_EPS = 1e-9


def queue_bounds(view: SpeedView, queue_depth_k: float) -> tuple[float, float]:
    """Admissible band [v_min, v_max] of doc/basis-theory.md §7.3.

    v_max = L / (k * p_worst + g_worst): a part entering the workspace behind k queued picks
    must still be pickable when the arm reaches it. Capped by the operational ceiling;
    v_min is the operational floor."""
    u_min, u_max = view.workspace_window_uv[0], view.workspace_window_uv[1]
    band = max(0.0, u_max - u_min)
    cycle = view.arm_cycle
    denominator = queue_depth_k * cycle.occupancy_worst_s + cycle.grab_worst_s
    v_max = band / denominator if denominator > 0.0 else math.inf
    v_min, ceiling = view.band
    v_max = min(v_max, ceiling)
    return v_min, max(v_min, v_max)


def _in_workspace(view: SpeedView) -> list[float]:
    u_min, u_max = view.workspace_window_uv[0], view.workspace_window_uv[1]
    return [u for u in view.objects_u if u_min <= u <= u_max]


# ---------------------------------------------------------------------------
# Constant
# ---------------------------------------------------------------------------


@speed_law("constant", default_gate="never")
def constant(view: SpeedView) -> SpeedDecision:
    """Hold speed.static_mm_s; the setpoint never moves."""
    return SpeedDecision(view.static_mm_s)


# ---------------------------------------------------------------------------
# Inverse density (rate regulation)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class InverseDensityConfig:
    headroom: float = 0.75          # fraction of the arm's capacity 1/t_pick to present
    transit_min_s: float = 2.0      # minimum residence in the workspace: v <= L / this
    density_length_mm: float = 0.0  # length the density is measured over; <= 0 = u_max
    spacing_lead_objects: int = 4   # leading parts whose spacing caps the speed


@speed_law("inverse_density", default_gate="at_contact", config=InverseDensityConfig)
def inverse_density(view: SpeedView, cfg: InverseDensityConfig) -> SpeedDecision:
    """The denser the belt, the slower it runs: v = headroom/t_pick * L / N, spacing-capped.

    Holds the presentation rate lambda_nom = headroom / t_pick by setting the speed inversely
    to the density rho = N / L (doc/basis-theory.md §6). Three regimes emerge from the clamps:
    sparse -> the transit cap (fetch the few parts fast), regulated -> interior, dense -> the
    floor. A spacing cap is min-ed on: the tightest gap among the leading parts must take a
    whole pick cycle to arrive, so a cluster slows the belt even when N is small."""
    u_min, u_max = view.workspace_window_uv[0], view.workspace_window_uv[1]
    v_min, ceiling = view.band
    v_cap = max(v_min, min((u_max - u_min) / max(1e-6, cfg.transit_min_s), ceiling))
    t_pick = max(1e-6, view.arm_cycle.cycle_s)
    measure_mm = cfg.density_length_mm if cfg.density_length_mm > 0.0 else u_max
    n = view.n_in_window
    if n <= 0:
        v_density = v_cap
    else:
        v_density = max(v_min, min(v_cap, cfg.headroom / t_pick * measure_mm / n))

    v_spacing = math.inf
    lead = sorted(view.objects_u, reverse=True)[:cfg.spacing_lead_objects]
    if len(lead) >= 2:
        gap = min(a - b for a, b in zip(lead, lead[1:]))
        if gap > 0.0:
            v_spacing = gap / t_pick
    target = max(v_min, min(v_density, v_spacing))
    return SpeedDecision(target, {"n": n, "v_density": round(v_density, 2),
                                  "v_spacing": None if math.isinf(v_spacing) else round(v_spacing, 2)})


# ---------------------------------------------------------------------------
# Predictive rank
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PredictiveRankConfig:
    planner: str | None = None      # planner that scores candidates; None = scheduling.planner
    horizon_s: float = 45.0         # scoring horizon W: picks counted up to now + W
    queue_depth_k: float = 1.0      # picks the band must absorb: v_max = L / (k*p + g)
    candidate_step_mm_s: float = 5.0  # candidate grid step


def _score_candidates(view: SpeedView, cfg: PredictiveRankConfig):
    """Score the setpoint (the incumbent, if admissible) and every admissible grid speed as
    (v, K(V), schedule).

    A grid candidate must lie in the band, be reachable within the horizon and be one commit
    away from the setpoint, so the speed that was scored is the speed the step-limited commit
    actually sends."""
    v_min, v_max = queue_bounds(view, cfg.queue_depth_k)
    horizon_end = view.now + cfg.horizon_s
    setpoint = view.setpoint_mm_s
    incumbent = None
    if v_min <= setpoint <= v_max:
        schedule = view.schedule_for(view.forecast_for(setpoint), cfg.planner)
        incumbent = (setpoint, picks_within(schedule, horizon_end), schedule)
    candidates = []
    step = max(cfg.candidate_step_mm_s, 0.1)
    v = v_min
    while v <= v_max + _GRID_EPS:
        forecast = view.forecast_for(v)
        if view.within_step(v) and forecast.reachable_in(cfg.horizon_s):
            schedule = view.schedule_for(forecast, cfg.planner)
            candidates.append((v, picks_within(schedule, horizon_end), schedule))
        v += step
    return (v_min, v_max), incumbent, candidates


def _rank_decision(view: SpeedView, cfg: PredictiveRankConfig, hysteresis: bool) -> SpeedDecision:
    started = time.perf_counter()
    bounds, incumbent, candidates = _score_candidates(view, cfg)
    scored = ([incumbent] if incumbent is not None else []) + candidates
    if not scored:
        # Nothing admissible near the setpoint: head for the band (the step-limited commit
        # walks the belt there over several decisions).
        target, score, schedule = min(max(view.setpoint_mm_s, bounds[0]), bounds[1]), -1, None
    else:
        # Most picks, then the fastest speed; the incumbent comes first, so it wins exact ties.
        winner = max(scored, key=lambda entry: (entry[1], entry[0]))
        if hysteresis and incumbent is not None and winner[1] <= incumbent[1]:
            winner = incumbent
        target, score, schedule = winner
        if winner is incumbent:
            schedule = None          # re-plan on the live forecast rather than reuse
    return SpeedDecision(
        target,
        {"score": score, "bounds_mm_s": [round(b, 2) for b in bounds], "evaluated": len(scored),
         "decision_s": round(time.perf_counter() - started, 4)},
        schedule=schedule,
        schedule_planner=cfg.planner,
    )


@speed_law("predictive_rank", default_gate="arm_free", config=PredictiveRankConfig, deadband=False)
def predictive_rank(view: SpeedView, cfg: PredictiveRankConfig) -> SpeedDecision:
    """The speed whose schedule makes the most picks within the horizon; ties go faster.

    Candidates are enumerated, not searched: the score is integer-valued with no guarantee of
    unimodality. The setpoint is scored first and replaced only by a strictly better candidate
    or an equally good faster one (doc/basis-theory.md §7.3)."""
    return _rank_decision(view, cfg, hysteresis=False)


@speed_law("predictive_rank_hysteresis", default_gate="arm_free", config=PredictiveRankConfig,
           deadband=False)
def predictive_rank_hysteresis(view: SpeedView, cfg: PredictiveRankConfig) -> SpeedDecision:
    """predictive_rank where the setpoint yields only to a strictly better score (ablation)."""
    return _rank_decision(view, cfg, hysteresis=True)


# ---------------------------------------------------------------------------
# Bound-based laws
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class BoundConfig:
    queue_depth_k: float = 1.0


@speed_law("bound_only", default_gate="arm_free", config=BoundConfig)
def bound_only(view: SpeedView, cfg: BoundConfig) -> SpeedDecision:
    """Sit at the queue bound v_max and never look at the belt (open loop)."""
    v_min, v_max = queue_bounds(view, cfg.queue_depth_k)
    return SpeedDecision(max(v_min, v_max))


@dataclass(frozen=True)
class BacklogConfig:
    queue_depth_k: float = 1.0      # the bound the result is clamped to
    occupancy_mean_s: float = 2.0   # p: mean arm occupancy per pick (unmeasured on the cell)
    grab_mean_s: float = 1.0        # g: mean dispatch -> contact (unmeasured on the cell)


@speed_law("backlog_patience", default_gate="arm_free", config=BacklogConfig)
def backlog_patience(view: SpeedView, cfg: BacklogConfig) -> SpeedDecision:
    """Patience for the backlog actually observed: v = L / (N*p + g), N parts in the workspace."""
    v_min, v_max = queue_bounds(view, cfg.queue_depth_k)
    band = view.workspace_window_uv[1] - view.workspace_window_uv[0]
    n = len(_in_workspace(view))
    return SpeedDecision(min(v_max, max(v_min, band / (n * cfg.occupancy_mean_s + cfg.grab_mean_s))),
                         {"n": n})


@speed_law("min_slack", default_gate="arm_free", config=BacklogConfig)
def min_slack(view: SpeedView, cfg: BacklogConfig) -> SpeedDecision:
    """Slow to what the tightest part can survive: part i in line needs (u_max - u_i)/v >= i*p + g."""
    v_min, v_max = queue_bounds(view, cfg.queue_depth_k)
    u_max = view.workspace_window_uv[1]
    waiting = sorted((u for u in view.objects_u if u < u_max), reverse=True)
    limits = [(u_max - u) / (i * cfg.occupancy_mean_s + cfg.grab_mean_s) for i, u in enumerate(waiting)]
    if not limits:
        return SpeedDecision(v_max)
    return SpeedDecision(min(v_max, max(v_min, min(limits))), {"n": len(limits)})


# ---------------------------------------------------------------------------
# Load-scheduled and open-loop laws
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RateScheduleConfig:
    window_s: float = 15.0          # detections counted over this many seconds
    # (detections per minute, mm/s) points; linear in between, flat outside.
    table: tuple[tuple[float, float], ...] = ((0.0, 30.0), (60.0, 30.0))
    deadband_mm_s: float = 0.0      # keep the setpoint when the target is this close


@speed_law("rate_schedule", default_gate="arm_free", config=RateScheduleConfig)
def rate_schedule(view: SpeedView, cfg: RateScheduleConfig) -> SpeedDecision:
    """Gain scheduling: the speed a declared table gives for the load the camera measures."""
    recent = sum(1 for t in view.detection_times if t >= view.now - cfg.window_s)
    rate = 60.0 * recent / cfg.window_s if cfg.window_s > 0.0 else 0.0
    points = sorted(cfg.table)
    target = points[-1][1]
    if rate <= points[0][0]:
        target = points[0][1]
    for (r0, v0), (r1, v1) in zip(points, points[1:]):
        if r0 <= rate <= r1:
            target = v0 + (v1 - v0) * (rate - r0) / (r1 - r0) if r1 > r0 else v1
            break
    if abs(target - view.setpoint_mm_s) <= cfg.deadband_mm_s:
        target = view.setpoint_mm_s
    return SpeedDecision(target, {"rate_per_min": round(rate, 1)})


@dataclass(frozen=True)
class PeriodicTwoSpeedConfig:
    period_s: float = 20.0
    fast_share: float = 0.5         # share of each period spent fast
    fast_mm_s: float = 60.0
    slow_mm_s: float = 30.0


@speed_law("periodic_two_speed", default_gate="arm_free", config=PeriodicTwoSpeedConfig)
def periodic_two_speed(view: SpeedView, cfg: PeriodicTwoSpeedConfig) -> SpeedDecision:
    """Alternate two speeds on a fixed clock, blind to the belt (open-loop control)."""
    phase = (view.now % cfg.period_s) / cfg.period_s
    return SpeedDecision(cfg.fast_mm_s if phase < cfg.fast_share else cfg.slow_mm_s)
