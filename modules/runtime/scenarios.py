"""Scenario registry: what `main.py --scheduler --scenario <name>` and the operator console run.

A scenario is a name, a description, a run function taking a `RunContext` (and the pick
executor when it moves the arm) and where its parts come from (`feed`: the camera, or the
seeded virtual feeder of `runtime/virtual_feed.py`). To add one, write the run function (see
`runtime/loop.py`) and register it in `SCENARIOS`; the CLI, the console and the dashboard pick
it up from here.

Every run keeps a part record (`runtime/outcomes.py`); with a `record_dir` it is written there
at the end (parts.csv, flow.csv, summary.json, and arrivals.csv for a virtual feed).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from modules.core.delta import DeltaArm
from modules.core.frames import ConveyorFrame
from modules.core.tracking import BeltPositionTracker, BeltTracker
from modules.runtime.cell_view import layout
from modules.runtime.loop import RunContext, run_observe_loop, run_pick_loop
from modules.runtime.outcomes import PartLedger
from modules.runtime.pick_executor import RealtimePickExecutor
from modules.runtime.planning import PickPlanner
from modules.runtime.speed_source import ConveyorSpeedSource, StaticSpeedSource
from modules.runtime.state import RealtimeState
from modules.scheduling.registry import validate_plugin_config
from modules.settings import Settings

Dispatch = Callable[[dict[str, Any]], Any]
RequestStatus = Callable[[], dict[str, Any] | None]


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    run: Callable[..., None]
    moves_arm: bool                 # needs the pick executor (and so a live PLC)
    feed: str = "camera"            # camera (or the PLC simulator's camera) | virtual


SCENARIOS: dict[str, Scenario] = {
    "production": Scenario(
        "production",
        "Pick parts from the moving belt with the configured planner and speed law.",
        run_pick_loop, moves_arm=True),
    "simulate_feeder": Scenario(
        "simulate_feeder",
        "Pick virtual parts drawn from feeder.seed / feeder.kind riding the real belt: "
        "no camera, no parts on the belt; for repeatable data runs.",
        run_pick_loop, moves_arm=True, feed="virtual"),
    "test_vision_only": Scenario(
        "test_vision_only",
        "Camera, tracking and belt feedback only: the arm stays idle, no belt command is sent.",
        run_observe_loop, moves_arm=False),
}


def get_scenario(name: str) -> Scenario:
    try:
        return SCENARIOS[name]
    except KeyError:
        raise ValueError(f"Unknown scenario '{name}'. Available: {', '.join(sorted(SCENARIOS))}") from None


def run_scenario(
    name: str,
    settings: Settings,
    *,
    dispatch: Dispatch | None = None,
    request_status: RequestStatus | None = None,
    duration_s: float | None = None,
    event_sink: Callable[[str, dict[str, Any]], None] | None = None,
    frame_register: Callable[[Any], None] | None = None,
    disable_native_window: bool = False,
    image_source: Any = None,
    stop_event: threading.Event | None = None,
    record_dir: Path | None = None,
    record_meta: dict[str, Any] | None = None,
) -> None:
    """Run one scenario to its end (duration, stop event, Ctrl-C or window close).

    `dispatch` / `request_status` are the PLC link; without them the belt is treated as
    static (camera-only runs). `image_source` replaces the camera with any object exposing
    `poll(now) -> list[ObjectDetection]` (the PLC simulator's camera under --sim); a scenario
    with a virtual feed ignores it. `event_sink` receives the structured dashboard events;
    `frame_register` is handed the image source so the dashboard can stream its frames.
    `record_dir` receives the part record at the end, `record_meta` is added to its
    summary.json."""
    scenario = get_scenario(name)
    validate_plugin_config(settings)
    if scenario.moves_arm and (dispatch is None or request_status is None):
        raise RuntimeError(f"Scenario '{name}' moves the arm and needs a live PLC link.")

    start_time = time.monotonic()
    started_iso = datetime.now().isoformat(timespec="seconds")
    frame = ConveyorFrame.from_settings(settings.conveyor)
    arm = DeltaArm.from_settings(settings, frame)
    tracker = BeltTracker(frame, workspace_window_uv=settings.conveyor.workspace_window_uv,
                          stale_timeout_s=settings.runtime.stale_timeout_s,
                          camera_window_uv=settings.conveyor.camera_window_uv)
    executor = None
    state_kwargs: dict[str, Any] = {}
    if scenario.moves_arm:
        executor = RealtimePickExecutor(dispatch, request_status, settings, arm)
        state_kwargs["ipc_lock"] = executor.ipc_lock
        request_status = executor.request_status
    if request_status is not None:
        decoder = BeltPositionTracker(velocity_ema_alpha=settings.conveyor.velocity_ema_alpha)
        speed_source: Any = ConveyorSpeedSource(request_status, frame, decoder)
    else:
        speed_source = StaticSpeedSource()

    if scenario.feed == "virtual":
        from modules.runtime.virtual_feed import VirtualFeed

        image_source = VirtualFeed(settings, start_time, speed_source.position_at, duration_s=duration_s)
    elif image_source is None:
        from modules.vision.pipeline import VisionImageProcessing

        image_source = VisionImageProcessing(settings, start_time,
                                             show_window=settings.vision.show_window and not disable_native_window)
    if frame_register is not None:
        frame_register(image_source)

    def outcome_event(payload: dict[str, Any]) -> None:
        if event_sink is not None:
            event_sink("outcome", payload)

    ledger = PartLedger(start_time, grip_tolerance_mm=settings.runtime.grip_tolerance_mm,
                        source=scenario.feed, on_close=outcome_event)
    meta = run_meta(name, settings, started_iso, duration_s, scenario.feed, image_source)
    meta.update(record_meta or {})
    if event_sink is not None:
        event_sink("layout", {**layout(settings, frame), "run": meta})

    state = RealtimeState(tracker=tracker, frame=frame, **state_kwargs)
    run = RunContext(
        scenario=name,
        settings=settings,
        state=state,
        planner=PickPlanner(settings, arm, frame, tracker, ledger),
        speed_source=speed_source,
        image_source=image_source,
        start_time=start_time,
        duration_s=duration_s,
        event_sink=event_sink,
        stop_event=stop_event,
        round_trip_s=(lambda: executor.round_trip.average_s) if executor is not None else (lambda: 0.0),
        record_dir=record_dir,
        record_meta=meta,
    )
    print(f"[INFO] Running scenario: {name} — {scenario.description}")
    print(f"[INFO] Fixed PLC slot count: {settings.plc.interpolar_points}")
    print("[INFO] Scenario will run until interrupted" if duration_s is None
          else f"[INFO] Scenario duration: {duration_s:.2f}s")
    if scenario.moves_arm:
        scenario.run(run, executor)
    else:
        scenario.run(run)


def run_meta(name: str, settings: Settings, started_iso: str, duration_s: float | None, feed: str,
             image_source: Any) -> dict[str, Any]:
    """What a run's summary.json says about its set-up: enough to group runs for a report."""
    meta: dict[str, Any] = {
        "scenario": name,
        "started": started_iso,
        "duration_requested_s": duration_s,
        "planner": settings.scheduling.planner,
        "speed_law": settings.speed.law,
        "setpoint_gate": settings.speed.setpoint_gate,
        "static_mm_s": settings.speed.static_mm_s,
        "band_mm_s": [settings.speed.band.min_mm_s, settings.speed_ceiling_mm_s()],
        "feed": feed,
    }
    if feed == "virtual":
        cfg = settings.feeder
        meta["feeder"] = {"kind": cfg.kind, "seed": cfg.seed, "max_parts": cfg.max_parts,
                          "params": dict(cfg.kinds.get(cfg.kind) or {}),
                          **image_source.describe()}
    return meta
