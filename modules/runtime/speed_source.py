"""Where the belt position and speed come from."""

from __future__ import annotations

import time
from typing import Any, Callable

from modules.core.frames import ConveyorFrame
from modules.core.tracking import BeltPositionTracker
from modules.runtime.state import SpeedSample


# conveyor_state values the belt cannot move in (plc_package, Section_Conveyor).
_CONVEYOR_FAULT_STATES = {3: "an axis error", 4: "servo OFF"}


class ConveyorSpeedSource:
    """Belt position and speed from the Omron belt feedback, `conveyor_position` and
    `speed_current` (true mm and mm/s: the PLC worker has already applied
    `conveyor.position_scale_mm`)."""

    def __init__(
        self,
        request_status: Callable[[], dict[str, Any] | None],
        frame: ConveyorFrame,
        decoder: BeltPositionTracker,
    ) -> None:
        self.request_status = request_status
        self.frame = frame
        self.decoder = decoder
        self._conveyor_state: int | None = None
        # Latest raw PLC status (pose, end effector, belt and rotation feedback), cached so the
        # perception tick reads the robot pose without a second PLC round trip.
        self.last_status: dict[str, Any] | None = None

    def sample(self, now: float) -> SpeedSample:
        # Stamp the reading at the midpoint of the blocking PLC round trip, not at the
        # caller's pre-request `now`: the PLC latched conveyor_position during the exchange,
        # and a pre-request stamp makes position_at() backdating read an early belt position
        # (pick-accuracy-findings T8).
        t_request = time.monotonic()
        try:
            status = self.request_status()
        except Exception as exc:
            print(f"[WARN] ConveyorSpeedSource failed to read status: {exc}")
            status = None
        now = max(now, (t_request + time.monotonic()) / 2.0)
        self.last_status = status

        if status is not None:
            self._note_conveyor_state(status.get("conveyor_state"))
            conveyor_position = status.get("conveyor_position")
            if conveyor_position is not None:
                velocity = status.get("speed_current")
                self.decoder.update(float(conveyor_position), now,
                                    None if velocity is None else float(velocity))

        scalar = self.decoder.velocity_mm_per_s
        vx, vy = self.frame.velocity_to_robot(scalar)
        return SpeedSample(vx=vx, vy=vy, timestamp=now, position_mm=self.decoder.position_mm,
                           speed_uv=scalar)

    def position_at(self, t: float) -> float | None:
        """Belt position at a past capture time (camera-latency compensation); None when the
        history is too stale, so the caller falls back to the current position."""
        return self.decoder.position_at(t)

    def _note_conveyor_state(self, state: Any) -> None:
        if state is None:
            return
        state = int(state)
        if state in _CONVEYOR_FAULT_STATES and state != self._conveyor_state:
            print(f"[WARN] belt servo reports {_CONVEYOR_FAULT_STATES[state]} (conveyor_state={state}): "
                  "speed commands have no effect until it is cleared on the PLC")
        elif self._conveyor_state in _CONVEYOR_FAULT_STATES and state not in _CONVEYOR_FAULT_STATES:
            print(f"[INFO] belt servo recovered (conveyor_state={state})")
        self._conveyor_state = state


class StaticSpeedSource:
    """A belt that never moves: camera-only runs without a PLC (`--no-plc`). The camera is
    then the only position source, so detections stay where they are seen."""

    last_status: dict[str, Any] | None = None

    def sample(self, now: float) -> SpeedSample:
        return SpeedSample(vx=0.0, vy=0.0, timestamp=now, position_mm=0.0, speed_uv=0.0)

    def position_at(self, t: float) -> float | None:
        return None
