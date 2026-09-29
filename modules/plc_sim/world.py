"""Boards on the belt, the feeder and suction scoring for ``modules.plc_sim``.

A board is gripped when the pump is on and the cup comes down to within
``CONTACT_BAND_MM`` of ``pickup_height`` over it (closest board, within
``grip_tolerance_mm`` of the cup centre in the belt frame). It is released where the
pump turns off: near its own bin = placed, near the other bin = wrong bin, elsewhere
= dropped. A board that leaves the workspace unpicked is lost.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from modules.core.frames import ConveyorFrame
from modules.settings import Settings

CONTACT_BAND_MM = 2.0
REARM_BAND_MM = 8.0
BIN_TOLERANCE_MM = 30.0
LOST_MARGIN_MM = 30.0


@dataclass
class Board:
    board_id: str
    board_type: str
    u_anchor: float
    v: float
    belt_anchor: float
    heading_deg: float = 0.0
    state: str = "belt"  # belt | held | placed | wrong_bin | dropped | lost
    xy: tuple[float, float] | None = None  # robot XY once off the belt

    def u_at(self, belt_position: float) -> float:
        return self.u_anchor + (belt_position - self.belt_anchor)


@dataclass
class Contact:
    t: float
    cup_uv: tuple[float, float]
    board_id: str | None
    error_uv: tuple[float, float] | None  # board − cup, belt frame (u along the belt)
    gripped: bool


class World:
    def __init__(
        self,
        frame: ConveyorFrame,
        *,
        pickup_height: float,
        workspace_window_uv: tuple[float, float, float, float],
        bins: dict[str, tuple[float, float]],
        grip_tolerance_mm: float = 12.7,
    ) -> None:
        self.frame = frame
        self.pickup_height = float(pickup_height)
        self.workspace_window_uv = workspace_window_uv
        self.bins = bins  # board type -> robot XY of its bin
        self.grip_tolerance_mm = grip_tolerance_mm
        self.boards: list[Board] = []
        self.contacts: list[Contact] = []
        self.held: Board | None = None
        self._armed = True
        self._counter = 0

    @classmethod
    def from_settings(cls, settings: Settings, frame: ConveyorFrame | None = None) -> "World":
        return cls(
            frame or ConveyorFrame.from_settings(settings.conveyor),
            pickup_height=settings.robot.heights.pickup,
            workspace_window_uv=settings.conveyor.workspace_window_uv,
            bins={name: (spec.bin[0], spec.bin[1]) for name, spec in settings.object_types.items()},
        )

    def spawn(self, board_type: str, v: float, heading_deg: float, belt_position: float,
              u: float = 0.0) -> Board:
        self._counter += 1
        board = Board(f"sb{self._counter:04d}", board_type, u, v, belt_position, heading_deg)
        self.boards.append(board)
        return board

    def update(self, t: float, belt_position: float, tcp: tuple[float, float, float],
               pump_on: bool) -> None:
        x, y, z = tcp
        if z > self.pickup_height + REARM_BAND_MM:
            self._armed = True
        if pump_on and self._armed and self.held is None and z <= self.pickup_height + CONTACT_BAND_MM:
            self._armed = False
            cup_u, cup_v = self.frame.to_conveyor(x, y)
            best, best_d, best_err = None, math.inf, None
            for board in self.boards:
                if board.state != "belt":
                    continue
                err = (board.u_at(belt_position) - cup_u, board.v - cup_v)
                d = math.hypot(*err)
                if d < best_d:
                    best, best_d, best_err = board, d, err
            gripped = best is not None and best_d <= self.grip_tolerance_mm
            if gripped:
                best.state = "held"
                self.held = best
            self.contacts.append(Contact(t, (cup_u, cup_v), best.board_id if best else None,
                                         best_err, gripped))
        if self.held is not None:
            self.held.xy = (x, y)
            if not pump_on:
                self._release(self.held, (x, y))
                self.held = None
        u_max = self.workspace_window_uv[1]
        for board in self.boards:
            if board.state == "belt" and board.u_at(belt_position) > u_max + LOST_MARGIN_MM:
                board.state = "lost"

    def _release(self, board: Board, xy: tuple[float, float]) -> None:
        board.xy = xy
        nearest = min(self.bins.items(), key=lambda kv: math.dist(kv[1], xy), default=None)
        if nearest is not None and math.dist(nearest[1], xy) <= BIN_TOLERANCE_MM:
            board.state = "placed" if nearest[0] == board.board_type else "wrong_bin"
        else:
            board.state = "dropped"

    def counts(self) -> dict[str, int]:
        out = {"spawned": len(self.boards), "contacts": len(self.contacts),
               "grips": sum(1 for c in self.contacts if c.gripped)}
        for state in ("belt", "held", "placed", "wrong_bin", "dropped", "lost"):
            out[state] = sum(1 for b in self.boards if b.state == state)
        return out


@dataclass
class Feeder:
    """Puts boards on the belt at the camera line (u = 0)."""

    interval_s: float
    types: list[str]
    lanes: list[float]
    mode: str = "periodic"  # periodic | poisson
    seed: int = 1
    _next_t: float | None = None
    _index: int = 0
    _rng: random.Random = field(init=False)

    def __post_init__(self) -> None:
        self._rng = random.Random(self.seed)

    def tick(self, now: float, world: World, belt_position: float) -> None:
        if self._next_t is None:
            self._next_t = now
        if now < self._next_t or not self.types or not self.lanes:
            return
        board_type = self.types[self._index % len(self.types)]
        lane = self.lanes[self._index % len(self.lanes)]
        world.spawn(board_type, lane, self._rng.uniform(-90.0, 90.0), belt_position)
        self._index += 1
        if self.mode == "poisson":
            self._next_t = now + self._rng.expovariate(1.0 / self.interval_s)
        else:
            self._next_t = now + self.interval_s
