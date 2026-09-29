"""Operator console back-end: one owner of the PLC link, manual control vs scenario runs.

Used by ``main.py --interface`` run on its own (no ``--cli`` / ``--scheduler``). The web
dashboard (``modules/ui/dashboard.py``) forwards every ``/api/*`` request to
:meth:`Supervisor.handle_api`.

Modes: ``idle`` → ``manual`` (while a manual command is being sent) → ``idle``, or
``idle`` → ``running`` (a scenario thread) → ``stopping`` → ``idle``. Manual commands are
refused while a scenario runs and while the arm is still moving. Two properties of the
deployed Omron program make the second rule necessary (``doc/plc/version-diff-and-defects.md``
§3a): a command 3 sent on top of a running trajectory makes the setpoint jump (O2), and a
command 2 aborted by another motion command leaves every later command 2 ignored (O6).
Every manual target is checked against the PLC's own IK first (O1).

A scenario start may carry run settings (planner, speed law, constant speed, and the virtual
feeder's kind, seed, rate and part count); they apply to that run only, through
`settings.with_overrides`. Each run's part record goes to `<record_root>/run<NN>_<scenario>/`.
"""
from __future__ import annotations

import math
import threading
import time
import traceback
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from modules.comm.packets import COMMAND_ID, RobotPacket
from modules.core.feeders import FEEDERS, RATE_FIELDS
from modules.core.frames import ConveyorFrame
from modules.core.kinematics import calc_inverse_kinematics
from modules.runtime.cell_view import arm_geometry, layout
from modules.runtime.scenarios import SCENARIOS, run_scenario
from modules.scheduling.registry import catalogue, validate_plugin_config
from modules.settings import Settings, with_overrides

IDLE, MANUAL, RUNNING, STOPPING = "idle", "manual", "running", "stopping"
JOG_MAX_MM = 20.0
POLL_PERIOD_S = 0.2
STATIONARY_WINDOW_S = 0.4
STATIONARY_MM = 0.3


class Busy(RuntimeError):
    """The request is valid but cannot run in the current state (HTTP 409)."""


class Supervisor:
    def __init__(
        self,
        dispatch: Callable[[dict[str, Any]], Any],
        request_status: Callable[[], dict[str, Any] | None],
        settings: Settings,
        *,
        emit: Callable[[str, dict[str, Any]], None],
        attach_camera: Callable[[Any], None] | None = None,
        sim: Any = None,
        sim_camera: Any = None,
        record_root: Path | None = None,
    ) -> None:
        self._dispatch_raw = dispatch
        self._status_raw = request_status
        self._ipc = threading.RLock()
        self._lock = threading.RLock()
        self.settings = settings
        self.emit = emit
        self.attach_camera = attach_camera
        self.n = settings.plc.interpolar_points
        self.sim = sim
        self.sim_camera = sim_camera
        self.record_root = record_root
        self.frame = ConveyorFrame.from_settings(settings.conveyor)
        self.plugins = catalogue()
        self.run_count = 0
        self.run_settings: dict[str, Any] = {}

        self.workspace = settings.robot.limits
        self.limits = {
            "z_min": self.workspace.z_min_mm,
            "z_max": self.workspace.z_max_mm,
            "r_max": self.workspace.radius_xy_mm,
            "belt_max": settings.conveyor.hw_max_mm_s,
            "jog_max": JOG_MAX_MM,
        }
        clearance = settings.robot.heights.clearance
        self.presets: dict[str, tuple[float, float, float]] = {"home": settings.robot.home_position}
        for name, spec in settings.object_types.items():
            self.presets[f"bin {name}"] = (spec.bin[0], spec.bin[1], clearance)

        self.mode = IDLE
        self.scenario: str | None = None
        self.run_started: float | None = None
        self.run_plans = 0
        self._run_thread: threading.Thread | None = None
        self._stop_event: threading.Event | None = None
        self.pump_cmd = 0
        self.belt_cmd = 0.0
        self.last_status: dict[str, Any] | None = None
        self.link_ok = False
        self._poses: deque[tuple[float, tuple[float, float, float]]] = deque(maxlen=50)
        self._busy_until = 0.0
        self.log: deque[dict[str, Any]] = deque(maxlen=300)
        self._shutdown = threading.Event()
        self._poll_thread: threading.Thread | None = None

    # ---- IPC (serialised: the worker's request/response queue is not re-entrant) --------
    def dispatch(self, package: dict[str, Any]) -> Any:
        with self._ipc:
            return self._dispatch_raw(package)

    def request_status(self) -> dict[str, Any] | None:
        with self._ipc:
            return self._status_raw()

    # ---- lifecycle ----------------------------------------------------------------------
    def start(self) -> None:
        self._poll_thread = threading.Thread(target=self._poll_loop, name="supervisor-poll",
                                             daemon=True)
        self._poll_thread.start()
        self.emit("layout", {**layout(self.settings, self.frame), "run": None})
        self._log("info", "operator console ready" + (" (PLC simulator)" if self.sim else ""))

    def shutdown(self) -> None:
        self.stop_scenario(reason="console shutdown")
        if self._run_thread is not None:
            self._run_thread.join(timeout=30.0)
        self._shutdown.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2.0)

    def _poll_loop(self) -> None:
        last_slow = 0.0
        while not self._shutdown.is_set():
            if self.mode != RUNNING:
                self._refresh_status()
            now = time.monotonic()
            if now - last_slow >= 1.0:
                last_slow = now
                self.emit("sup", self.state())
                if self.sim is not None:
                    self.emit("sim", self.sim.snapshot())
            time.sleep(POLL_PERIOD_S)

    def _refresh_status(self) -> dict[str, Any] | None:
        try:
            status = self.request_status()
        except Exception as exc:  # link down: keep polling, report it
            status = None
            if self.link_ok:
                self._log("error", f"PLC status failed: {exc}")
        self.link_ok = status is not None
        if status is None:
            return None
        self.last_status = status
        pose = status.get("pos_EE")
        if isinstance(pose, (list, tuple)) and len(pose) >= 3:
            p = (float(pose[0]), float(pose[1]), float(pose[2]))
            self._poses.append((time.monotonic(), p))
            payload = {"x": round(p[0], 2), "y": round(p[1], 2), "z": round(p[2], 2),
                       "e": self.pump_cmd, "arm": arm_geometry(self.frame, p)}
        else:
            payload = {}
        payload.update({
            "scenario": self.scenario or "console",
            "speed_mm_s": status.get("speed_current"),
            "position_mm": status.get("conveyor_position"),
            "rotate_deg": status.get("rotate_current"),
            "task_doing": status.get("task_doing"),
            "task_state": status.get("task_state"),
        })
        self.emit("status", payload)
        return status

    # ---- state ----------------------------------------------------------------------------
    def arm_idle(self) -> tuple[bool, str]:
        status = self.last_status
        if status is None:
            return False, "no PLC status yet"
        doing, state = status.get("task_doing"), status.get("task_state")
        if doing == COMMAND_ID["goto_absolute"] and state == 2:
            return False, "a goto is still running (a second goto would be ignored: O6)"
        if doing == COMMAND_ID["calibrate"] and state == 2:
            return False, "homing in progress"
        now = time.monotonic()
        if now < self._busy_until:
            return False, "trajectory just sent"
        recent = [p for t, p in self._poses if t >= now - STATIONARY_WINDOW_S]
        if len(recent) < 2:
            return False, "waiting for fresh pose samples"
        if max(math.dist(recent[0], p) for p in recent) > STATIONARY_MM:
            return False, "arm is moving"
        return True, "idle"

    def state(self) -> dict[str, Any]:
        if self.mode in (RUNNING, STOPPING):
            idle, why = False, "scenario owns the arm"
            self.pump_cmd = None  # the trajectories' E values drive the pump now
        else:
            idle, why = self.arm_idle()
        return {
            "mode": self.mode,
            "scenario": self.scenario,
            "elapsed_s": round(time.monotonic() - self.run_started, 1) if self.run_started else None,
            "plans": self.run_plans,
            "sim": self.sim is not None,
            "link_ok": self.link_ok,
            "arm_idle": idle,
            "arm_note": why,
            "pump_cmd": self.pump_cmd,
            "belt_cmd": self.belt_cmd,
            "scenarios": sorted(SCENARIOS),
            "scenario_feeds": {name: sc.feed for name, sc in SCENARIOS.items()},
            "presets": {k: [round(c, 2) for c in v] for k, v in self.presets.items()},
            "limits": self.limits,
            "plugins": self.plugins,
            "feeders": sorted(FEEDERS),
            "run_defaults": self.run_defaults(),
            "run_settings": self.run_settings,
        }

    def run_defaults(self) -> dict[str, Any]:
        """The run settings the start form begins with: the loaded config."""
        s = self.settings
        rates = {}
        for kind, field_name in RATE_FIELDS.items():
            params = s.feeder.kinds.get(kind) or {}
            if field_name is not None and field_name in params:
                rates[kind] = params[field_name]
        return {
            "planner": s.scheduling.planner,
            "speed_law": s.speed.law,
            "static_mm_s": s.speed.static_mm_s,
            "feeder_kind": s.feeder.kind,
            "feeder_seed": s.feeder.seed,
            "feeder_max_parts": s.feeder.max_parts,
            "feeder_rates": rates,
            "feeder_rate_fields": RATE_FIELDS,
        }

    def run_overrides(self, body: dict[str, Any]) -> dict[str, Any]:
        """Dotted config overrides from the start form; blank fields keep the config value."""
        def given(key: str) -> bool:
            return body.get(key) not in (None, "")

        overrides: dict[str, Any] = {}
        if given("planner"):
            overrides["scheduling.planner"] = str(body["planner"])
        if given("speed_law"):
            overrides["speed.law"] = str(body["speed_law"])
        if given("static_mm_s"):
            overrides["speed.static_mm_s"] = float(body["static_mm_s"])
        kind = str(body["feeder_kind"]) if given("feeder_kind") else self.settings.feeder.kind
        if given("feeder_kind"):
            if kind not in FEEDERS:
                raise ValueError(f"unknown feeder '{kind}'")
            overrides["feeder.kind"] = kind
        if given("feeder_seed"):
            overrides["feeder.seed"] = int(body["feeder_seed"])
        if given("feeder_max_parts"):
            overrides["feeder.max_parts"] = int(body["feeder_max_parts"])
        if given("feeder_rate"):
            field_name = RATE_FIELDS.get(kind)
            if field_name is None:
                raise ValueError(f"feeder '{kind}' has no single rate; set its parameters in config.yaml")
            rate = float(body["feeder_rate"])
            if rate <= 0.0:
                raise ValueError("feeder rate must be positive")
            overrides[f"feeder.kinds.{kind}.{field_name}"] = rate
        return overrides

    def _log(self, level: str, text: str) -> None:
        entry = {"t": time.time(), "level": level, "text": text}
        self.log.append(entry)
        self.emit("log", entry)
        print(f"[CONSOLE] {level.upper()} {text}", flush=True)

    # ---- API ------------------------------------------------------------------------------
    def handle_api(self, method: str, path: str, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        routes: dict[tuple[str, str], Callable[[dict[str, Any]], dict[str, Any]]] = {
            ("GET", "/api/state"): lambda b: self.state(),
            ("GET", "/api/log"): lambda b: {"log": list(self.log)},
            ("POST", "/api/manual/goto"): self._api_goto,
            ("POST", "/api/manual/jog"): self._api_jog,
            ("POST", "/api/manual/preset"): self._api_preset,
            ("POST", "/api/manual/pump"): self._api_pump,
            ("POST", "/api/manual/home"): self._api_home,
            ("POST", "/api/belt"): self._api_belt,
            ("POST", "/api/rotate"): self._api_rotate,
            ("POST", "/api/scenario/start"): self._api_start,
            ("POST", "/api/scenario/stop"): lambda b: self.stop_scenario(reason="operator"),
            ("POST", "/api/stop"): self._api_emergency_stop,
        }
        handler = routes.get((method, path.split("?", 1)[0]))
        if handler is None:
            return 404, {"ok": False, "error": f"no route {method} {path}"}
        try:
            result = handler(body or {})
            return 200, {"ok": True, **result}
        except Busy as exc:
            return 409, {"ok": False, "error": str(exc)}
        except (ValueError, KeyError, TypeError) as exc:
            return 400, {"ok": False, "error": str(exc)}
        except Exception as exc:
            self._log("error", f"{path} failed: {exc}")
            return 500, {"ok": False, "error": str(exc)}

    # -- manual
    def _manual(self, label: str, action: Callable[[], dict[str, Any]], *, need_idle_arm: bool = True
                ) -> dict[str, Any]:
        with self._lock:
            if self.mode in (RUNNING, STOPPING):
                raise Busy(f"scenario '{self.scenario}' is running — stop it first")
            if self.mode == MANUAL:
                raise Busy("another manual command is being sent")
            if need_idle_arm:
                self._refresh_status()
                idle, why = self.arm_idle()
                if not idle:
                    raise Busy(f"arm not ready: {why}")
            self.mode = MANUAL
        try:
            result = action()
            self._log("info", label)
            return result
        finally:
            with self._lock:
                self.mode = IDLE
            self.emit("sup", self.state())

    def check_target(self, x: float, y: float, z: float) -> None:
        problem = self.workspace.violation(x, y, z)
        if problem is not None:
            raise ValueError(problem)
        ret, _, limit = calc_inverse_kinematics(x, y, z)
        if ret:
            raise ValueError("no IK solution for this point")
        if limit:
            raise ValueError("point violates the PLC's −20° joint limit (the PLC would not report it: O1)")

    def _goto(self, x: float, y: float, z: float) -> dict[str, Any]:
        self.check_target(x, y, z)
        packet = RobotPacket(commandID=COMMAND_ID["goto_absolute"], argument_number=1,
                             argument_x=[x], argument_y=[y], argument_z=[z],
                             argument_e=[0], argument_time=[0.0]).to_dict(self.n)
        self.dispatch(packet)
        # task_state turns 2 only on the PLC's next scan and the pose needs a few polls to
        # show motion; until then the arm must not look idle (a second goto: O6).
        self._busy_until = time.monotonic() + 0.5
        return {"target": [round(x, 2), round(y, 2), round(z, 2)]}

    def _current_pose(self) -> tuple[float, float, float]:
        if not self._poses:
            raise Busy("no pose from the PLC yet")
        return self._poses[-1][1]

    def _api_goto(self, body: dict[str, Any]) -> dict[str, Any]:
        x, y, z = float(body["x"]), float(body["y"]), float(body["z"])
        self.check_target(x, y, z)
        return self._manual(f"goto ({x:.1f}, {y:.1f}, {z:.1f})", lambda: self._goto(x, y, z))

    def _api_jog(self, body: dict[str, Any]) -> dict[str, Any]:
        axis = str(body["axis"]).lower()
        step = float(body["step"])
        if axis not in ("x", "y", "z"):
            raise ValueError("axis must be x, y or z")
        if abs(step) > JOG_MAX_MM or step == 0.0:
            raise ValueError(f"jog step must be non-zero and at most {JOG_MAX_MM:.0f} mm")

        def act() -> dict[str, Any]:
            x, y, z = self._current_pose()
            x += step if axis == "x" else 0.0
            y += step if axis == "y" else 0.0
            z += step if axis == "z" else 0.0
            return self._goto(x, y, z)

        return self._manual(f"jog {axis} {step:+.1f} mm", act)

    def _api_preset(self, body: dict[str, Any]) -> dict[str, Any]:
        name = str(body["name"])
        if name not in self.presets:
            raise ValueError(f"unknown preset '{name}'")
        x, y, z = self.presets[name]
        return self._manual(f"goto preset '{name}'", lambda: self._goto(x, y, z))

    def _api_pump(self, body: dict[str, Any]) -> dict[str, Any]:
        on = 1 if bool(body.get("on")) else 0

        def act() -> dict[str, Any]:
            # Seven copies of the current pose: segment 0 has zero length, so the chain
            # stops there by design (O3) with the pump holding E[0] — the only
            # pump-in-place command the deployed program has (program-old.md rung 19).
            p = self._current_pose()
            packet = RobotPacket(commandID=COMMAND_ID["go_trajectory"], argument_number=self.n,
                                 argument_x=[p[0]] * self.n, argument_y=[p[1]] * self.n,
                                 argument_z=[p[2]] * self.n, argument_e=[on] * self.n,
                                 argument_time=[0.1] * self.n).to_dict(self.n)
            self.dispatch(packet)
            self.pump_cmd = on
            self._busy_until = time.monotonic() + 0.3
            return {"pump": on}

        return self._manual(f"pump {'ON' if on else 'OFF'}", act)

    def _api_home(self, body: dict[str, Any]) -> dict[str, Any]:
        if not body.get("confirm"):
            raise ValueError("homing moves every axis to its switch: send confirm=true")

        def act() -> dict[str, Any]:
            packet = RobotPacket(commandID=COMMAND_ID["calibrate"], argument_number=0).to_dict(self.n)
            self.dispatch(packet)
            self._busy_until = time.monotonic() + 0.5
            return {"homing": True}

        return self._manual("homing (command 4)", act)

    def _siemens(self, label: str, package: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            if self.mode in (RUNNING, STOPPING):
                raise Busy(f"scenario '{self.scenario}' owns the belt and the cup — stop it first")
        self.dispatch(package)
        self._log("info", label)
        return {}

    def _api_belt(self, body: dict[str, Any]) -> dict[str, Any]:
        speed = float(body.get("speed", 0.0))
        if not 0.0 <= speed <= self.limits["belt_max"]:
            raise ValueError(f"belt speed must be in [0, {self.limits['belt_max']:.0f}] mm/s")
        self.belt_cmd = speed
        return self._siemens(f"belt speed {speed:.0f} mm/s", {
            "commandID": COMMAND_ID["change_speed"], "CommandID": COMMAND_ID["change_speed"],
            "rotate": 0.0, "speed": speed})

    def _api_rotate(self, body: dict[str, Any]) -> dict[str, Any]:
        deg = float(body["deg"])
        if abs(deg) > 359.0:
            raise ValueError("rotation must be within ±359°")
        return self._siemens(f"cup rotation {deg:.1f}°", {
            "commandID": COMMAND_ID["rotate_absolute"], "CommandID": COMMAND_ID["rotate_absolute"],
            "rotate": math.radians(deg), "speed": 0.0})

    # -- scenarios
    def _api_start(self, body: dict[str, Any]) -> dict[str, Any]:
        name = str(body["name"])
        if name not in SCENARIOS:
            raise ValueError(f"unknown scenario '{name}'")
        duration = body.get("duration")
        duration_s = float(duration) if duration not in (None, "", 0, "0") else None
        overrides = self.run_overrides(body.get("settings") or {})
        run_settings = with_overrides(self.settings, overrides) if overrides else self.settings
        validate_plugin_config(run_settings)
        with self._lock:
            if self.mode != IDLE:
                raise Busy(f"console is {self.mode}")
            if not self.link_ok:
                raise Busy("no PLC link")
            self._refresh_status()
            idle, why = self.arm_idle()
            if not idle:
                raise Busy(f"arm not ready: {why}")
            self.mode = RUNNING
            self.scenario = name
            self.run_started = time.monotonic()
            self.run_plans = 0
            self.run_count += 1
            self.run_settings = overrides
            self._stop_event = threading.Event()
            self._run_thread = threading.Thread(
                target=self._run, args=(name, duration_s, self._stop_event, run_settings, overrides),
                name=f"scenario-{name}", daemon=True)
            self._run_thread.start()
        described = ", ".join(f"{k}={v}" for k, v in overrides.items())
        self._log("info", f"scenario '{name}' started" + (f" for {duration_s:.0f}s" if duration_s else "")
                  + (f" ({described})" if described else ""))
        self.emit("sup", self.state())
        return {"scenario": name}

    def _run(self, name: str, duration_s: float | None, stop_event: threading.Event,
             settings: Settings, overrides: dict[str, Any]) -> None:
        virtual = SCENARIOS[name].feed == "virtual"
        paused_feeder = None
        if virtual and self.sim is not None:
            # The simulator's own boards would be picked blindly: pause them for this run.
            with self.sim.lock:
                paused_feeder, self.sim.feeder = self.sim.feeder, None
        record_dir = None
        if self.record_root is not None:
            stamp = datetime.now().strftime("%H%M%S")
            record_dir = self.record_root / f"run{self.run_count:02d}_{stamp}_{name}"
        try:
            def sink(kind: str, payload: dict[str, Any]) -> None:
                if kind == "plan":
                    self.run_plans += 1
                self.emit(kind, payload)

            run_scenario(
                name, settings, dispatch=self.dispatch, request_status=self.request_status,
                duration_s=duration_s, event_sink=sink, frame_register=self.attach_camera,
                disable_native_window=True, image_source=None if virtual else self.sim_camera,
                stop_event=stop_event, record_dir=record_dir,
                record_meta={"sim": self.sim is not None, "overrides": overrides, "console_run": self.run_count},
            )
            self._log("info", f"scenario '{name}' ended ({self.run_plans} plans)"
                      + (f"; record in {record_dir}" if record_dir is not None else ""))
        except Exception as exc:
            self._log("error", f"scenario '{name}' crashed: {exc}")
            traceback.print_exc()
        finally:
            if paused_feeder is not None:
                with self.sim.lock:
                    self.sim.feeder = paused_feeder
            try:  # the belt keeps its last commanded speed otherwise
                self.dispatch({"commandID": COMMAND_ID["change_speed"],
                               "CommandID": COMMAND_ID["change_speed"], "rotate": 0.0, "speed": 0.0})
                self.belt_cmd = 0.0
            except Exception as exc:
                self._log("error", f"belt stop after run failed: {exc}")
            if self.attach_camera is not None:
                self.attach_camera(self.sim_camera)
            with self._lock:
                self.mode = IDLE
                self.scenario = None
                self.run_started = None
            self.emit("sup", self.state())

    def stop_scenario(self, *, reason: str) -> dict[str, Any]:
        with self._lock:
            if self.mode != RUNNING or self._stop_event is None:
                return {"stopping": False}
            self.mode = STOPPING
            self._stop_event.set()
        self._log("warn", f"stopping scenario '{self.scenario}' ({reason}); the pick in flight finishes first")
        self.emit("sup", self.state())
        return {"stopping": True}

    def _api_emergency_stop(self, body: dict[str, Any]) -> dict[str, Any]:
        self.stop_scenario(reason="STOP button")
        try:
            self.dispatch({"commandID": COMMAND_ID["change_speed"], "CommandID": COMMAND_ID["change_speed"],
                           "rotate": 0.0, "speed": 0.0})
            self.belt_cmd = 0.0
        except Exception as exc:
            self._log("error", f"belt stop failed: {exc}")
        self._log("warn", "STOP: belt stopped. An arm motion already sent to the Omron PLC cannot "
                          "be aborted (no stop command on the PLC: P10) — use the hardware E-stop.")
        return {"belt_stopped": True}
