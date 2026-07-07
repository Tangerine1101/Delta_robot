from __future__ import annotations

import argparse
import itertools
import math
import multiprocessing as mp
from queue import Empty
from typing import Any

from modules.EthernetCom import (
    PLCGateway,
    SiemensGateway,
    load_config,
    robot_rad_to_wire_deg,
    wire_deg_to_robot_rad,
)
from modules.cli import run_interactive
from modules.interface import DashboardServer
from modules.scheduler import (
    EvaluateExecutor,
    NullExecutor,
    RealtimePickExecutor,
    SCENARIO_NAMES,
    run_scheduler_scenario,
)

_ACCURACY_SCENARIOS = ("test_accuracy", "test_acceptance")

# How often (seconds) the worker probes the PLC connection when idle, to prevent
# EtherNet/IP and snap7 sessions from being dropped by firmware keep-alive timers.
_KEEPALIVE_S = 25.0


def _worker(
    command_queue: mp.Queue,
    response_queue: mp.Queue,
    ip: str,
    port: int,
    interpolar_points: int,
) -> None:
    config = load_config()
    if ip in ("127.0.0.1", "localhost"):
        siemens_ip = ip
        siemens_port = port
    else:
        siemens_ip = getattr(config, "siemens_ip", "192.168.250.2")
        siemens_port = getattr(config, "siemens_port", 1502)

    gateway = PLCGateway(ip=ip, port=port, interpolar_points=interpolar_points)
    siemens_gateway = SiemensGateway(ip=siemens_ip, port=siemens_port)

    try:
        gateway.connect()
        siemens_gateway.connect()
    except Exception as exc:
        response_queue.put({"ok": False, "type": "connect_failed", "req_id": None, "error": str(exc)})
        return

    response_queue.put({"ok": True, "type": "connected", "req_id": None, "ip": ip, "port": port})

    try:
        while True:
            try:
                message = command_queue.get(timeout=_KEEPALIVE_S)
            except Empty:
                # Idle keepalive: probe both connections to prevent firmware session timeouts.
                try:
                    gateway._probe_connection()
                except Exception:
                    pass
                try:
                    siemens_gateway.get_status()
                except Exception:
                    pass
                continue

            req_id = message.get("req_id")
            message_type = message.get("type")

            if message_type == "shutdown":
                response_queue.put({"ok": True, "type": "shutdown", "req_id": req_id})
                break

            if message_type == "status":
                try:
                    status = gateway.get_package()
                    if status is not None:
                        try:
                            s_status = siemens_gateway.get_status()
                            if s_status is not None:
                                rotate_wire = s_status.get("rotate_current")
                                status.update({
                                    # Wire degrees [-359,359] feedback -> R-frame
                                    # DEGREES, verbatim (identity zero, no wrap so
                                    # the true PLC angle shows). Human-readable for
                                    # logs/dashboard; radians live only inside the
                                    # scheduler algorithm.
                                    "rotate_current": (
                                        math.degrees(wire_deg_to_robot_rad(rotate_wire))
                                        if rotate_wire is not None else None
                                    ),
                                    "speed_current": s_status.get("speed_current"),
                                    "siemens_task_doing": s_status.get("task_doing"),
                                    "siemens_task_state": s_status.get("task_state"),
                                    "conveyor_position": s_status.get("conveyor_position"),
                                })
                        except Exception as s_exc:
                            print(f"[WARN] Failed to query Siemens status: {s_exc}")
                    response_queue.put({"ok": True, "type": "status", "req_id": req_id, "data": status})
                except Exception as exc:
                    response_queue.put({"ok": False, "type": "error", "req_id": req_id, "error": str(exc)})
                continue

            if message_type == "send":
                try:
                    pkg = message["package"]
                    cmd_id = pkg.get("commandID")
                    if cmd_id in (7, 8, 9):
                        # Siemens command. rotate_absolute (7) carries an R-frame
                        # angle in RADIANS: convert VERBATIM to wire degrees
                        # [-359,359] (identity zero, no wrap) on the wire only
                        # (echo the original radian pkg back to the caller).
                        if cmd_id == 7:
                            wire_pkg = dict(pkg)
                            wire_pkg["rotate"] = robot_rad_to_wire_deg(
                                pkg.get("rotate", 0.0)
                            )
                        else:
                            wire_pkg = pkg
                        s_status = siemens_gateway.send_package(wire_pkg)
                        response_queue.put(
                            {
                                "ok": True,
                                "type": "sent",
                                "req_id": req_id,
                                "commandID": cmd_id,
                                "package": pkg,
                                "status": s_status,
                            }
                        )
                    else:
                        # Omron command
                        package = gateway.send_package(pkg)
                        status = gateway.get_package()
                        response_queue.put(
                            {
                                "ok": True,
                                "type": "sent",
                                "req_id": req_id,
                                "commandID": package.get("commandID"),
                                "package": package,
                                "status": status,
                            }
                        )
                except Exception as exc:
                    response_queue.put({"ok": False, "type": "error", "req_id": req_id, "error": str(exc)})
                continue

            response_queue.put(
                {
                    "ok": False,
                    "type": "error",
                    "req_id": req_id,
                    "error": f"Unknown message type: {message_type}",
                }
            )
    finally:
        gateway.disconnect()
        siemens_gateway.disconnect()


def _wait_for_response(
    response_queue: mp.Queue,
    expected_id: int | None,
    timeout: float = 5.0,
) -> dict[str, Any] | None:
    """Drain queue until we get the response with req_id == expected_id.

    Responses with a different (older) req_id are discarded with a warning.
    Returns None on timeout.
    """
    import time
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        try:
            resp = response_queue.get(timeout=remaining)
        except Empty:
            return None
        if resp.get("req_id") == expected_id:
            return resp
        # Stale response from a previous timed-out request.
        print(f"[WARN] discarding stale IPC response (req_id={resp.get('req_id')}, expected={expected_id})")


def _start_worker(
    ctx: Any,
    command_queue: mp.Queue,
    response_queue: mp.Queue,
    args: argparse.Namespace,
) -> "mp.Process | None":
    """Start the PLC worker and wait for connection confirmation.

    Returns the Process on success, None if connection failed.
    """
    worker = ctx.Process(
        target=_worker,
        args=(command_queue, response_queue, args.ip, args.port, args.interpolar_points),
        daemon=True,
    )
    worker.start()
    startup = _wait_for_response(response_queue, expected_id=None, timeout=10.0)
    if startup is None:
        print("[ERROR] PLC worker did not report readiness in time — aborting.")
        worker.terminate()
        worker.join(timeout=2.0)
        return None
    if not startup.get("ok"):
        print(f"[ERROR] Worker failed to connect: {startup.get('error')}")
        worker.join(timeout=2.0)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=2.0)
        return None
    print(f"[INFO] Worker connected to {startup.get('ip')}:{startup.get('port')}")
    return worker


def _stop_worker(worker: "mp.Process", command_queue: mp.Queue, response_queue: mp.Queue, req_counter: Any) -> None:
    req_id = next(req_counter)
    command_queue.put({"type": "shutdown", "req_id": req_id})
    _wait_for_response(response_queue, expected_id=req_id, timeout=5.0)
    worker.join(timeout=5.0)
    if worker.is_alive():
        worker.terminate()
        worker.join(timeout=5.0)


def _run_cli(args: argparse.Namespace) -> None:
    ctx = mp.get_context("spawn")
    command_queue: mp.Queue = ctx.Queue()
    response_queue: mp.Queue = ctx.Queue()
    req_counter = itertools.count(1)

    worker = _start_worker(ctx, command_queue, response_queue, args)
    if worker is None:
        return

    def dispatch(package: dict[str, Any]) -> dict[str, Any] | None:
        req_id = next(req_counter)
        command_queue.put({"type": "send", "package": package, "req_id": req_id})
        response = _wait_for_response(response_queue, expected_id=req_id, timeout=10.0)
        if response is None:
            print("[WARN] no response from PLC worker")
            return None
        if not response.get("ok", False):
            print(f"[ERROR] {response.get('error')}")
            return None
        return response.get("status")

    def request_status() -> dict[str, Any] | None:
        req_id = next(req_counter)
        command_queue.put({"type": "status", "req_id": req_id})
        response = _wait_for_response(response_queue, expected_id=req_id, timeout=10.0)
        if response is None:
            print("[WARN] no response from PLC worker")
            return None
        if not response.get("ok", False):
            print(f"[ERROR] {response.get('error')}")
            return None
        return response.get("data")

    try:
        run_interactive(
            dispatch,
            request_status,
            interpolar_points=args.interpolar_points,
            prompt=args.prompt,
        )
    finally:
        _stop_worker(worker, command_queue, response_queue, req_counter)


def _start_interface(args: argparse.Namespace) -> "tuple[DashboardServer | None, dict[str, Any]]":
    """Start the web dashboard if --interface was passed.

    Returns (server, kwargs) where kwargs are forwarded to run_scheduler_scenario
    to wire structured events + the camera MJPEG source and suppress the native
    cv2 window. When --interface is off, returns (None, {}).
    """
    if not getattr(args, "interface", False):
        return None, {}
    config = load_config()
    iface_cfg = getattr(config, "interface", {}) or {}
    port = args.interface_port if args.interface_port is not None else int(iface_cfg.get("port", 8000))
    mjpeg_fps = float(iface_cfg.get("mjpeg_fps", 15))
    server = DashboardServer(port=port, mjpeg_fps=mjpeg_fps)
    server.start()
    return server, {
        "event_sink": server.emit,
        "frame_register": server.attach_camera,
        "disable_native_window": True,
    }


def _run_scheduler(args: argparse.Namespace) -> None:
    if args.simulate_executor:
        server, iface_kwargs = _start_interface(args)
        try:
            run_scheduler_scenario(
                args.scenario,
                duration_s=args.duration,
                interpolar_points=args.interpolar_points,
                **iface_kwargs,
            )
        finally:
            if server is not None:
                server.stop()
        return

    ctx = mp.get_context("spawn")
    command_queue: mp.Queue = ctx.Queue()
    response_queue: mp.Queue = ctx.Queue()
    req_counter = itertools.count(1)

    worker = _start_worker(ctx, command_queue, response_queue, args)
    if worker is None:
        return

    def dispatch(package: dict[str, Any]) -> dict[str, Any] | None:
        req_id = next(req_counter)
        command_queue.put({"type": "send", "package": package, "req_id": req_id})
        response = _wait_for_response(response_queue, expected_id=req_id, timeout=10.0)
        if response is None:
            raise TimeoutError("no response from PLC worker while sending scheduler package")
        if not response.get("ok", False):
            raise RuntimeError(str(response.get("error")))
        return response.get("status")

    def request_status() -> dict[str, Any] | None:
        req_id = next(req_counter)
        command_queue.put({"type": "status", "req_id": req_id})
        response = _wait_for_response(response_queue, expected_id=req_id, timeout=10.0)
        if response is None:
            raise TimeoutError("no response from PLC worker while polling status")
        if not response.get("ok", False):
            raise RuntimeError(str(response.get("error")))
        return response.get("data")

    config = load_config()
    scheduler_config = getattr(config, "scheduler", {}) or {}
    wait_margin_s = float(scheduler_config.get("execution_margin_s", 0.3))
    status_poll_interval_s = float(scheduler_config.get("poll_interval_s", 0.05))
    pick_arrival_tolerance_mm = float(scheduler_config.get("pick_arrival_tolerance_mm", 5.0))
    if args.scenario == "test_vision_only":
        # Connect the full PLC (Omron + Siemens) for live belt feedback, but keep
        # the robot idle: NullExecutor reads conveyor_position and sends the belt
        # speed command without dispatching any Omron trajectory.
        executor = NullExecutor(dispatch=dispatch, request_status=request_status)
    elif args.scenario in _ACCURACY_SCENARIOS:
        # test_accuracy/test_acceptance grip static fake objects, not a moving belt
        # — no live position gate needed. EvaluateExecutor (the same real-hardware
        # backend the 'evaluate' scenario uses) dispatches each phase and waits for
        # real pos_EE convergence, and accumulates per-phase wall-clock timing.
        executor = EvaluateExecutor(
            dispatch,
            request_status,
            interpolar_points=args.interpolar_points,
            position_tolerance_mm=float(scheduler_config.get("evaluate_position_tolerance_mm", 0.01)),
            status_poll_interval_s=status_poll_interval_s,
            wait_timeout_s=float(scheduler_config.get("evaluate_wait_timeout_s", 10.0)),
            stability_window_s=float(scheduler_config.get("evaluate_stability_window_s", 0.4)),
            stability_mm=float(scheduler_config.get("evaluate_stability_mm", 0.3)),
            stability_arm_mm=float(scheduler_config.get("evaluate_stability_arm_mm", 3.0)),
        )
    else:
        executor = RealtimePickExecutor(
            dispatch,
            request_status,
            interpolar_points=args.interpolar_points,
            wait_margin_s=wait_margin_s,
            status_poll_interval_s=status_poll_interval_s,
            position_tolerance_mm=pick_arrival_tolerance_mm,
            rotate_home_tolerance_deg=float(
                scheduler_config.get("rotate_home_tolerance_deg", 0.0)
            ),
            rotate_offset_rad=math.radians(
                float(scheduler_config.get("rotate_offset_deg", 0.0))
            ),
            rotate_sign=float(scheduler_config.get("rotate_sign", 1.0)),
            rotate_refresh_max_delta_deg=float(
                scheduler_config.get("rotate_refresh_max_delta_deg", 15.0)
            ),
        )

    server, iface_kwargs = _start_interface(args)
    try:
        run_scheduler_scenario(
            args.scenario,
            duration_s=args.duration,
            interpolar_points=args.interpolar_points,
            executor=executor,
            **iface_kwargs,
        )
    finally:
        if server is not None:
            server.stop()
        if hasattr(executor, "close"):
            executor.close()
        _stop_worker(worker, command_queue, response_queue, req_counter)


def main() -> None:
    parser = argparse.ArgumentParser(description="Delta robot command line entrypoint")
    config = load_config()
    default_interpolar_points = int(getattr(config, "interpolar_points", 4))

    parser.add_argument(
        "--cli",
        action="store_true",
        help="Run the interactive CLI mode",
    )
    parser.add_argument(
        "--scheduler",
        action="store_true",
        help="Run the offline scheduler simulator/benchmark mode",
    )
    parser.add_argument("--ip", default=config.ip_address, help="PLC IP address")
    parser.add_argument("--port", type=int, default=config.port, help="PLC port")
    parser.add_argument(
        "--interpolar-points",
        type=int,
        default=default_interpolar_points,
        help="Fixed number of array elements that must match the PLC struct",
    )
    parser.add_argument("--prompt", default="robot> ", help="CLI prompt text")
    parser.add_argument(
        "--scenario",
        default="test_throughput",
        choices=sorted(SCENARIO_NAMES),
        help="Scheduler scenario name",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional scheduler runtime in seconds. Omit for continuous run.",
    )
    parser.add_argument(
        "--simulate-executor",
        action="store_true",
        help="Run scheduler without sending PickPlan trajectories to the PLC",
    )
    parser.add_argument(
        "--interface",
        action="store_true",
        help="Serve a live web dashboard (events + annotated camera MJPEG) instead "
             "of the native cv2 window. Open http://localhost:<port>. For real-camera "
             "scenarios this suppresses the native cv2 window to avoid GUI conflicts.",
    )
    parser.add_argument(
        "--interface-port",
        type=int,
        default=None,
        help="Web dashboard port (default: config.interface.port or 8000).",
    )
    args = parser.parse_args()

    if args.interpolar_points <= 0:
        parser.error("--interpolar-points must be a positive integer.")

    if args.cli == args.scheduler:
        parser.error("Choose exactly one mode: --cli or --scheduler.")

    if args.cli:
        _run_cli(args)
        return

    _run_scheduler(args)


if __name__ == "__main__":
    main()
