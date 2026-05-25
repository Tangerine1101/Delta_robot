from __future__ import annotations

import argparse
import multiprocessing as mp
from queue import Empty
from typing import Any

from modules.EthernetCom import PLCGateway, SiemensGateway, load_config
from modules.cli import run_interactive
from modules.scheduler import RealRobotExecutor, SCENARIO_NAMES, run_scheduler_scenario


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
        response_queue.put(
            {
                "ok": True,
                "type": "connected",
                "ip": ip,
                "port": port,
            }
        )

        while True:
            message = command_queue.get()
            message_type = message.get("type")

            if message_type == "shutdown":
                response_queue.put({"ok": True, "type": "shutdown"})
                break

            if message_type == "status":
                try:
                    status = gateway.get_package()
                    if status is not None:
                        try:
                            s_status = siemens_gateway.get_status()
                            if s_status is not None:
                                status.update({
                                    "rotate_current": s_status.get("rotate_current"),
                                    "speed_current": s_status.get("speed_current"),
                                    "siemens_task_doing": s_status.get("task_doing"),
                                    "siemens_task_state": s_status.get("task_state"),
                                })
                        except Exception as s_exc:
                            print(f"[WARN] Failed to query Siemens status: {s_exc}")
                    response_queue.put({"ok": True, "type": "status", "data": status})
                except Exception as exc:
                    response_queue.put({"ok": False, "type": "error", "error": str(exc)})
                continue

            if message_type == "send":
                try:
                    pkg = message["package"]
                    cmd_id = pkg.get("commandID")
                    if cmd_id in (7, 8, 9):
                        # Siemens command
                        s_status = siemens_gateway.send_package(pkg)
                        response_queue.put(
                            {
                                "ok": True,
                                "type": "sent",
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
                                "commandID": package.get("commandID"),
                                "package": package,
                                "status": status,
                            }
                        )
                except Exception as exc:
                    response_queue.put({"ok": False, "type": "error", "error": str(exc)})
                continue

            response_queue.put(
                {
                    "ok": False,
                    "type": "error",
                    "error": f"Unknown message type: {message_type}",
                }
            )
    finally:
        gateway.disconnect()
        siemens_gateway.disconnect()


def _wait_for_response(response_queue: mp.Queue, timeout: float = 5.0) -> dict[str, Any] | None:
    try:
        return response_queue.get(timeout=timeout)
    except Empty:
        return None


def _run_cli(args: argparse.Namespace) -> None:
    ctx = mp.get_context("spawn")
    command_queue: mp.Queue = ctx.Queue()
    response_queue: mp.Queue = ctx.Queue()
    worker = ctx.Process(
        target=_worker,
        args=(
            command_queue,
            response_queue,
            args.ip,
            args.port,
            args.interpolar_points,
        ),
        daemon=True,
    )
    worker.start()

    startup = _wait_for_response(response_queue, timeout=10.0)
    if startup is None:
        print("[WARN] PLC worker did not report readiness in time")
    elif startup.get("ok"):
        print(f"[INFO] Worker connected to {startup.get('ip')}:{startup.get('port')}")
    else:
        print(f"[ERROR] Worker failed to start: {startup.get('error')}")
        command_queue.put({"type": "shutdown"})
        worker.join(timeout=2.0)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=2.0)
        return

    def dispatch(package: dict[str, Any]) -> dict[str, Any] | None:
        command_queue.put({"type": "send", "package": package})
        response = _wait_for_response(response_queue, timeout=10.0)
        if response is None:
            print("[WARN] no response from PLC worker")
            return None
        if not response.get("ok", False):
            print(f"[ERROR] {response.get('error')}")
            return None
        return response.get("status")

    def request_status() -> dict[str, Any] | None:
        command_queue.put({"type": "status"})
        response = _wait_for_response(response_queue, timeout=10.0)
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
        command_queue.put({"type": "shutdown"})
        _wait_for_response(response_queue, timeout=5.0)
        worker.join(timeout=5.0)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=5.0)


def _run_scheduler(args: argparse.Namespace) -> None:
    if args.simulate_executor:
        run_scheduler_scenario(
            args.scenario,
            duration_s=args.duration,
            interpolar_points=args.interpolar_points,
        )
        return

    ctx = mp.get_context("spawn")
    command_queue: mp.Queue = ctx.Queue()
    response_queue: mp.Queue = ctx.Queue()
    worker = ctx.Process(
        target=_worker,
        args=(
            command_queue,
            response_queue,
            args.ip,
            args.port,
            args.interpolar_points,
        ),
        daemon=True,
    )
    worker.start()

    startup = _wait_for_response(response_queue, timeout=10.0)
    if startup is None:
        print("[WARN] PLC worker did not report readiness in time")
    elif startup.get("ok"):
        print(f"[INFO] Worker connected to {startup.get('ip')}:{startup.get('port')}")
    else:
        print(f"[ERROR] Worker failed to start: {startup.get('error')}")
        command_queue.put({"type": "shutdown"})
        worker.join(timeout=2.0)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=2.0)
        return

    def dispatch(package: dict[str, Any]) -> dict[str, Any] | None:
        command_queue.put({"type": "send", "package": package})
        response = _wait_for_response(response_queue, timeout=10.0)
        if response is None:
            raise TimeoutError("no response from PLC worker while sending scheduler package")
        if not response.get("ok", False):
            raise RuntimeError(str(response.get("error")))
        return response.get("status")

    def request_status() -> dict[str, Any] | None:
        command_queue.put({"type": "status"})
        response = _wait_for_response(response_queue, timeout=10.0)
        if response is None:
            raise TimeoutError("no response from PLC worker while polling status")
        if not response.get("ok", False):
            raise RuntimeError(str(response.get("error")))
        return response.get("data")

    config = load_config()
    scheduler_config = getattr(config, "scheduler", {}) or {}
    wait_margin_s = float(scheduler_config.get("execution_margin_s", 0.3))
    status_poll_interval_s = float(scheduler_config.get("poll_interval_s", 0.05))
    executor = RealRobotExecutor(
        dispatch,
        request_status,
        interpolar_points=args.interpolar_points,
        wait_margin_s=wait_margin_s,
        status_poll_interval_s=status_poll_interval_s,
    )

    try:
        run_scheduler_scenario(
            args.scenario,
            duration_s=args.duration,
            interpolar_points=args.interpolar_points,
            executor=executor,
        )
    finally:
        command_queue.put({"type": "shutdown"})
        _wait_for_response(response_queue, timeout=5.0)
        worker.join(timeout=5.0)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=5.0)


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
