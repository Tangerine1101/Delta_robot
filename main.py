from __future__ import annotations

import argparse
import multiprocessing as mp
from queue import Empty
from typing import Any

from modules.EthernetCom import PLCGateway, load_config
from modules.cli import run_interactive
from modules.scheduler import SCENARIO_NAMES, run_scheduler_scenario


def _worker(
    command_queue: mp.Queue,
    response_queue: mp.Queue,
    ip: str,
    port: int,
    interpolar_points: int,
) -> None:
    gateway = PLCGateway(ip=ip, port=port, interpolar_points=interpolar_points)
    try:
        gateway.connect()
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
                    response_queue.put({"ok": True, "type": "status", "data": status})
                except Exception as exc:
                    response_queue.put({"ok": False, "type": "error", "error": str(exc)})
                continue

            if message_type == "send":
                try:
                    package = gateway.send_package(message["package"])
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Delta robot command line entrypoint")
    config = load_config()
    default_interpolar_points = int(getattr(config, "interpolar_points", 6))

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
    args = parser.parse_args()

    if args.interpolar_points <= 0:
        parser.error("--interpolar-points must be a positive integer.")

    if args.cli == args.scheduler:
        parser.error("Choose exactly one mode: --cli or --scheduler.")

    if args.cli:
        _run_cli(args)
        return

    run_scheduler_scenario(
        args.scenario,
        duration_s=args.duration,
        interpolar_points=args.interpolar_points,
    )


if __name__ == "__main__":
    main()
