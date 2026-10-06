"""Delta robot entry point.

    python3 main.py --scheduler --scenario production [--sim] [--interface]
    python3 main.py --scheduler --scenario simulate_feeder --set feeder.seed=3 [--sim]
    python3 main.py --scheduler --scenario test_vision_only [--no-plc]
    python3 main.py --cli [--dummy]
    python3 main.py --interface [--sim]          # operator console

`--set key=value` overrides one config value for this run (dotted path, e.g.
`--set speed.law=predictive_rank`); the config file is never written.
"""
from __future__ import annotations

import argparse
import time
from typing import Any

from modules.settings import Settings, SettingsError, load_settings, with_overrides


def _start_plc_sim(args: argparse.Namespace, settings: Settings, feed_interval_s: float | None = None) -> Any:
    """Start the in-process PLC simulator (modules.plc_sim) for --dummy / --sim.

    Binds an ephemeral localhost port. The worker's localhost branch routes both the Omron
    mock (pylogix MockPLC) and the Siemens mock to this one JSON-lines socket;
    `sim.address` is the (host, port) to connect to.
    """
    from modules.plc_sim.sim import PLCSim

    sim = PLCSim(
        settings=settings,
        servo_tau_s=args.sim_servo_tau,
        tag_latency_s=args.sim_tag_latency,
        feed_interval_s=feed_interval_s,
    )
    sim.start()
    host, port = sim.serve()
    print(f"[INFO] PLC simulator (Omron Matching_Code_10 + Siemens) on {host}:{port}")
    if settings.pose_stream.enabled:
        sim.start_pose_stream(settings.pose_stream.port)
    return sim


def _link(args: argparse.Namespace, settings: Settings, *, raise_errors: bool = True) -> Any:
    from modules.comm.plc_link import PlcLink

    return PlcLink(settings, args.ip, args.port, runlog=args.runlog, raise_errors=raise_errors)


def _run_cli(args: argparse.Namespace, settings: Settings) -> None:
    from modules.ui.cli import run_interactive

    sim = None
    if args.dummy:
        sim = _start_plc_sim(args, settings)
        args.ip, args.port = sim.address
    link = _link(args, settings, raise_errors=False)
    if not link.ok:
        if sim is not None:
            sim.shutdown()
        return
    try:
        run_interactive(link.dispatch, link.request_status, settings, prompt=args.prompt)
    finally:
        link.close()
        if sim is not None:
            sim.shutdown()


def _dashboard(args: argparse.Namespace, settings: Settings) -> Any:
    from modules.ui.dashboard import DashboardServer

    port = args.interface_port if args.interface_port is not None else settings.interface.port
    return DashboardServer(port=port, mjpeg_fps=settings.interface.mjpeg_fps)


def _run_scheduler(args: argparse.Namespace, settings: Settings) -> None:
    from modules.runtime.scenarios import SCENARIOS, run_scenario

    sim = None
    link = None
    server = None
    image_source = None
    virtual_feed = SCENARIOS[args.scenario].feed == "virtual"
    try:
        if args.sim:
            # A virtual feed brings its own parts: the simulator's boards would be invisible.
            sim = _start_plc_sim(args, settings, feed_interval_s=None if virtual_feed else args.sim_feed)
            args.ip, args.port = sim.address
            image_source = sim.camera()
        if not args.no_plc:
            link = _link(args, settings)
            if not link.ok:
                return
        iface: dict[str, Any] = {}
        if args.interface:
            server = _dashboard(args, settings)
            server.start()
            iface = {"event_sink": server.emit, "frame_register": server.attach_camera,
                     "disable_native_window": True}
        run_scenario(
            args.scenario, settings,
            dispatch=link.dispatch if link is not None else None,
            request_status=link.request_status if link is not None else None,
            duration_s=args.duration,
            image_source=image_source,
            record_dir=args.runlog.dir if args.runlog is not None else None,
            record_meta={"sim": bool(args.sim), "overrides": list(args.set)},
            **iface,
        )
    finally:
        if server is not None:
            server.stop()
        if link is not None:
            link.close()
        if sim is not None:
            print("[SIM]", sim.summary())
            sim.shutdown()


def _run_console(args: argparse.Namespace, settings: Settings) -> None:
    """Operator console: web UI with manual control and scenario switching.

    Starts idle — no scenario runs until the operator starts one from the page.
    """
    from modules.ui.supervisor import Supervisor

    sim = None
    if args.sim:
        sim = _start_plc_sim(args, settings, feed_interval_s=args.sim_feed)
        args.ip, args.port = sim.address
    link = _link(args, settings)
    if not link.ok:
        if sim is not None:
            sim.shutdown()
        return

    server = _dashboard(args, settings)
    sim_camera = sim.camera() if sim is not None else None
    supervisor = Supervisor(link.dispatch, link.request_status, settings, emit=server.emit,
                            attach_camera=server.attach_camera, sim=sim, sim_camera=sim_camera,
                            record_root=args.runlog.dir if args.runlog is not None else None)
    server.set_api_handler(supervisor.handle_api)
    server.start()
    if sim_camera is not None:
        server.attach_camera(sim_camera)
    supervisor.start()
    print(f"[INFO] Operator console at http://localhost:{server.port}  (Ctrl-C to quit)")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print()
    finally:
        import signal

        # A second Ctrl-C must not cut the shutdown short (belt stop, worker exit).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        supervisor.shutdown()
        server.stop()
        link.close()
        if sim is not None:
            print("[SIM]", sim.summary())
            sim.shutdown()


def _parse_overrides(pairs: list[str]) -> dict[str, Any]:
    from ruamel.yaml import YAML

    yaml = YAML(typ="safe")
    overrides: dict[str, Any] = {}
    for pair in pairs:
        key, sep, raw = pair.partition("=")
        if not sep or not key:
            raise SettingsError(f"--set expects key=value, got {pair!r}")
        overrides[key.strip()] = yaml.load(raw) if raw.strip() else None
    return overrides


def _open_runlog(args: argparse.Namespace, settings: Settings, console: bool) -> Any:
    """Open log/<timestamp>_<mode>/ unless disabled in config or with --no-log."""
    if args.no_log or not settings.logging.enabled:
        return None
    from dataclasses import asdict

    from modules.runlog import RunLog

    if console:
        mode = "console"
    elif args.cli:
        mode = "cli"
    else:
        mode = f"scheduler-{args.scenario}"
    if args.sim or args.dummy:
        mode += "-sim"
    return RunLog.open(settings.logging.dir, mode, {"plc_ip": args.ip, "pose_stream": asdict(settings.pose_stream),
                                                    "overrides": args.set})


def main() -> None:
    from modules.runtime.scenarios import SCENARIOS

    parser = argparse.ArgumentParser(description="Delta robot command line entrypoint")
    parser.add_argument("--cli", action="store_true", help="Run the interactive CLI mode")
    parser.add_argument("--scheduler", action="store_true", help="Run a scenario (see --scenario)")
    parser.add_argument("--scenario", default="production", choices=sorted(SCENARIOS),
                        help="Scenario to run with --scheduler")
    parser.add_argument("--duration", type=float, default=None,
                        help="Scenario runtime in seconds. Omit for a continuous run.")
    parser.add_argument("--ip", default=None, help="Omron PLC IP address (default: plc.omron.ip)")
    parser.add_argument("--port", type=int, default=None, help="Omron PLC port (default: plc.omron.port)")
    parser.add_argument("--prompt", default="robot> ", help="CLI prompt text")
    parser.add_argument("--dummy", action="store_true",
                        help="CLI only: connect to the in-process PLC simulator instead of hardware.")
    parser.add_argument("--no-plc", action="store_true",
                        help="Scheduler only, for scenarios that do not move the arm: run without a PLC "
                             "(static belt).")
    parser.add_argument("--interface", action="store_true",
                        help="Serve the live web dashboard instead of the native cv2 window. On its own "
                             "(no --cli/--scheduler): operator console with manual control and scenario "
                             "start/stop.")
    parser.add_argument("--interface-port", type=int, default=None,
                        help="Web dashboard port (default: interface.port).")
    parser.add_argument("--sim", action="store_true",
                        help="Scheduler / console: run against the in-process PLC simulator "
                             "(scan-accurate Omron Matching_Code_10, Siemens belt, boards, camera).")
    parser.add_argument("--sim-feed", type=float, default=2.5,
                        help="--sim: seconds between boards fed onto the belt.")
    parser.add_argument("--sim-servo-tau", type=float, default=0.0,
                        help="--sim / --dummy: first-order servo lag per axis in seconds (0 = ideal).")
    parser.add_argument("--sim-tag-latency", type=float, default=0.002,
                        help="--sim / --dummy: simulated round trip per Omron tag request in seconds.")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                        help="Override one config value for this run, e.g. --set speed.law=predictive_rank")
    parser.add_argument("--no-log", action="store_true",
                        help="Do not write a run log under logging.dir.")
    args = parser.parse_args()

    console = args.interface and not args.cli and not args.scheduler
    if not console and args.cli == args.scheduler:
        parser.error("Choose one mode: --cli, --scheduler, or --interface on its own (operator console).")
    if args.dummy and not args.cli:
        parser.error("--dummy only applies to --cli.")
    if args.sim and not (args.scheduler or console):
        parser.error("--sim applies to --scheduler and to the operator console (use --dummy with --cli).")
    if args.no_plc and (not args.scheduler or args.sim or SCENARIOS[args.scenario].moves_arm):
        parser.error("--no-plc applies to --scheduler scenarios that do not move the arm, without --sim.")

    try:
        settings = load_settings()
        if args.set:
            settings = with_overrides(settings, _parse_overrides(args.set))
    except SettingsError as exc:
        parser.error(str(exc))
    args.ip = args.ip or settings.plc.omron.ip
    args.port = args.port or settings.plc.omron.port

    args.runlog = _open_runlog(args, settings, console)
    try:
        if console:
            _run_console(args, settings)
        elif args.cli:
            _run_cli(args, settings)
        else:
            _run_scheduler(args, settings)
    finally:
        if args.runlog is not None:
            args.runlog.close()


if __name__ == "__main__":
    main()
