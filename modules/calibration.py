#!/usr/bin/env python3
import time
import json
import argparse
import sys
from pathlib import Path
from modules.EthernetCom import PLCGateway, COMMAND_ID

def run_calibration(samples: int, ip: str, port: int, test_mechanical: bool):
    print(f"[*] Connecting to PLC at {ip}:{port}...")
    try:
        gateway = PLCGateway(ip=ip, port=port)
        gateway.connect()
    except Exception as exc:
        print(f"[ERROR] Failed to connect to PLC: {exc}")
        sys.exit(1)

    print(f"[*] Connected. Measuring Ethernet latency with {samples} roundtrip samples...")
    rtt_times = []
    for i in range(samples):
        t0 = time.perf_counter()
        status = gateway.get_package()
        t1 = time.perf_counter()
        if status is not None:
            rtt_times.append(t1 - t0)
        time.sleep(0.01)

    if not rtt_times:
        print("[ERROR] Failed to measure latency. Readings returned empty.")
        gateway.disconnect()
        sys.exit(1)

    avg_rtt = sum(rtt_times) / len(rtt_times)
    min_rtt = min(rtt_times)
    max_rtt = max(rtt_times)
    # The ethernet delay is defined as the one-way latency (RTT / 2)
    ethernet_delay = avg_rtt / 2.0

    print("\n=== Ethernet Latency Profiling Results ===")
    print(f"  Samples: {len(rtt_times)}")
    print(f"  Min RTT: {min_rtt*1000:.3f} ms")
    print(f"  Max RTT: {max_rtt*1000:.3f} ms")
    print(f"  Avg RTT: {avg_rtt*1000:.3f} ms")
    print(f"  Calculated ethernet_delay_s (RTT / 2): {ethernet_delay:.6f} s\n")

    robot_movement_delay = None
    if test_mechanical:
        print("[*] Running mechanical response profiling...")
        print("[*] Dispatching a test command (stop command) to measure PLC response time...")
        # Send a stop command (safe, non-moving) with doing_bit=1
        package = {
            "commandID": COMMAND_ID["stop"],
            "argument_number": 0,
            "argument_x": [0.0]*4,
            "argument_y": [0.0]*4,
            "argument_z": [0.0]*4,
            "argument_e": [0]*4,
            "argument_time": [0.0]*4,
            "doing_bit": 1
        }
        t_dispatch = time.perf_counter()
        gateway.send_package(package)
        
        # Poll status until task_doing is acknowledged
        # Note: For mock/real PLC, doing_bit = 1 will cause the PLC to set task_doing = 1.
        # Let's wait up to 2 seconds
        t_ack = None
        for _ in range(200):
            status = gateway.get_package()
            if status and status.get("task_doing") == 1:
                t_ack = time.perf_counter()
                break
            time.sleep(0.01)
            
        if t_ack is not None:
            robot_movement_delay = t_ack - t_dispatch
            print(f"  PLC acknowledged command execution in: {robot_movement_delay*1000:.3f} ms")
            print(f"  Calculated robot_movement_delay_s: {robot_movement_delay:.6f} s\n")
        else:
            print("[WARN] Did not receive task_doing acknowledgment from PLC. Skipping mechanical delay calibration.")

    gateway.disconnect()

    # Save to config.json
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        try:
            with config_path.open("r", encoding="utf-8") as f:
                config_data = json.load(f)
            
            print(f"[*] Updating config file: {config_path}")
            if "scheduler" not in config_data:
                config_data["scheduler"] = {}
                
            config_data["scheduler"]["ethernet_delay_s"] = round(ethernet_delay, 6)
            if robot_movement_delay is not None:
                config_data["scheduler"]["robot_movement_delay_s"] = round(robot_movement_delay, 6)
                
            with config_path.open("w", encoding="utf-8") as f:
                json.dump(config_data, f, indent=4)
            print("[*] Calibration parameters written successfully.")
        except Exception as exc:
            print(f"[ERROR] Failed to save config: {exc}")
    else:
        print(f"[WARN] config.json not found at {config_path}. Skipping save.")

def main():
    # Load defaults from config
    from modules.EthernetCom import load_config
    config = load_config()
    
    parser = argparse.ArgumentParser(description="Delta Robot Auto-Calibration Utility")
    parser.add_argument("--ip", default=config.ip_address, help=f"PLC IP address (default: {config.ip_address})")
    parser.add_argument("--port", type=int, default=getattr(config, "port", 502), help=f"PLC port (default: {getattr(config, 'port', 502)})")
    parser.add_argument("--samples", type=int, default=50, help="Number of RTT samples (default: 50)")
    parser.add_argument("--run-mechanical", action="store_true", help="Run mechanical move delay profiling")
    args = parser.parse_args()

    run_calibration(args.samples, args.ip, args.port, args.run_mechanical)

if __name__ == "__main__":
    main()
