"""Simple console telemetry monitor using pymavlink.

This script connects to a MAVLink target over a serial COM port and prints
basic telemetry such as flight mode, armed state, position, and battery data.
"""

import argparse
import datetime
import os
import sys
from typing import Optional

PROJECT_BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if PROJECT_BASE not in sys.path:
    sys.path.insert(0, PROJECT_BASE)

from pymavlink import mavutil


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Connect to a MAVLink target and print basic telemetry.",
    )
    parser.add_argument(
        "port",
        help="Serial port or device path (e.g. COM3, /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=57600,
        help="Baud rate for the serial connection (default: 57600)",
    )
    parser.add_argument(
        "--source-system",
        type=int,
        default=255,
        help="Source system ID for messages we send (default: 255)",
    )
    parser.add_argument(
        "--source-component",
        type=int,
        default=190,
        help="Source component ID for messages we send (default: 190)",
    )
    parser.add_argument(
        "--heartbeat-timeout",
        type=int,
        default=30,
        help="Seconds to wait for initial heartbeat before giving up.",
    )
    return parser.parse_args()


def describe_mode(master: mavutil.mavfile, msg) -> str:
    mode = "unknown"
    if msg.get_type() == "HEARTBEAT":
        base_mode = msg.base_mode
        custom_mode = msg.custom_mode
        mode_mapping = master.mode_mapping()
        if mode_mapping:
            mode = mode_mapping.get(custom_mode, str(custom_mode))
        else:
            mode = str(custom_mode)
        armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return f"{mode} ({'ARMED' if armed else 'DISARMED'})"
    return mode


def format_position(msg) -> str:
    if msg is None:
        return "lat: ---, lon: ---, alt: ---"
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.relative_alt / 1000.0
    return f"lat: {lat:.6f}, lon: {lon:.6f}, rel alt: {alt:.1f} m"


def format_battery(msg) -> str:
    if msg is None:
        return "voltage: ---, current: ---, remaining: ---%"
    voltage = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else None
    current = msg.current_battery / 100.0 if msg.current_battery != 65535 else None
    remaining = msg.battery_remaining if msg.battery_remaining != -1 else None
    parts = []
    parts.append(f"voltage: {voltage:.2f} V" if voltage is not None else "voltage: ---")
    parts.append(f"current: {current:.2f} A" if current is not None else "current: ---")
    parts.append(
        f"remaining: {remaining}%" if remaining is not None else "remaining: ---%"
    )
    return ", ".join(parts)


def main() -> None:
    args = parse_args()

    print(f"Connecting to {args.port} at {args.baud} baud...")
    master = mavutil.mavlink_connection(
        args.port,
        baud=args.baud,
        source_system=args.source_system,
        source_component=args.source_component,
    )

    try:
        master.wait_heartbeat(timeout=args.heartbeat_timeout)
    except mavutil.mavlink.MAVError:
        raise SystemExit(
            f"No heartbeat received within {args.heartbeat_timeout} seconds."
        )

    print(
        f"Connected to system {master.target_system} component {master.target_component}."
    )
    print("Press Ctrl+C to stop.\n")

    last_heartbeat: Optional[object] = None
    last_gps: Optional[object] = None
    last_sys_status: Optional[object] = None
    last_vfr: Optional[object] = None

    try:
        while True:
            msg = master.recv_match(
                type=[
                    "HEARTBEAT",
                    "GLOBAL_POSITION_INT",
                    "SYS_STATUS",
                    "VFR_HUD",
                ],
                blocking=True,
            )
            if msg is None:
                continue

            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                last_heartbeat = msg
            elif mtype == "GLOBAL_POSITION_INT":
                last_gps = msg
            elif mtype == "SYS_STATUS":
                last_sys_status = msg
            elif mtype == "VFR_HUD":
                last_vfr = msg

            timestamp = datetime.datetime.now().strftime("%H:%M:%S")
            mode = describe_mode(master, last_heartbeat) if last_heartbeat else "--"
            position = format_position(last_gps)
            battery = format_battery(last_sys_status)
            alt = f"alt: {last_vfr.alt:.1f} m" if last_vfr else "alt: ---"
            airspeed = (
                f"airspeed: {last_vfr.airspeed:.1f} m/s" if last_vfr else "airspeed: ---"
            )

            print(
                f"[{timestamp}] {mode} | {position} | {alt}, {airspeed} | {battery}"
            )
    except KeyboardInterrupt:
        print("\nExiting on user request.")


if __name__ == "__main__":
    main()
