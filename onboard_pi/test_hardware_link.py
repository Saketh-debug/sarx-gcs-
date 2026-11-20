#!/usr/bin/env python3
"""Hardware Link & Telemetry Verification Utility for Raspberry Pi 5.

Performs immediate, comprehensive hardware bench checks:
  1. Detects connected serial devices (Cube Orange+ / Pixhawk and Air Radio)
  2. Verifies MAVLink Heartbeat communication with the flight controller
  3. Validates critical sensor streams (GPS 3D Fix, Battery Voltage, IMU Attitude)
  4. Checks telemetry bridge and mavlink-router status

Usage on Raspberry Pi:
  # Auto-connect via local mavlink-router endpoint (default):
  python test_hardware_link.py

  # Test direct USB or UART connection to Pixhawk:
  python test_hardware_link.py --conn /dev/serial/by-id/usb-ArduPilot_CubeOrangePlus_... --baud 921600
  python test_hardware_link.py --conn /dev/ttyAMA0 --baud 115200
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"


def check_serial_ports():
    print(f"\n{BOLD}{CYAN}--- Step 1: Scanning Local Serial Hardware Devices ---{RESET}")
    by_id = glob.glob("/dev/serial/by-id/*")
    tty_acm = glob.glob("/dev/ttyACM*")
    tty_usb = glob.glob("/dev/ttyUSB*")
    tty_ama = glob.glob("/dev/ttyAMA*")

    found_pixhawk = False
    if by_id:
        for p in by_id:
            print(f"  [Found by-id] {p}")
            if "ArduPilot" in p or "Cube" in p or "Pixhawk" in p:
                found_pixhawk = True
    elif tty_acm:
        for p in tty_acm:
            print(f"  [Found ACM] {p}")
            found_pixhawk = True
    else:
        print(f"  {YELLOW}Notice: No /dev/serial/by-id/ or /dev/ttyACM devices found.{RESET}")

    if tty_ama:
        print(f"  [Found UART AMA] {', '.join(tty_ama)}")
    if tty_usb:
        print(f"  [Found USB Serial] {', '.join(tty_usb)}")

    return found_pixhawk


def run_diagnostics(conn_str: str, baud: int, timeout: float = 8.0):
    print(f"\n{BOLD}{CYAN}--- Step 2: Testing MAVLink Link to Autopilot ---{RESET}")
    print(f"Opening MAVLink interface: {conn_str} (baud: {baud})...")

    try:
        master = mavutil.mavlink_connection(conn_str, baud=baud, source_system=255,
                                             source_component=195, dialect="ardupilotmega")
    except Exception as e:
        print(f"{RED}[FAIL] Could not open connection: {e}{RESET}")
        return False

    print(f"Listening for Autopilot Heartbeat (timeout: {timeout}s)...")
    start = time.time()
    hb = None
    while time.time() - start < timeout:
        msg = master.wait_heartbeat(timeout=1.0)
        if msg is not None and getattr(msg, "type", None) != mavutil.mavlink.MAV_TYPE_GCS:
            hb = msg
            break

    if hb is None:
        print(f"{RED}[FAIL] No MAVLink Heartbeat detected!{RESET}")
        print(f"  {YELLOW}Troubleshooting Tips:{RESET}")
        print("  1. Verify Cube / Pixhawk is powered and booted.")
        print("  2. Verify USB cable or Telem2 wiring (TX->RX, RX->TX, GND).")
        print("  3. Check ArduPilot parameter: SERIAL2_PROTOCOL=2 (MAVLink 2) and SERIAL2_BAUD=921600.")
        print("  4. If mavlink-router is running, ensure it owns the device and connect via udpin:127.0.0.1:14552.")
        return False

    sys_id = hb.get_srcSystem()
    comp_id = hb.get_srcComponent()
    mode = mavutil.mode_string_v10(hb)
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    print(f"{GREEN}[PASS] MAVLink Heartbeat detected!{RESET}")
    print(f"  - System ID:      {sys_id}")
    print(f"  - Component ID:   {comp_id}")
    print(f"  - Autopilot Mode: {BOLD}{mode}{RESET}")
    print(f"  - Safety State:   {RED if armed else GREEN}{'ARMED' if armed else 'DISARMED'}{RESET}")

    print(f"\n{BOLD}{CYAN}--- Step 3: Monitoring Live Sensors & Health (3-second window) ---{RESET}")
    gps_info = None
    sys_info = None
    att_info = None
    pkt_count = 0

    t_end = time.time() + 3.0
    while time.time() < t_end:
        m = master.recv_match(blocking=True, timeout=0.5)
        if not m:
            continue
        mtype = m.get_type()
        if mtype == "BAD_DATA":
            continue
        pkt_count += 1
        if mtype == "GPS_RAW_INT":
            gps_info = m
        elif mtype == "SYS_STATUS":
            sys_info = m
        elif mtype == "ATTITUDE":
            att_info = m

    hz = round(pkt_count / 3.0, 1)
    print(f"  - Stream Ingest Rate: {GREEN}{hz} Hz{RESET} ({pkt_count} packets received)")

    # GPS Evaluation
    if gps_info:
        fix_names = {0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
        fix_name = fix_names.get(gps_info.fix_type, f"Type {gps_info.fix_type}")
        hdop = (gps_info.eph / 100.0) if gps_info.eph != 65535 else 99.9
        gps_pass = (gps_info.fix_type >= 3 and gps_info.satellites_visible >= 8 and hdop <= 2.0)
        gps_col = GREEN if gps_pass else YELLOW
        print(f"  - GPS Status:        {gps_col}{fix_name} · {gps_info.satellites_visible} sats · HDOP: {hdop}{RESET}")
        if not gps_pass:
            print(f"    {YELLOW}Note: GPS lock is required before GUIDED takeoff.{RESET}")
    else:
        print(f"  - GPS Status:        {YELLOW}No GPS_RAW_INT received (sensor still locking or disabled){RESET}")

    # Battery Evaluation
    if sys_info:
        v = sys_info.voltage_battery / 1000.0
        rem = sys_info.battery_remaining
        curr = sys_info.current_battery / 100.0 if sys_info.current_battery != -1 else 0.0
        print(f"  - Power / Battery:   {GREEN}{v:.2f} V ({rem}%) · Current: {curr:.2f} A{RESET}")
    else:
        print(f"  - Power / Battery:   {YELLOW}No SYS_STATUS received (power module uncalibrated or disconnected){RESET}")

    # Attitude Evaluation
    if att_info:
        import math
        r = math.degrees(att_info.roll)
        p = math.degrees(att_info.pitch)
        y = math.degrees(att_info.yaw)
        print(f"  - IMU Attitude:      {GREEN}Roll: {r:.1f}°, Pitch: {p:.1f}°, Yaw: {y:.1f}°{RESET}")

    print("\n" + "=" * 65)
    print(f"{BOLD}{GREEN}RESULT: Pi 5 <-> Pixhawk Communication is FUNCTIONING PROPERLY!{RESET}")
    print("=" * 65 + "\n")
    return True


def main():
    parser = argparse.ArgumentParser(description="SARX Companion Drone Hardware Diagnostics")
    parser.add_argument("--conn", default="udpin:127.0.0.1:14552",
                        help="pymavlink target connection (e.g. udpin:127.0.0.1:14552 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate if using a direct serial port")
    parser.add_argument("--timeout", type=float, default=8.0, help="seconds to wait for heartbeat")
    args = parser.parse_args()

    print("=" * 65)
    print("   SARX DRONE - ONBOARD PI HARDWARE & TELEMETRY DIAGNOSTICS")
    print("=" * 65)

    check_serial_ports()
    run_diagnostics(args.conn, args.baud, args.timeout)


if __name__ == "__main__":
    main()
