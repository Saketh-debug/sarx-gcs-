#!/usr/bin/env python3
"""Autonomy daemon entrypoint (runs on the Raspberry Pi 5).

Connects to the local mavlink-router UDP endpoint, waits for a Cube heartbeat,
wires the TUNNEL dispatch (mission chunks -> MissionReceiver, operator commands
-> FSM), and runs the autonomy FSM loop.

Usage:
    python main.py --conn udpin:127.0.0.1:14552 --takeoff-alt 15
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "common"))

import mission_codec as mc  # noqa: E402
from mav_interface import MavInterface  # noqa: E402
from mission_rx import MissionReceiver  # noqa: E402
from fsm import AutonomyFSM, FsmConfig  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description="Companion autonomy daemon")
    p.add_argument("--conn", default="udpin:127.0.0.1:14552",
                   help="pymavlink connection to mavlink-router (or SITL)")
    p.add_argument("--takeoff-alt", type=float, default=15.0)
    p.add_argument("--accept-radius", type=float, default=1.5)
    p.add_argument("--dwell", type=float, default=2.0)
    p.add_argument("--no-land-on-last", action="store_true",
                   help="do not treat the final waypoint as the landing point")
    return p.parse_args()


def main():
    args = parse_args()
    cfg = FsmConfig(
        takeoff_alt_m=args.takeoff_alt,
        accept_radius_m=args.accept_radius,
        dwell_s=args.dwell,
        land_on_last_wp=not args.no_land_on_last,
    )

    # Dispatch TUNNEL frames to the right handler based on frame type.
    rx_holder = {}
    fsm_holder = {}

    def on_tunnel(payload: bytes) -> None:
        frame = mc.parse_frame(payload)
        if frame is None:
            return
        if frame["type"] == mc.MSG_MISSION:
            rx_holder["rx"].on_tunnel(payload)
        elif frame["type"] == mc.MSG_CMD and frame["data"]:
            fsm_holder["fsm"].handle_command(frame["data"][0])

    mav = MavInterface(args.conn, on_tunnel=on_tunnel)
    rx = MissionReceiver(mav)
    rx_holder["rx"] = rx

    if not mav.wait_heartbeat(timeout=120):
        print("[main] no heartbeat; exiting")
        sys.exit(1)
    mav.start()

    fsm = AutonomyFSM(mav, rx, cfg)
    fsm_holder["fsm"] = fsm
    try:
        fsm.run()
    except KeyboardInterrupt:
        print("\n[main] stopping")
        mav.stop()


if __name__ == "__main__":
    main()
