#!/usr/bin/env python3
"""Bench/SITL helper: send a mission straight to the autonomy over TUNNEL.

Lets you exercise the Pi autonomy (mission_rx + FSM) without the web UI or the
ground router in the loop. Reads waypoints from a KML or JSON file, packs them
with the shared codec, sends them as TUNNEL chunks, optionally issues START, and
prints any ACK / STATUS frames that come back.

Examples
--------
  # Off-vehicle codec round-trip (no connection needed):
  python mock_mission_sender.py --selftest

  # Send a KML mission to an autonomy listening on udp 14553, then start it:
  python mock_mission_sender.py --conn udpout:127.0.0.1:14553 \
      --kml ../samples/mission.kml --start
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import xml.dom.minidom as minidom

os.environ.setdefault("MAVLINK20", "1")  # TUNNEL (msg 385) is MAVLink 2 only
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "common"))
import mission_codec as mc  # noqa: E402


def parse_kml(path: str):
    """Extract an ordered waypoint list from a KML file (lon,lat,alt order)."""
    dom = minidom.parse(path)
    wps = []
    for node in dom.getElementsByTagName("coordinates"):
        text = (node.firstChild.data if node.firstChild else "").strip()
        for triple in text.replace("\n", " ").split():
            parts = triple.split(",")
            if len(parts) >= 2:
                lon, lat = float(parts[0]), float(parts[1])
                alt = float(parts[2]) if len(parts) >= 3 else 0.0
                wps.append(mc.Waypoint(lat, lon, alt))
    return wps


def selftest():
    wps = [mc.Waypoint(17.4123 + i * 1e-4, 78.1234 + i * 1e-4, 15) for i in range(25)]
    blob = mc.pack_mission(wps)
    asm = mc.MissionReassembler()
    out = None
    for chunk in mc.chunk_blob(blob):
        out = asm.feed(chunk) or out
    assert out is not None and len(out) == len(wps), "count mismatch"
    for a, b in zip(wps, out):
        assert abs(a.lat - b.lat) < 1e-6 and abs(a.lon - b.lon) < 1e-6 and a.alt == b.alt
    print(f"[selftest] OK: {len(wps)} waypoints round-tripped through TUNNEL chunking")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--selftest", action="store_true", help="codec round-trip only")
    p.add_argument("--conn", help="pymavlink connection (e.g. udpout:127.0.0.1:14553)")
    p.add_argument("--kml", help="KML file to read waypoints from")
    p.add_argument("--start", action="store_true", help="also send CMD_START")
    p.add_argument("--alt", type=float, default=15.0, help="altitude for KML wp with alt=0")
    args = p.parse_args()

    if args.selftest:
        selftest()
        return

    if not args.conn or not args.kml:
        p.error("need --conn and --kml (or --selftest)")

    from pymavlink import mavutil
    wps = parse_kml(args.kml)
    for w in wps:
        if w.alt <= 0:
            w.alt = args.alt
    print(f"[mock] parsed {len(wps)} waypoints from {args.kml}")

    m = mavutil.mavlink_connection(args.conn, source_system=255, source_component=200,
                                   dialect="ardupilotmega")
    for payload in mc.chunk_blob(mc.pack_mission(wps)):
        m.mav.tunnel_send(0, 0, mc.TUNNEL_PAYLOAD_TYPE, len(payload), list(payload))
        time.sleep(0.05)
    print("[mock] mission sent")

    if args.start:
        time.sleep(1.0)
        m.mav.tunnel_send(0, 0, mc.TUNNEL_PAYLOAD_TYPE, 128,
                          list(mc.make_cmd(mc.CMD_START)))
        print("[mock] START sent")

    print("[mock] listening for ACK/STATUS (Ctrl-C to quit) ...")
    try:
        while True:
            msg = m.recv_match(type="TUNNEL", blocking=True, timeout=1)
            if msg is None or msg.payload_type != mc.TUNNEL_PAYLOAD_TYPE:
                continue
            frame = mc.parse_frame(bytes(msg.payload[:msg.payload_length]))
            if not frame:
                continue
            if frame["type"] == mc.MSG_ACK:
                print(f"[mock] ACK count={frame['data'][0]}")
            elif frame["type"] == mc.MSG_NACK:
                print("[mock] NACK")
            elif frame["type"] == mc.MSG_STATUS:
                print(f"[mock] STATUS {frame['data'].decode('utf-8', 'replace')}")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
