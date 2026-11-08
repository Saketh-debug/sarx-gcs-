#!/usr/bin/env python3
"""SITL MAVLink Bridge for Mission Planner ArduCopter.exe.

Connects to ArduCopter SITL on TCP 5762 (Serial 1 / TELEM 1) and bridges MAVLink
to:
  - Ground Router on UDP 14551
  - Pi Autonomy on UDP 14553

Also bridges TUNNEL messages between Ground Router (14551) and Pi Autonomy (14553),
mirroring what the ESP-NOW link and mavlink-router do on physical hardware.

Usage:
  python simulation/sitl_bridge.py
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "common"))
import mission_codec as mc


class SitlBridge:
    def __init__(self, sitl_tcp: str = "tcp:127.0.0.1:5762",
                 ground_port: int = 14551, autonomy_port: int = 14553):
        self.sitl_tcp = sitl_tcp
        self.ground_addr = ("127.0.0.1", ground_port)
        self.autonomy_addr = ("127.0.0.1", autonomy_port)

        print(f"[bridge] Connecting to ArduCopter SITL on {sitl_tcp} ...")
        self.mav_sitl = mavutil.mavlink_connection(sitl_tcp, dialect="ardupilotmega")
        self.mav_sitl.wait_heartbeat()
        print(f"[bridge] Connected to ArduCopter SITL! (Sys {self.mav_sitl.target_system}, Comp {self.mav_sitl.target_component})")

        # Request continuous telemetry streams (position, attitude, raw sensors, rc)
        self.mav_sitl.mav.request_data_stream_send(
            self.mav_sitl.target_system,
            self.mav_sitl.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  # 10 Hz
            1    # start
        )

        # UDP socket for sending to and receiving from Ground Router and Autonomy
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))  # bind to ephemeral local port
        self.udp_sock.settimeout(0.1)

        self._running = True

    def sitl_to_endpoints(self):
        """Read from ArduCopter TCP and forward to Ground Router & Autonomy."""
        while self._running:
            try:
                msg = self.mav_sitl.recv_match(blocking=True, timeout=0.1)
            except Exception:
                continue
            if msg is None:
                continue

            raw = msg.get_msgbuf()
            for addr in (self.ground_addr, self.autonomy_addr):
                try:
                    self.udp_sock.sendto(raw, addr)
                except Exception:
                    pass

    def endpoints_to_sitl(self):
        """Listen on UDP for commands/TUNNEL from Ground Router & Autonomy."""
        parser = mavlink2.MAVLink(None)
        while self._running:
            try:
                data, addr = self.udp_sock.recvfrom(4096)
            except socket.timeout:
                continue
            except Exception:
                continue

            msgs = parser.parse_buffer(data)
            if any(m.get_type() == "TUNNEL" for m in msgs):
                print(f"[bridge] TUNNEL frame received from {addr} (len={len(data)})")
            if not msgs:
                # Forward raw bytes to SITL anyway if not parsed
                try:
                    self.mav_sitl.write(data)
                except Exception:
                    pass
                continue

            for msg in msgs:
                mtype = msg.get_type()
                # Bridge TUNNEL frames directly between Ground Router and Autonomy
                if mtype == "TUNNEL" and msg.payload_type == mc.TUNNEL_PAYLOAD_TYPE:
                    fwd_bytes = msg.get_msgbuf()
                    src_comp = msg.get_srcComponent()
                    if src_comp == 190 or addr[1] == self.ground_addr[1]:
                        # From Ground Router -> forward to Pi Autonomy
                        self.udp_sock.sendto(fwd_bytes, self.autonomy_addr)
                    else:
                        # From Pi Autonomy -> forward to Ground Router
                        self.udp_sock.sendto(fwd_bytes, self.ground_addr)
                    continue

                # Forward all commands/requests directly to SITL preserving original sysid/compid
                try:
                    self.mav_sitl.write(msg.get_msgbuf())
                except Exception:
                    pass

    def run(self):
        print("=" * 65)
        print("         SITL MAVLINK BRIDGE RUNNING")
        print(f"  ArduCopter SITL:  {self.sitl_tcp}")
        print(f"  Ground Router:    udpout:{self.ground_addr[0]}:{self.ground_addr[1]}")
        print(f"  Pi Autonomy:      udpout:{self.autonomy_addr[0]}:{self.autonomy_addr[1]}")
        print("=" * 65)

        t1 = threading.Thread(target=self.sitl_to_endpoints, daemon=True)
        t2 = threading.Thread(target=self.endpoints_to_sitl, daemon=True)
        t1.start()
        t2.start()

        try:
            while self._running:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n[bridge] Stopping...")
            self._running = False


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--sitl-tcp", default="tcp:127.0.0.1:5762")
    p.add_argument("--ground-port", type=int, default=14551)
    p.add_argument("--autonomy-port", type=int, default=14553)
    args = p.parse_args()

    bridge = SitlBridge(args.sitl_tcp, args.ground_port, args.autonomy_port)
    bridge.run()

