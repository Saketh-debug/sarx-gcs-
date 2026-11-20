#!/usr/bin/env python3
"""Lightweight mock ArduCopter vehicle & MAVLink bridge for Windows/Linux testing.

Simulates an ArduCopter flight controller (sysid=1, compid=1) and relays
MAVLink between:
  - Ground Router (udpout:127.0.0.1:14551)
  - Companion Autonomy (udpout:127.0.0.1:14553)
  - Mission Planner GCS (udpout:127.0.0.1:14550)

Allows end-to-end testing of the Companion Drone stack (Web UI -> Router ->
Autonomy FSM -> Simulated Vehicle) on Windows without compiling ArduPilot SITL.

Usage:
  python simulation/mock_vehicle.py
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "common"))
import mission_codec as mc


class MockVehicle:
    def __init__(self, ground_port=14551, autonomy_port=14553, mp_port=14550,
                 init_lat=17.4123, init_lon=78.1234):
        self.ground_addr = ("127.0.0.1", ground_port)
        self.autonomy_addr = ("127.0.0.1", autonomy_port)
        self.mp_addr = ("127.0.0.1", mp_port)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind on local port
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.1)

        self.mav = mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)

        # Vehicle state
        self.lat = init_lat
        self.lon = init_lon
        self.alt = 0.0  # relative alt in meters
        self.hdg = 0.0  # degrees
        self.armed = False
        self.mode = "LOITER"  # STABILIZE=0, ALT_HOLD=2, GUIDED=4, LOITER=5, LAND=9
        self.mode_id = 5

        self.target_lat = init_lat
        self.target_lon = init_lon
        self.target_alt = 0.0
        self.is_flying = False
        self.is_landing = False

        self._running = True
        self._lock = threading.Lock()

    def _dist_to(self, target_lat, target_lon) -> float:
        r = 6371000.0
        p1, p2 = math.radians(self.lat), math.radians(target_lat)
        dp = math.radians(target_lat - self.lat)
        dl = math.radians(target_lon - self.lon)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return 2 * r * math.asin(min(1.0, math.sqrt(a)))

    def _bearing_to(self, target_lat, target_lon) -> float:
        p1, p2 = math.radians(self.lat), math.radians(target_lat)
        dl = math.radians(target_lon - self.lon)
        y = math.sin(dl) * math.cos(p2)
        x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360

    def physics_loop(self):
        dt = 0.1
        speed_mps = 5.0
        climb_mps = 2.0
        descend_mps = 1.5

        while self._running:
            time.sleep(dt)
            with self._lock:
                if not self.armed:
                    continue

                # Altitude physics
                if self.is_landing:
                    if self.alt > 0.1:
                        self.alt = max(0.0, self.alt - descend_mps * dt)
                    else:
                        self.alt = 0.0
                        self.armed = False
                        self.is_flying = False
                        self.is_landing = False
                        self.mode = "LAND"
                        self.mode_id = 9
                        print("[vehicle] Touchdown! Disarmed.")
                        continue
                elif self.is_flying and self.alt < self.target_alt:
                    self.alt = min(self.target_alt, self.alt + climb_mps * dt)

                # Horizontal physics
                if self.is_flying and not self.is_landing:
                    dist = self._dist_to(self.target_lat, self.target_lon)
                    if dist > 0.3:
                        bearing = self._bearing_to(self.target_lat, self.target_lon)
                        self.hdg = bearing
                        step = min(dist, speed_mps * dt)
                        # convert step meters to lat/lon degrees
                        dlat = (step * math.cos(math.radians(bearing))) / 111320.0
                        dlon = (step * math.sin(math.radians(bearing))) / (111320.0 * math.cos(math.radians(self.lat)))
                        self.lat += dlat
                        self.lon += dlon

    def telemetry_loop(self):
        mav = self.mav
        while self._running:
            time.sleep(0.25)  # 4 Hz
            with self._lock:
                armed_flag = mavlink2.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0
                custom_flag = mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                base_mode = armed_flag | custom_flag

                # 1. HEARTBEAT
                hb = mav.heartbeat_encode(
                    mavlink2.MAV_TYPE_QUADROTOR,
                    mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    base_mode,
                    self.mode_id,
                    mavlink2.MAV_STATE_ACTIVE if self.armed else mavlink2.MAV_STATE_STANDBY
                ).pack(mav)

                # 2. GLOBAL_POSITION_INT
                pos = mav.global_position_int_encode(
                    int(time.time() * 1000) & 0xFFFFFFFF,
                    int(self.lat * 1e7),
                    int(self.lon * 1e7),
                    int(self.alt * 1000),       # alt AMSL (approx)
                    int(self.alt * 1000),       # relative alt
                    0, 0, 0,
                    int(self.hdg * 100)         # cdeg
                ).pack(mav)

                # 3. GPS_RAW_INT (3D fix, 14 sats, HDOP 0.8)
                gps = mav.gps_raw_int_encode(
                    int(time.time() * 1e6) & 0xFFFFFFFFFFFFFFFF,
                    3,  # 3D fix
                    int(self.lat * 1e7),
                    int(self.lon * 1e7),
                    int(self.alt * 1000),
                    80,  # eph (HDOP 0.8)
                    80,
                    0, 0,
                    14  # satellites visible
                ).pack(mav)

                # 4. SYS_STATUS (Battery voltage & capacity)
                sys_stat = mav.sys_status_encode(
                    0, 0, 0, 500,     # sensors present, enabled, health, load
                    15800,            # 15.80V (4S LiPo)
                    45,               # 0.45A
                    94,               # 94% battery remaining
                    0, 0, 0, 0, 0, 0  # drop_rate_comm, errors_comm, errors_count 1..4
                ).pack(mav)

                # 5. ATTITUDE
                att = mav.attitude_encode(
                    int(time.time() * 1000) & 0xFFFFFFFF,
                    0.005,         # roll rad (~0.3 deg)
                    -0.008,        # pitch rad (~-0.5 deg)
                    math.radians(self.hdg),
                    0.0, 0.0, 0.0
                ).pack(mav)

            for target in (self.ground_addr, self.autonomy_addr, self.mp_addr):
                try:
                    self.sock.sendto(hb, target)
                    self.sock.sendto(pos, target)
                    self.sock.sendto(gps, target)
                    self.sock.sendto(sys_stat, target)
                    self.sock.sendto(att, target)
                except Exception:
                    pass

    def rx_loop(self):
        parser = mavlink2.MAVLink(None)
        while self._running:
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except Exception:
                break

            msgs = parser.parse_buffer(data)
            if not msgs:
                continue

            for msg in msgs:
                mtype = msg.get_type()

                # Bridge TUNNEL frames between Ground Router (14551) and Autonomy (14553)
                if mtype == "TUNNEL" and msg.payload_type == mc.TUNNEL_PAYLOAD_TYPE:
                    fwd_bytes = msg.get_msgbuf()
                    src_comp = msg.get_srcComponent()
                    print(f"[mock_vehicle] TUNNEL from {addr} (src_comp={src_comp}) -> relaying", flush=True)
                    if src_comp == 190 or addr[1] == self.ground_addr[1]:
                        # Received from ground router (comp 190) -> relay to autonomy
                        self.sock.sendto(fwd_bytes, self.autonomy_addr)
                    else:
                        # Received from autonomy (comp 191) -> relay to ground router
                        self.sock.sendto(fwd_bytes, self.ground_addr)
                    continue

                with self._lock:
                    if mtype == "COMMAND_LONG":
                        cmd = msg.command
                        if cmd == mavlink2.MAV_CMD_COMPONENT_ARM_DISARM:
                            arm_val = int(msg.param1) == 1
                            self.armed = arm_val
                            print(f"[vehicle] ARM command received -> armed={self.armed}")
                        elif cmd == mavlink2.MAV_CMD_NAV_TAKEOFF:
                            self.target_alt = msg.param7
                            self.is_flying = True
                            self.is_landing = False
                            print(f"[vehicle] TAKEOFF command received -> climb to {self.target_alt}m")
                        elif cmd == mavlink2.MAV_CMD_NAV_LAND:
                            self.is_landing = True
                            self.mode = "LAND"
                            self.mode_id = 9
                            print("[vehicle] LAND command received -> descending")
                    elif mtype == "SET_MODE":
                        custom_mode = msg.custom_mode
                        self.mode_id = custom_mode
                        if custom_mode == 4:
                            self.mode = "GUIDED"
                        elif custom_mode == 5:
                            self.mode = "LOITER"
                        elif custom_mode == 9:
                            self.mode = "LAND"
                        print(f"[vehicle] SET_MODE command received -> mode={self.mode} (id={custom_mode})")
                    elif mtype == "SET_POSITION_TARGET_GLOBAL_INT":
                        self.target_lat = msg.lat_int / 1e7
                        self.target_lon = msg.lon_int / 1e7
                        self.target_alt = msg.alt
                        self.is_flying = True

    def run(self):
        print("=" * 60)
        print("Mock ArduCopter SITL & MAVLink Hub")
        print(f"  Ground Router port:   {self.ground_addr}")
        print(f"  Autonomy Daemon port: {self.autonomy_addr}")
        print(f"  Mission Planner port: {self.mp_addr}")
        print(f"  Initial Position:     lat={self.lat}, lon={self.lon}, alt=0.0m")
        print("=" * 60)

        t_rx = threading.Thread(target=self.rx_loop, daemon=True)
        t_tlm = threading.Thread(target=self.telemetry_loop, daemon=True)
        t_phy = threading.Thread(target=self.physics_loop, daemon=True)

        t_rx.start()
        t_tlm.start()
        t_phy.start()

        try:
            while self._running:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n[vehicle] Stopping...")
            self._running = False


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ground-port", type=int, default=14551)
    parser.add_argument("--autonomy-port", type=int, default=14553)
    parser.add_argument("--mp-port", type=int, default=14550)
    args = parser.parse_args()

    v = MockVehicle(args.ground_port, args.autonomy_port, args.mp_port)
    v.run()

