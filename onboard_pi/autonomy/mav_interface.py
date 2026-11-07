"""Thin pymavlink wrapper for GUIDED companion control on the Pi.

A background thread pumps the MAVLink stream and keeps a live `Telemetry`
snapshot. Commands (mode/arm/takeoff/setpoint/land) are thin wrappers around the
usual ArduPilot MAVLink messages. Incoming TUNNEL payloads addressed to our
custom payload_type are handed to a callback (the mission receiver).
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

os.environ.setdefault("MAVLINK20", "1")  # TUNNEL (msg 385) is MAVLink 2 only
from pymavlink import mavutil

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "common"))
import mission_codec as mc  # noqa: E402

EARTH_METERS_PER_DEG = 111319.5
DIALECT = "ardupilotmega"


@dataclass
class Telemetry:
    connected: bool = False
    lat: float = 0.0
    lon: float = 0.0
    rel_alt: float = 0.0
    armed: bool = False
    mode: str = ""
    fix_type: int = 0
    satellites: int = 0
    hdop: float = 99.9
    last_heartbeat: float = 0.0
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    """Great-circle distance in metres (good enough at waypoint scales)."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


class MavInterface:
    def __init__(self, conn_str: str, on_tunnel: Optional[Callable[[bytes], None]] = None):
        self.conn_str = conn_str
        self.master = mavutil.mavlink_connection(conn_str, source_system=1,
                                                 source_component=191, dialect=DIALECT)
        self.tlm = Telemetry()
        self._on_tunnel = on_tunnel
        self._running = True
        self._target_sys = 1
        self._target_comp = 1

    # ---- lifecycle ---------------------------------------------------------
    def wait_heartbeat(self, timeout: float = 60.0) -> bool:
        print(f"[mav] waiting for heartbeat on {self.conn_str} ...")
        hb = self.master.wait_heartbeat(timeout=timeout)
        if hb is None:
            return False
        self._target_sys = self.master.target_system
        self._target_comp = self.master.target_component
        with self.tlm._lock:
            self.tlm.connected = True
        print(f"[mav] heartbeat from system {self._target_sys} component {self._target_comp}")
        return True

    def start(self) -> None:
        threading.Thread(target=self._pump, daemon=True).start()

    def stop(self) -> None:
        self._running = False

    def _pump(self) -> None:
        while self._running:
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            t = msg.get_type()
            with self.tlm._lock:
                if t == "HEARTBEAT":
                    self.tlm.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.tlm.mode = mavutil.mode_string_v10(msg)
                    self.tlm.last_heartbeat = time.time()
                elif t == "GLOBAL_POSITION_INT":
                    self.tlm.lat = msg.lat / 1e7
                    self.tlm.lon = msg.lon / 1e7
                    self.tlm.rel_alt = msg.relative_alt / 1000.0
                elif t == "GPS_RAW_INT":
                    self.tlm.fix_type = msg.fix_type
                    self.tlm.satellites = msg.satellites_visible
                    self.tlm.hdop = msg.eph / 100.0 if msg.eph != 65535 else 99.9
            if t == "TUNNEL" and msg.payload_type == mc.TUNNEL_PAYLOAD_TYPE and self._on_tunnel:
                self._on_tunnel(bytes(msg.payload[:msg.payload_length]))

    def snapshot(self) -> Telemetry:
        with self.tlm._lock:
            return Telemetry(**{k: v for k, v in self.tlm.__dict__.items() if k != "_lock"})

    # ---- commands ----------------------------------------------------------
    def send_tunnel(self, payload: bytes) -> None:
        self.master.mav.tunnel_send(0, 0, mc.TUNNEL_PAYLOAD_TYPE, len(payload), list(payload))

    def set_mode(self, mode_name: str) -> None:
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self._target_sys,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

    def arm(self) -> None:
        self.master.mav.command_long_send(
            self._target_sys, self._target_comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    def takeoff(self, alt_m: float) -> None:
        self.master.mav.command_long_send(
            self._target_sys, self._target_comp,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt_m)

    def goto(self, lat: float, lon: float, alt_m: float) -> None:
        """Stream one GUIDED position target (position-only type_mask)."""
        self.master.mav.set_position_target_global_int_send(
            0, self._target_sys, self._target_comp,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # position enabled; vel/accel/yaw ignored
            int(lat * 1e7), int(lon * 1e7), alt_m,
            0, 0, 0, 0, 0, 0, 0, 0)

    def land(self) -> None:
        self.master.mav.command_long_send(
            self._target_sys, self._target_comp,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
