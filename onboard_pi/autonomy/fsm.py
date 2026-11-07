"""Deterministic autonomy FSM: fly an uploaded waypoint list in GUIDED mode.

States: BOOT -> IDLE -> PREARM -> TAKEOFF -> EXPLORE -> NAV_LAND -> LAND -> IDLE.

The FSM never bypasses ArduPilot safety: the pilot's RC mode switch, the Cube's
failsafes, and the GUIDED setpoint watchdog all remain authoritative. On ABORT
(or losing GUIDED) the FSM drops back to IDLE and stops streaming setpoints,
leaving the vehicle hovering / under pilot control.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "common"))
import mission_codec as mc  # noqa: E402

from mav_interface import MavInterface, Telemetry, haversine_m  # noqa: E402
from mission_rx import MissionReceiver  # noqa: E402
import status_tx  # noqa: E402


class State(Enum):
    BOOT = auto()
    IDLE = auto()
    PREARM = auto()
    TAKEOFF = auto()
    EXPLORE = auto()
    NAV_LAND = auto()
    LAND = auto()


@dataclass
class FsmConfig:
    takeoff_alt_m: float = 15.0
    accept_radius_m: float = 1.5
    dwell_s: float = 2.0
    land_on_last_wp: bool = True   # treat final waypoint as the landing point
    min_satellites: int = 10
    max_hdop: float = 1.2
    status_hz: float = 2.0


class AutonomyFSM:
    def __init__(self, mav: MavInterface, rx: MissionReceiver, cfg: FsmConfig):
        self.mav = mav
        self.rx = rx
        self.cfg = cfg
        self.state = State.BOOT
        self.mission: List[mc.Waypoint] = []
        self.wp_index = 0
        self._dwell_until = 0.0
        self._start_requested = False
        self._abort_requested = False
        self._lock = threading.Lock()
        self._last_status = 0.0

    # ---- operator commands (called from the MAVLink pump thread) -----------
    def handle_command(self, cmd_id: int) -> None:
        with self._lock:
            if cmd_id == mc.CMD_START:
                self._start_requested = True
                print("[fsm] START requested")
            elif cmd_id == mc.CMD_ABORT:
                self._abort_requested = True
                print("[fsm] ABORT requested")

    def _consume_start(self) -> bool:
        with self._lock:
            v, self._start_requested = self._start_requested, False
            return v

    def _consume_abort(self) -> bool:
        with self._lock:
            v, self._abort_requested = self._abort_requested, False
            return v

    # ---- helpers -----------------------------------------------------------
    def _emit_status(self, note: str = "") -> None:
        now = time.time()
        if now - self._last_status >= 1.0 / self.cfg.status_hz:
            self._last_status = now
            status_tx.send_status(self.mav, self.state.name, self.wp_index,
                                  len(self.mission), note)

    def _goto(self, wp: mc.Waypoint) -> None:
        self.mav.goto(wp.lat, wp.lon, wp.alt if wp.alt > 0 else self.cfg.takeoff_alt_m)

    def _dist_to(self, tlm: Telemetry, wp: mc.Waypoint) -> float:
        return haversine_m(tlm.lat, tlm.lon, wp.lat, wp.lon)

    def _in_guided(self, tlm: Telemetry) -> bool:
        return tlm.mode.upper() == "GUIDED"

    # ---- main loop ---------------------------------------------------------
    def run(self) -> None:
        print("[fsm] running")
        self.state = State.IDLE
        while True:
            tlm = self.mav.snapshot()

            # a freshly uploaded mission always returns us to a clean IDLE
            new = self.rx.take_new_mission()
            if new is not None:
                self.mission = new
                self.wp_index = 0
                if self.state not in (State.IDLE,):
                    print("[fsm] new mission mid-flight -> returning to IDLE")
                self.state = State.IDLE

            # global abort: stop autonomy, hold in IDLE (vehicle hovers in GUIDED)
            if self._consume_abort():
                self.state = State.IDLE

            self._tick(tlm)
            self._emit_status()
            time.sleep(0.1)  # 10 Hz

    def _tick(self, tlm: Telemetry) -> None:
        if self.state == State.IDLE:
            if self.mission and self._consume_start():
                self.state = State.PREARM

        elif self.state == State.PREARM:
            if tlm.fix_type < 3 or tlm.satellites < self.cfg.min_satellites \
                    or tlm.hdop > self.cfg.max_hdop:
                self._emit_status(f"prearm wait fix={tlm.fix_type} sat={tlm.satellites}")
                return
            print("[fsm] prearm OK -> GUIDED + arm + takeoff")
            self.mav.set_mode("GUIDED")
            time.sleep(0.5)
            self.mav.arm()
            time.sleep(1.0)
            self.mav.takeoff(self.cfg.takeoff_alt_m)
            self.state = State.TAKEOFF

        elif self.state == State.TAKEOFF:
            if not self._in_guided(tlm):
                print("[fsm] left GUIDED during takeoff -> IDLE")
                self.state = State.IDLE
                return
            if tlm.rel_alt >= 0.95 * self.cfg.takeoff_alt_m:
                print(f"[fsm] reached takeoff alt {tlm.rel_alt:.1f} m -> EXPLORE")
                self.wp_index = 0
                self._dwell_until = 0.0
                if self.mission:
                    self._goto(self.mission[0])
                self.state = State.EXPLORE

        elif self.state == State.EXPLORE:
            if not self._in_guided(tlm):
                print("[fsm] left GUIDED -> IDLE")
                self.state = State.IDLE
                return
            last_idx = len(self.mission) - 1
            # With land_on_last_wp, EXPLORE covers waypoints 0..last-1 and the
            # final descent happens at the last waypoint in NAV_LAND. Otherwise
            # EXPLORE covers all waypoints and NAV_LAND re-targets the last one.
            explore_end = last_idx - 1 if self.cfg.land_on_last_wp else last_idx
            if self.wp_index > explore_end:
                self.state = State.NAV_LAND
                return
            wp = self.mission[self.wp_index]
            self._goto(wp)
            d = self._dist_to(tlm, wp)
            self._emit_status(f"wp{self.wp_index} d={d:.1f}m")
            if d <= self.cfg.accept_radius_m:
                if self._dwell_until == 0.0:
                    self._dwell_until = time.time() + self.cfg.dwell_s
                elif time.time() >= self._dwell_until:
                    self._dwell_until = 0.0
                    self.wp_index += 1
            else:
                self._dwell_until = 0.0

        elif self.state == State.NAV_LAND:
            if not self._in_guided(tlm):
                self.state = State.IDLE
                return
            land_wp = self.mission[-1]
            self._goto(land_wp)
            d = self._dist_to(tlm, land_wp)
            self._emit_status(f"to-land d={d:.1f}m")
            if d <= self.cfg.accept_radius_m:
                print("[fsm] arrived at landing point -> LAND")
                self.mav.land()
                self.state = State.LAND

        elif self.state == State.LAND:
            self._emit_status("landing")
            if not tlm.armed:  # ArduPilot auto-disarms on touchdown
                print("[fsm] disarmed after touchdown -> IDLE")
                self.wp_index = 0
                self.state = State.IDLE
