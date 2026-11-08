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
    HOVER = auto()
    EXPLORE = auto()
    NAV_LAND = auto()
    LAND = auto()


@dataclass
class FsmConfig:
    takeoff_alt_m: float = 15.0
    accept_radius_m: float = 2.5
    dwell_s: float = 2.0
    land_on_last_wp: bool = True   # treat final waypoint as the landing point
    min_satellites: int = 8
    max_hdop: float = 2.0
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
        self._takeoff_requested = False
        self._takeoff_alt = cfg.takeoff_alt_m
        self._land_requested = False
        self._active_takeoff_alt = cfg.takeoff_alt_m
        self._next_after_takeoff = State.HOVER
        self._lock = threading.Lock()
        self._last_status = 0.0
        self._guided_lost_count = 0

    # ---- operator commands (called from the MAVLink pump thread) -----------
    def handle_command(self, cmd_id: int, param: float = 0.0) -> None:
        with self._lock:
            if cmd_id == mc.CMD_START:
                self._start_requested = True
                print("[fsm] START requested")
            elif cmd_id == mc.CMD_ABORT:
                self._abort_requested = True
                print("[fsm] ABORT requested")
            elif cmd_id == mc.CMD_TAKEOFF:
                self._takeoff_requested = True
                self._takeoff_alt = param if param > 0 else self.cfg.takeoff_alt_m
                print(f"[fsm] TAKEOFF requested (alt={self._takeoff_alt:.1f}m)")
            elif cmd_id == mc.CMD_LAND:
                self._land_requested = True
                print("[fsm] LAND requested")

    def _consume_start(self) -> bool:
        with self._lock:
            v, self._start_requested = self._start_requested, False
            return v

    def _consume_abort(self) -> bool:
        with self._lock:
            v, self._abort_requested = self._abort_requested, False
            return v

    def _consume_takeoff(self) -> tuple[bool, float]:
        with self._lock:
            v, self._takeoff_requested = self._takeoff_requested, False
            return v, self._takeoff_alt

    def _consume_land(self) -> bool:
        with self._lock:
            v, self._land_requested = self._land_requested, False
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

    def _advance_waypoint(self) -> None:
        old_idx = self.wp_index
        self.wp_index += 1
        self._dwell_until = 0.0
        last_idx = len(self.mission) - 1
        explore_end = last_idx - 1 if self.cfg.land_on_last_wp else last_idx
        if self.wp_index <= explore_end:
            next_wp = self.mission[self.wp_index]
            print(f"[fsm] WP {old_idx + 1} complete -> commanding WP {self.wp_index + 1}/{len(self.mission)}: lat={next_wp.lat}, lon={next_wp.lon}, alt={next_wp.alt}")
            self._goto(next_wp)
        else:
            print(f"[fsm] Completed all exploration waypoints (0..{explore_end}) -> transitioning to NAV_LAND")
            self.state = State.NAV_LAND
            land_wp = self.mission[-1]
            print(f"[fsm] Commanding landing point: lat={land_wp.lat}, lon={land_wp.lon}, alt={land_wp.alt}")
            self._goto(land_wp)

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

            # global abort: stop autonomy, hold in HOVER if airborne else IDLE
            if self._consume_abort():
                if tlm.armed and tlm.rel_alt >= 2.0:
                    print("[fsm] abort -> holding in HOVER")
                    self.state = State.HOVER
                else:
                    self.state = State.IDLE

            # global land command
            if self._consume_land():
                print("[fsm] LAND command received -> commanding land")
                self.mav.land()
                self.state = State.LAND

            self._tick(tlm)
            self._emit_status()
            time.sleep(0.1)  # 10 Hz

    def _tick(self, tlm: Telemetry) -> None:
        if self.state == State.IDLE:
            req_takeoff, to_alt = self._consume_takeoff()
            if req_takeoff:
                if tlm.armed and tlm.rel_alt >= 2.0:
                    print("[fsm] TAKEOFF requested while already airborne -> ensuring GUIDED -> HOVER")
                    if not self._in_guided(tlm):
                        self.mav.set_mode("GUIDED")
                    self.state = State.HOVER
                else:
                    self._active_takeoff_alt = to_alt
                    self._next_after_takeoff = State.HOVER
                    self.state = State.PREARM
            elif self.mission and self._consume_start():
                if tlm.armed and tlm.rel_alt >= 2.0:
                    print("[fsm] START requested while already airborne -> ensuring GUIDED -> EXPLORE")
                    if not self._in_guided(tlm):
                        self.mav.set_mode("GUIDED")
                    self.wp_index = 0
                    self._dwell_until = 0.0
                    self._goto(self.mission[0])
                    self.state = State.EXPLORE
                else:
                    self._active_takeoff_alt = self.cfg.takeoff_alt_m
                    self._next_after_takeoff = State.EXPLORE
                    self.state = State.PREARM

        elif self.state == State.PREARM:
            if tlm.fix_type < 3 or tlm.satellites < self.cfg.min_satellites \
                    or tlm.hdop > self.cfg.max_hdop:
                self._emit_status(f"prearm wait fix={tlm.fix_type} sat={tlm.satellites}")
                return
            target_alt = getattr(self, "_active_takeoff_alt", self.cfg.takeoff_alt_m)
            print(f"[fsm] prearm OK -> GUIDED + arm + takeoff ({target_alt:.1f}m)")
            self.mav.set_mode("GUIDED")
            time.sleep(0.5)
            self.mav.arm()
            time.sleep(1.0)
            self.mav.takeoff(target_alt)
            self.state = State.TAKEOFF

        elif self.state == State.TAKEOFF:
            if not self._in_guided(tlm):
                self._guided_lost_count += 1
                if self._guided_lost_count >= 8:
                    print(f"[fsm] left GUIDED during takeoff (mode={tlm.mode}) -> IDLE")
                    self.state = State.IDLE
                    self._guided_lost_count = 0
                    return
            else:
                self._guided_lost_count = 0

            target_alt = getattr(self, "_active_takeoff_alt", self.cfg.takeoff_alt_m)
            if tlm.rel_alt >= 0.95 * target_alt:
                next_state = getattr(self, "_next_after_takeoff", State.HOVER)
                if next_state == State.EXPLORE and self.mission:
                    print(f"[fsm] reached takeoff alt {tlm.rel_alt:.1f} m -> EXPLORE ({len(self.mission)} waypoints)")
                    self.wp_index = 0
                    self._dwell_until = 0.0
                    self._goto(self.mission[0])
                    self.state = State.EXPLORE
                else:
                    print(f"[fsm] reached takeoff alt {tlm.rel_alt:.1f} m -> HOVER (awaiting operator)")
                    self.state = State.HOVER

        elif self.state == State.HOVER:
            if not self._in_guided(tlm):
                self._guided_lost_count += 1
                if self._guided_lost_count >= 8:
                    print(f"[fsm] left GUIDED during hover (mode={tlm.mode}) -> IDLE")
                    self.state = State.IDLE
                    self._guided_lost_count = 0
                    return
            else:
                self._guided_lost_count = 0

            self._emit_status("hovering in GUIDED")
            if self.mission and self._consume_start():
                print(f"[fsm] START requested from HOVER -> EXPLORE ({len(self.mission)} waypoints)")
                self.wp_index = 0
                self._dwell_until = 0.0
                self._goto(self.mission[0])
                self.state = State.EXPLORE

        elif self.state == State.EXPLORE:
            if not self._in_guided(tlm):
                self._guided_lost_count += 1
                if self._guided_lost_count >= 8:
                    print(f"[fsm] left GUIDED during EXPLORE (mode={tlm.mode}) -> IDLE")
                    self.state = State.IDLE
                    self._guided_lost_count = 0
                    return
            else:
                self._guided_lost_count = 0

            last_idx = len(self.mission) - 1
            # With land_on_last_wp, EXPLORE covers waypoints 0..last-1 and the
            # final descent happens at the last waypoint in NAV_LAND. Otherwise
            # EXPLORE covers all waypoints and NAV_LAND re-targets the last one.
            explore_end = last_idx - 1 if self.cfg.land_on_last_wp else last_idx
            if self.wp_index > explore_end:
                print(f"[fsm] Exploration waypoints complete -> transitioning to NAV_LAND")
                self.state = State.NAV_LAND
                self._dwell_until = 0.0
                land_wp = self.mission[-1]
                print(f"[fsm] Commanding landing point: lat={land_wp.lat}, lon={land_wp.lon}, alt={land_wp.alt}")
                self._goto(land_wp)
                return

            wp = self.mission[self.wp_index]
            self._goto(wp)
            d = self._dist_to(tlm, wp)
            if d > 2000.0:
                self._emit_status(f"wp{self.wp_index + 1}/{len(self.mission)} {d/1000:.1f}km away!")
            else:
                self._emit_status(f"wp{self.wp_index + 1}/{len(self.mission)} d={d:.1f}m")

            if d <= self.cfg.accept_radius_m:
                if self._dwell_until == 0.0:
                    if self.cfg.dwell_s > 0:
                        self._dwell_until = time.time() + self.cfg.dwell_s
                        print(f"[fsm] Arrived at WP {self.wp_index + 1}/{len(self.mission)} (d={d:.1f}m) -> dwelling {self.cfg.dwell_s:.1f}s")
                    else:
                        self._advance_waypoint()
                elif time.time() >= self._dwell_until:
                    self._dwell_until = 0.0
                    self._advance_waypoint()
            else:
                # If dwelling, allow hysteresis up to accept_radius * 1.5 so GPS jitter doesn't reset dwell
                if self._dwell_until > 0.0 and d > self.cfg.accept_radius_m * 1.5:
                    print(f"[fsm] Drifted from WP {self.wp_index + 1} (d={d:.1f}m) -> resetting dwell")
                    self._dwell_until = 0.0

        elif self.state == State.NAV_LAND:
            if not self._in_guided(tlm):
                self._guided_lost_count += 1
                if self._guided_lost_count >= 8:
                    print(f"[fsm] left GUIDED during NAV_LAND (mode={tlm.mode}) -> IDLE")
                    self.state = State.IDLE
                    self._guided_lost_count = 0
                    return
            else:
                self._guided_lost_count = 0

            land_wp = self.mission[-1]
            self._goto(land_wp)
            d = self._dist_to(tlm, land_wp)
            self._emit_status(f"to-land d={d:.1f}m")
            if d <= self.cfg.accept_radius_m:
                print(f"[fsm] Arrived at landing point (d={d:.1f}m) -> commanding LAND")
                self.mav.land()
                self.state = State.LAND

        elif self.state == State.LAND:
            self._emit_status("landing")
            if not tlm.armed:  # ArduPilot auto-disarms on touchdown
                print("[fsm] Disarmed after touchdown -> mission completed successfully, returning to IDLE")
                self.wp_index = 0
                self.state = State.IDLE
