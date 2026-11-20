#!/usr/bin/env python3
"""Ground router / multiplexer.

Sole owner of the ESP-NOW virtual COM port. It:
  * forwards all MAVLink both ways between the link and Mission Planner (UDP),
  * serves the web UI over a WebSocket, pushing live position + FSM status as JSON,
  * accepts a waypoint mission from the web UI, packs it with the shared codec,
    and injects it onto the link as MAVLink TUNNEL chunks.

Mission Planner connects as a UDP client to 127.0.0.1:14550 (NOT to the COM port).

Examples
--------
  # Real hardware (Windows): the ground ESP32 shows up as e.g. COM5
  python ground_router.py --link COM5 --baud 115200

  # SITL: point the link at the simulator's spare MAVLink output
  python ground_router.py --link udp:127.0.0.1:14551
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import queue
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")  # TUNNEL (msg 385) is MAVLink 2 only
from pymavlink import mavutil

DIALECT = "ardupilotmega"

# Make the shared codec importable whether run from repo root or this folder.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "common"))
import mission_codec as mc  # noqa: E402


class Router:
    def __init__(self, link_str: str, baud: int, mp_udp: str) -> None:
        print(f"[router] opening link: {link_str} (baud {baud})")
        self.link = mavutil.mavlink_connection(link_str, baud=baud, source_system=255,
                                               source_component=190, dialect=DIALECT)
        print(f"[router] Mission Planner endpoint: udpin:{mp_udp}")
        self.mp = mavutil.mavlink_connection(f"udpin:{mp_udp}", source_system=255,
                                             source_component=190, dialect=DIALECT)

        self.ui_outbox: "queue.Queue[dict]" = queue.Queue()      # JSON -> web UI
        self.mission_inbox: "queue.Queue[list]" = queue.Queue()  # waypoints <- web UI
        self.command_inbox: "queue.Queue[int]" = queue.Queue()   # cmd id <- web UI
        self._last_pos_push = 0.0
        self._last_gps_push = 0.0
        self._last_sys_push = 0.0
        self._last_att_push = 0.0
        self._last_stats_push = 0.0
        self._last_hb_time = 0.0
        self._packet_count = 0
        self._last_packet_count = 0
        self._last_stats_time = time.time()
        self._running = True

    # ---- link <-> MP forwarding + UI extraction ----------------------------
    def _pump_link(self) -> None:
        import math
        import struct
        while self._running:
            try:
                msg = self.link.recv_match(blocking=True, timeout=0.5)
            except Exception:
                continue
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == "BAD_DATA":
                continue

            self._packet_count += 1
            # forward every valid frame to Mission Planner untouched
            try:
                self.mp.write(msg.get_msgbuf())
            except Exception:
                pass

            now = time.time()
            if mtype == "GLOBAL_POSITION_INT":
                if now - self._last_pos_push >= 0.25:  # ~4 Hz to the browser
                    self._last_pos_push = now
                    self.ui_outbox.put({
                        "kind": "position",
                        "lat": msg.lat / 1e7,
                        "lon": msg.lon / 1e7,
                        "alt": msg.relative_alt / 1000.0,
                        "hdg": (msg.hdg / 100.0) if msg.hdg != 65535 else None,
                    })
            elif mtype == "HEARTBEAT":
                if getattr(msg, "type", None) != mavutil.mavlink.MAV_TYPE_GCS:
                    self._last_hb_time = now
                    self.ui_outbox.put({
                        "kind": "heartbeat",
                        "armed": bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                        "mode": mavutil.mode_string_v10(msg),
                        "sys_id": msg.get_srcSystem(),
                        "comp_id": msg.get_srcComponent(),
                    })
            elif mtype == "GPS_RAW_INT":
                if now - self._last_gps_push >= 0.5:  # 2 Hz
                    self._last_gps_push = now
                    self.ui_outbox.put({
                        "kind": "gps",
                        "fix_type": msg.fix_type,
                        "satellites": msg.satellites_visible,
                        "hdop": (msg.eph / 100.0) if msg.eph != 65535 else 99.9,
                    })
            elif mtype == "SYS_STATUS":
                if now - self._last_sys_push >= 0.5:  # 2 Hz
                    self._last_sys_push = now
                    self.ui_outbox.put({
                        "kind": "sys_status",
                        "voltage": round(msg.voltage_battery / 1000.0, 2),
                        "current": round(msg.current_battery / 100.0, 2) if msg.current_battery != -1 else None,
                        "battery_remaining": msg.battery_remaining if msg.battery_remaining != -1 else None,
                    })
            elif mtype == "ATTITUDE":
                if now - self._last_att_push >= 0.2:  # 5 Hz
                    self._last_att_push = now
                    self.ui_outbox.put({
                        "kind": "attitude",
                        "roll": round(math.degrees(msg.roll), 1),
                        "pitch": round(math.degrees(msg.pitch), 1),
                        "yaw": round(math.degrees(msg.yaw), 1),
                    })
            elif mtype == "RADIO_STATUS":
                self.ui_outbox.put({
                    "kind": "radio",
                    "rssi": msg.rssi,
                    "remrssi": msg.remrssi,
                    "rxerrors": msg.rxerrors,
                    "fixed": msg.fixed,
                    "noise": getattr(msg, "noise", 0),
                    "remnoise": getattr(msg, "remnoise", 0),
                })
            elif mtype == "TUNNEL" and msg.payload_type == mc.TUNNEL_PAYLOAD_TYPE:
                self._handle_tunnel(bytes(msg.payload[:msg.payload_length]))

            # Periodic link statistics push (1 Hz)
            if now - self._last_stats_push >= 1.0:
                dt = now - self._last_stats_time
                pkts = self._packet_count - self._last_packet_count
                rate_hz = round(pkts / dt, 1) if dt > 0 else 0.0
                self._last_stats_push = now
                self._last_stats_time = now
                self._last_packet_count = self._packet_count
                hb_age = round(now - self._last_hb_time, 1) if self._last_hb_time > 0 else None
                self.ui_outbox.put({
                    "kind": "link_stats",
                    "rate_hz": rate_hz,
                    "total_packets": self._packet_count,
                    "hb_age": hb_age,
                })

    def _handle_tunnel(self, payload: bytes) -> None:
        frame = mc.parse_frame(payload)
        if frame is None:
            return
        if frame["type"] == mc.MSG_ACK:
            count, crc = frame["data"][0], int.from_bytes(frame["data"][1:3], "little")
            self.ui_outbox.put({"kind": "ack", "ok": True, "count": count, "crc": crc})
        elif frame["type"] == mc.MSG_NACK:
            self.ui_outbox.put({"kind": "ack", "ok": False})
        elif frame["type"] == mc.MSG_STATUS:
            # STATUS data = UTF-8 JSON produced by the Pi's status_tx
            try:
                self.ui_outbox.put({"kind": "status", **json.loads(frame["data"].decode("utf-8"))})
            except Exception:
                pass
        elif frame["type"] == mc.MSG_PONG:
            try:
                import struct
                sent_t = struct.unpack("<d", frame["data"][:8])[0]
                rtt = (time.time() - sent_t) * 1000.0
                self.ui_outbox.put({"kind": "pong", "rtt_ms": round(rtt, 1), "t": sent_t})
            except Exception:
                pass

    def _pump_mp(self) -> None:
        while self._running:
            try:
                msg = self.mp.recv_match(blocking=True, timeout=0.5)
            except Exception:
                continue
            if msg is None or msg.get_type() == "BAD_DATA":
                continue
            try:
                self.link.write(msg.get_msgbuf())  # uplink from MP to the vehicle
            except Exception:
                pass

    def _pump_mission(self) -> None:
        while self._running:
            try:
                waypoints = self.mission_inbox.get(timeout=0.5)
            except queue.Empty:
                waypoints = None
            if waypoints is not None:
                self._send_mission(waypoints)
            try:
                item = self.command_inbox.get_nowait()
                if isinstance(item, tuple):
                    cmd_id, alt = item
                else:
                    cmd_id, alt = item, 0.0
                self._send_command(cmd_id, alt)
            except queue.Empty:
                pass

    def _send_command(self, cmd_id: int, alt: float = 0.0) -> None:
        payload = mc.make_cmd(cmd_id, alt)
        print(f"[router] sending command id={cmd_id} (param alt={alt:.1f}m)")
        self.link.mav.tunnel_send(0, 0, mc.TUNNEL_PAYLOAD_TYPE, len(payload), list(payload))

    def _send_mission(self, wp_dicts: list) -> None:
        wps = [mc.Waypoint(w["lat"], w["lon"], w.get("alt", 0)) for w in wp_dicts]
        blob = mc.pack_mission(wps)
        chunks = mc.chunk_blob(blob)
        print(f"[router] transmitting mission: {len(wps)} wp, {len(chunks)} TUNNEL chunk(s)")
        print(f"[router] self.link clients: {getattr(self.link, 'clients', None)}")
        for payload in chunks:
            self.link.mav.tunnel_send(
                0, 0, mc.TUNNEL_PAYLOAD_TYPE, len(payload), list(payload))
            time.sleep(0.05)  # gentle spacing for the lossy ESP-NOW link

    def start_threads(self) -> None:
        for fn in (self._pump_link, self._pump_mp, self._pump_mission):
            threading.Thread(target=fn, daemon=True).start()

    def stop(self) -> None:
        self._running = False


# ---- WebSocket server (asyncio) --------------------------------------------
async def ws_handler(websocket, router: Router):
    import websockets  # local import keeps the dep optional for non-UI use
    clients.add(websocket)
    print("[ws] web UI connected")
    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if msg.get("kind") == "mission" and isinstance(msg.get("waypoints"), list):
                router.mission_inbox.put(msg["waypoints"])
            elif msg.get("kind") == "ping":
                router.command_inbox.put((mc.CMD_PING, time.time()))
            elif msg.get("kind") == "command":
                cmd_map = {
                    "start": mc.CMD_START,
                    "abort": mc.CMD_ABORT,
                    "takeoff": mc.CMD_TAKEOFF,
                    "land": mc.CMD_LAND,
                }
                cmd_id = cmd_map.get(msg.get("cmd"))
                if cmd_id is not None:
                    alt = float(msg.get("alt", 0.0))
                    router.command_inbox.put((cmd_id, alt))
    except websockets.ConnectionClosed:
        pass
    finally:
        clients.discard(websocket)
        print("[ws] web UI disconnected")


clients: set = set()


async def ui_broadcaster(router: Router):
    loop = asyncio.get_event_loop()
    while True:
        item = await loop.run_in_executor(None, _drain_ui, router)
        if item is None:
            continue
        payload = json.dumps(item)
        for ws in list(clients):
            try:
                await ws.send(payload)
            except Exception:
                clients.discard(ws)


def _drain_ui(router: Router):
    try:
        return router.ui_outbox.get(timeout=0.5)
    except queue.Empty:
        return None


async def main_async(router: Router, ws_host: str, ws_port: int):
    import websockets
    router.start_threads()
    async with websockets.serve(lambda ws: ws_handler(ws, router), ws_host, ws_port):
        print(f"[ws] serving web UI on ws://{ws_host}:{ws_port}")
        await ui_broadcaster(router)


def parse_args():
    p = argparse.ArgumentParser(description="Companion-drone ground router")
    p.add_argument("--link", required=True,
                   help="pymavlink connection to the ESP-NOW link or SITL "
                        "(e.g. COM5, /dev/ttyUSB0, udp:127.0.0.1:14551)")
    p.add_argument("--baud", type=int, default=115200, help="serial baud (ignored for UDP)")
    p.add_argument("--mp-udp", default="127.0.0.1:14550",
                   help="UDP endpoint Mission Planner connects to")
    p.add_argument("--ws-host", default="127.0.0.1")
    p.add_argument("--ws-port", type=int, default=8080)
    return p.parse_args()


def main():
    args = parse_args()
    router = Router(args.link, args.baud, args.mp_udp)
    try:
        asyncio.run(main_async(router, args.ws_host, args.ws_port))
    except KeyboardInterrupt:
        print("\n[router] shutting down")
        router.stop()


if __name__ == "__main__":
    main()
