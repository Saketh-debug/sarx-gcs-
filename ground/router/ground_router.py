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
        self._running = True

    # ---- link <-> MP forwarding + UI extraction ----------------------------
    def _pump_link(self) -> None:
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
            # forward every valid frame to Mission Planner untouched
            try:
                self.mp.write(msg.get_msgbuf())
            except Exception:
                pass

            if mtype == "GLOBAL_POSITION_INT":
                now = time.time()
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
                    self.ui_outbox.put({
                        "kind": "heartbeat",
                        "armed": bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                        "mode": mavutil.mode_string_v10(msg),
                    })
            elif mtype == "TUNNEL" and msg.payload_type == mc.TUNNEL_PAYLOAD_TYPE:
                self._handle_tunnel(bytes(msg.payload[:msg.payload_length]))

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
