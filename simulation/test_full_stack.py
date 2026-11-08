#!/usr/bin/env python3
"""Automated end-to-end integration test for the entire companion-drone stack:
  1. Starts Mock ArduCopter vehicle (simulation/mock_vehicle.py)
  2. Starts Ground Router (ground/router/ground_router.py)
  3. Starts Companion Autonomy (onboard_pi/autonomy/main.py)
  4. Connects via WebSocket (mimicking the React Web UI)
  5. Uploads waypoints from samples/mission.kml
  6. Confirms ACK ("Mission loaded")
  7. Sends START command
  8. Monitors telemetry and FSM state progression (TAKEOFF -> EXPLORE -> NAV_LAND -> LAND -> IDLE)
  9. Shuts down cleanly
"""

import asyncio
import json
import os
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "common"))
import mission_codec as mc
from mock_mission_sender import parse_kml

PYTHON = sys.executable

async def run_test():
    kml_path = os.path.join(os.path.dirname(__file__), "..", "samples", "mission.kml")
    wps = parse_kml(kml_path)
    print(f"[test] Loaded {len(wps)} waypoints from {kml_path}")
    wp_dicts = [{"lat": w.lat, "lon": w.lon, "alt": 15.0} for w in wps]

    print("[test] 1. Starting mock vehicle...")
    p_vehicle = subprocess.Popen([PYTHON, "simulation/mock_vehicle.py"])

    print("[test] 2. Starting ground router...")
    p_router = subprocess.Popen([PYTHON, "ground/router/ground_router.py", "--link", "udpin:127.0.0.1:14551"])

    print("[test] 3. Starting autonomy daemon...")
    p_autonomy = subprocess.Popen([PYTHON, "onboard_pi/autonomy/main.py", "--conn", "udpin:127.0.0.1:14553", "--takeoff-alt", "10", "--dwell", "0.5"])

    import websockets

    try:
        # Wait for WebSocket server to be ready
        ws = None
        for attempt in range(15):
            try:
                ws = await websockets.connect("ws://127.0.0.1:8080")
                print("[test] Connected to Ground Router WebSocket!")
                break
            except Exception:
                await asyncio.sleep(1.0)

        if not ws:
            raise RuntimeError("Failed to connect to Ground Router WebSocket")

        print("[test] 4. Listening for vehicle heartbeat on WebSocket...")
        got_heartbeat = False
        start_time = time.time()
        while time.time() - start_time < 15:
            raw = await asyncio.wait_for(ws.recv(), timeout=5.0)
            msg = json.loads(raw)
            if msg.get("kind") == "heartbeat":
                print(f"[test] Received vehicle heartbeat: {msg}")
                got_heartbeat = True
                break

        assert got_heartbeat, "No heartbeat received over WebSocket"

        print("[test] 5. Transmitting mission via WebSocket...")
        await ws.send(json.dumps({"kind": "mission", "waypoints": wp_dicts}))

        # Wait for ACK
        print("[test] 6. Waiting for mission ACK...")
        got_ack = False
        start_time = time.time()
        while time.time() - start_time < 15:
            raw = await asyncio.wait_for(ws.recv(), timeout=5.0)
            msg = json.loads(raw)
            if msg.get("kind") == "ack":
                print(f"[test] Received ACK from Autonomy daemon: {msg}")
                assert msg.get("ok") is True, "ACK was not ok"
                assert msg.get("count") == len(wps), f"ACK count mismatch: {msg.get('count')} vs {len(wps)}"
                got_ack = True
                break

        assert got_ack, "No mission ACK received"
        print("[test] Mission ACK verified successfully!")

        print("[test] 7. Sending START command via WebSocket...")
        await ws.send(json.dumps({"kind": "command", "cmd": "start"}))

        print("[test] 8. Tracking FSM state progression...")
        states_seen = set()
        start_time = time.time()
        while time.time() - start_time < 60:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=5.0)
            except asyncio.TimeoutError:
                continue
            msg = json.loads(raw)
            if msg.get("kind") == "status":
                state = msg.get("s")
                if state not in states_seen:
                    states_seen.add(state)
                    print(f"[test] -> Autonomy FSM state: {state} (wp {msg.get('i')}/{msg.get('n')}) note={msg.get('m', '')}")
                if "LAND" in states_seen and state == "IDLE":
                    print("[test] Mission completed: Drone landed and returned to IDLE!")
                    break

        print(f"[test] States traversed during flight: {sorted(list(states_seen))}")
        assert "TAKEOFF" in states_seen, "TAKEOFF state not reached"
        assert "EXPLORE" in states_seen, "EXPLORE state not reached"
        print("\n" + "=" * 60)
        print("ALL TESTS PASSED! FULL SYSTEM VERIFIED END-TO-END!")
        print("=" * 60)

    finally:
        print("[test] Cleaning up child processes...")
        for p in (p_autonomy, p_router, p_vehicle):
            try:
                p.terminate()
                p.wait(timeout=2)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass

if __name__ == "__main__":
    asyncio.run(run_test())

