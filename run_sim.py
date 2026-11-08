#!/usr/bin/env python3
"""One-click runner for the full Companion Drone simulation stack:
  1. Mock ArduCopter Vehicle & MAVLink Hub (simulation/mock_vehicle.py)
  2. Ground Router Daemon (ground/router/ground_router.py)
  3. Autonomy Daemon (onboard_pi/autonomy/main.py)
  4. React Web UI (Vite dev server at http://127.0.0.1:5173)

Press Ctrl+C to stop all services cleanly.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
import webbrowser

ROOT = os.path.dirname(os.path.abspath(__file__))
VENV_PYTHON = os.path.join(ROOT, ".venv", "Scripts", "python.exe")
if not os.path.exists(VENV_PYTHON):
    VENV_PYTHON = os.path.join(ROOT, ".venv", "bin", "python")
if not os.path.exists(VENV_PYTHON):
    VENV_PYTHON = sys.executable

PROCS: list[subprocess.Popen] = []


def cleanup(*args):
    print("\n[launcher] Shutting down all companion drone services...")
    for p in PROCS:
        try:
            p.terminate()
        except Exception:
            pass
    for p in PROCS:
        try:
            p.wait(timeout=2)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
    print("[launcher] All services terminated.")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, cleanup)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, cleanup)

    print("=" * 65)
    print("      COMPANION DRONE GCS - COMPLETE SIMULATION STACK")
    print("=" * 65)

    # 1. Mock ArduCopter Vehicle
    print("[launcher] (1/4) Launching Mock ArduCopter SITL Hub...")
    p_vehicle = subprocess.Popen(
        [VENV_PYTHON, os.path.join(ROOT, "simulation", "mock_vehicle.py")],
        cwd=ROOT
    )
    PROCS.append(p_vehicle)
    time.sleep(1.0)

    # 2. Ground Router
    print("[launcher] (2/4) Launching Ground Router (udpin:14551 -> WS:8080)...")
    p_router = subprocess.Popen(
        [VENV_PYTHON, os.path.join(ROOT, "ground", "router", "ground_router.py"),
         "--link", "udpin:127.0.0.1:14551"],
        cwd=ROOT
    )
    PROCS.append(p_router)
    time.sleep(1.0)

    # 3. Pi Autonomy Daemon
    print("[launcher] (3/4) Launching Pi Autonomy Daemon (conn: udpin:14553)...")
    p_autonomy = subprocess.Popen(
        [VENV_PYTHON, os.path.join(ROOT, "onboard_pi", "autonomy", "main.py"),
         "--conn", "udpin:127.0.0.1:14553", "--takeoff-alt", "15", "--accept-radius", "2.5"],
        cwd=ROOT
    )
    PROCS.append(p_autonomy)
    time.sleep(1.0)

    # 4. Web UI
    print("[launcher] (4/4) Launching React Web UI (Vite dev server)...")
    npm_cmd = "npm.cmd" if sys.platform == "win32" else "npm"
    web_dir = os.path.join(ROOT, "ground", "web_ui")
    p_ui = subprocess.Popen(
        [npm_cmd, "run", "dev"],
        cwd=web_dir
    )
    PROCS.append(p_ui)
    time.sleep(2.0)

    url = "http://127.0.0.1:5173"
    print("\n" + "=" * 65)
    print("  ALL SERVICES ARE RUNNING!")
    print(f"  Web UI:            {url}")
    print("  Ground WebSocket:  ws://127.0.0.1:8080")
    print("  Mission Planner:   Connect UDP to 127.0.0.1:14550 (optional)")
    print("  Sample Mission:    samples/mission.kml")
    print("=" * 65)
    print("Opening web UI in browser... Press Ctrl+C anytime to stop.\n")

    try:
        webbrowser.open(url)
    except Exception:
        pass

    try:
        while True:
            for p in PROCS:
                code = p.poll()
                if code is not None:
                    print(f"[launcher] Process {p.args} exited with code {code}")
                    cleanup()
            time.sleep(1.0)
    except KeyboardInterrupt:
        cleanup()


if __name__ == "__main__":
    main()

