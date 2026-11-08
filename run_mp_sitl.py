#!/usr/bin/env python3
"""One-click runner for the Companion Drone stack connected to Mission Planner SITL:
  1. SITL MAVLink Bridge (simulation/sitl_bridge.py -> tcp:127.0.0.1:5762)
  2. Ground Router Daemon (ground/router/ground_router.py)
  3. Pi Autonomy Daemon (onboard_pi/autonomy/main.py)
  4. React Web UI (Vite dev server at http://127.0.0.1:5173)

Prerequisite: Start SITL in Mission Planner first (Simulation tab -> Multirotor).
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
    print("\n[launcher] Shutting down companion drone services...")
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
    print("[launcher] All companion services terminated.")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, cleanup)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, cleanup)

    print("=" * 65)
    print("   SARX COMPANION DRONE - MISSION PLANNER SITL RUNNER")
    print("=" * 65)

    # 1. SITL MAVLink Bridge
    print("[launcher] (1/4) Launching SITL MAVLink Bridge (connecting to TCP 5762)...")
    p_bridge = subprocess.Popen(
        [VENV_PYTHON, "-u", os.path.join(ROOT, "simulation", "sitl_bridge.py")],
        cwd=ROOT
    )
    PROCS.append(p_bridge)
    time.sleep(1.5)

    # 2. Ground Router
    print("[launcher] (2/4) Launching Ground Router (udpin:14551 -> WS:8080)...")
    p_router = subprocess.Popen(
        [VENV_PYTHON, "-u", os.path.join(ROOT, "ground", "router", "ground_router.py"),
         "--link", "udpin:127.0.0.1:14551", "--mp-udp", "127.0.0.1:14559"],
        cwd=ROOT
    )
    PROCS.append(p_router)
    time.sleep(1.0)

    # 3. Pi Autonomy Daemon
    print("[launcher] (3/4) Launching Pi Autonomy Daemon (conn: udpin:14553)...")
    p_autonomy = subprocess.Popen(
        [VENV_PYTHON, "-u", os.path.join(ROOT, "onboard_pi", "autonomy", "main.py"),
         "--conn", "udpin:127.0.0.1:14553", "--takeoff-alt", "15", "--accept-radius", "2.5"],
        cwd=ROOT
    )
    PROCS.append(p_autonomy)
    time.sleep(1.0)

    # 4. Web UI
    print("[launcher] (4/4) Launching React Web UI...")
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
    print("  ALL SERVICES ARE RUNNING AND CONNECTED TO SITL!")
    print(f"  Web UI:            {url}")
    print("  Ground WebSocket:  ws://127.0.0.1:8080")
    print("  Mission Planner:   Already connected to SITL on TCP 5760")
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

