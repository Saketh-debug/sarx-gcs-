# Companion Drone GCS — KML-Driven Autonomy

Upload a Mission Planner **KML waypoint plan** (or click waypoints on the map),
hit **Transmit** then **Start**, and the Raspberry Pi flies the waypoints in
ArduPilot **GUIDED** mode — takeoff → each waypoint → landing point → land.
Mission Planner stays the primary GCS; the RC flight-mode switch and all Cube
failsafes can take over at any instant.

The ESP-NOW telemetry bridge is assumed already working. This repo adds only the
**ground web UI + router** and the **Pi autonomy**.

## What's here

```
common/mission_codec.py     Shared waypoint pack/unpack + CRC16 + TUNNEL chunking
ground/router/              Router: owns the COM port, splits to MP (UDP) + UI (WS),
                            injects the mission as MAVLink TUNNEL
ground/web_ui/              React/Leaflet UI: KML upload, draw-on-map, live position, FSM status
onboard_pi/autonomy/        FSM daemon that flies the uploaded waypoints (pymavlink)
onboard_pi/config/          mavlink-router.conf + systemd unit
simulation/                 SITL runner + mock mission sender / codec self-test
samples/mission.kml         Example MP-style waypoint KML
```

Data path: `UI → (WebSocket) → router → MAVLink TUNNEL → ESP-NOW → Pi autonomy`,
with ACK + FSM status flowing back the same way.

## Run it (SITL first — no hardware)

```bash
# 1. ArduPilot SITL + MAVProxy hub (needs ardupilot source / sim_vehicle.py)
./simulation/sitl_runner.sh

# 2. Ground router (connects to SITL's spare output as the "link")
pip install -r ground/router/requirements.txt
python ground/router/ground_router.py --link udpin:127.0.0.1:14551

# 3. Web UI
cd ground/web_ui && npm install && npm run dev        # http://localhost:5173

# 4. Autonomy (stands in for the Pi)
python onboard_pi/autonomy/main.py --conn udpin:127.0.0.1:14553
```

Then in the browser: upload `samples/mission.kml` (or click the map) → **Transmit**
→ wait for "Mission loaded ✓" → **Start**. Optionally connect Mission Planner to
UDP `127.0.0.1:14550`.

Quick off-vehicle checks:
```bash
python simulation/mock_mission_sender.py --selftest     # codec round-trip
```

## Manual steps you execute yourself

**A. Mission Planner — export the KML.** Flight Plan tab → build the waypoint
mission → Save/Export as **KML** (`.kml`). That file is what you upload.

**B. Cube Orange+ params** (Full Parameter Tree → set → Write):
`SERIAL0_BAUD=921600`, `SERIAL2_PROTOCOL=2`, `SERIAL2_BAUD=921600`,
`FS_GCS_ENABLE=1`, `FS_GCS_TIMEOUT=5`, `FS_THR_ENABLE=1`, `FS_EKF_ACTION=1`,
`BATT_FS_LOW_ACT=2`, `BATT_FS_CRT_ACT=1`, `DISARM_DELAY=10`, `WP_YAW_BEHAVIOR=1`.
(Optional hard fence: `FENCE_ENABLE=1`, `FENCE_TYPE`, `FENCE_ACTION`.)

**C. RC transmitter (FlySky FS-i6S).** Map `FLTMODE_CH`; set switch positions
`1=AltHold`, `2=Loiter`, `3=Guided`.

**D. Raspberry Pi 5.**
1. Wire the air ESP32 to a Pi UART; confirm `/dev/ttyAMA0` and the Cube at
   `/dev/serial/by-id/usb-ArduPilot_CubeOrangePlus*`.
2. `sudo apt install mavlink-router python3-pip && pip install -r onboard_pi/requirements.txt`
3. Edit the device paths in `onboard_pi/config/mavlink-router.conf`, copy it to
   `/etc/mavlink-router/main.conf`, install `autonomy.service`, enable both.

**E. Windows ground station.**
1. Install Python 3.11+ and Node 20+.
2. Find the ground ESP32 **COM port** (Device Manager).
3. `python ground/router/ground_router.py --link COMx --baud 115200`
4. `cd ground/web_ui && npm install && npm run dev`
5. Point **Mission Planner** at UDP `127.0.0.1:14550` (do not open the COM port
   in MP — the router owns it).

## Validation gates (do in order)

1. **SITL** — full UI→Pi→land loop on the simulated map.
2. **Codec** — `mock_mission_sender.py --selftest`; and
   `--conn udpout:127.0.0.1:14553 --kml samples/mission.kml --start` against the
   autonomy to test mission RX + FSM in isolation.
3. **Bench, PROPS OFF** — real Cube + Pi + ESP-NOW. Confirm GUIDED + motor spool,
   then the three failsafes (RC→Loiter instant override; `kill -9` autonomy → 3 s
   Loiter hold; unplug ground ESP32 → 5 s RTL). **Confirm TUNNEL survives the real
   ESP-NOW bridge here.**
4. **Field** — 5 m box low/slow first, then a full KML survey, hand on the switch.

## Safety notes
- The UI never auto-arms: **Start** is a separate explicit gate after the ACK.
- The FSM drops to IDLE (vehicle hovers) if GUIDED is lost or on **Abort**.
- ArduPilot's GUIDED 3 s setpoint watchdog, geofence, and battery/EKF/RC
  failsafes remain the authoritative safety net.
