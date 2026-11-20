System Architecture Overview

The core of this architecture is designed to solve a fundamental hardware constraint: a physical USB Serial COM port can only be opened by one program at a time, yet an autonomous drone operations stack requires two simultaneous ground systems:

Mission Planner (Primary GCS): For full flight instrumentation, artificial horizon HUD, parameter calibration, safety checks, and emergency backup.
Custom Web UI (Mission Dispatcher): A clean, interactive browser map interface for drawing polygons, clicking waypoints, dispatching autonomous search grids, and monitoring mission progress.
1. Architecture Topology Diagram
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │                                   GROUND STATION (PC)                                  │
 │                                                                                        │
 │   ┌─────────────────────────────┐                 ┌────────────────────────────────┐   │
 │   │      Custom Ground UI       │                 │     Mission Planner (GCS)      │   │
 │   │  (React / Leaflet Map UI)   │                 │ (Full HUD, Telemetry, Safety)  │   │
 │   └──────────────┬──────────────┘                 └───────────────▲────────────────┘   │
 │                  │ WebSocket (JSON)                               │ MAVLink UDP        │
 │                  │ (ws://127.0.0.1:8080)                          │ (udp://127.0.0.1:  │
 │                  │                                                │        14550)      │
 │                  ▼                                                ▼                    │
 │   ┌────────────────────────────────────────────────────────────────────────────────┐   │
 │   │                      Ground Router Daemon (ground_router.py)                   │   │
 │   │                                                                                │   │
 │   │  • Owns the single physical USB COM port exclusively                           │   │
 │   │  • Bidirectional MAVLink multiplexer: forwards flight data to Mission Planner │   │
 │   │  • Extracts GPS/attitude to JSON for the Web UI (4 Hz)                         │   │
 │   │  • Compresses Web UI missions into binary MAVLink TUNNEL packets               │   │
 │   └───────────────────────────────────────┬────────────────────────────────────────┘   │
 │                                           │ USB Virtual COM (e.g., COM5 @ 115200 baud) │
 │                                           ▼                                            │
 │                           ┌───────────────────────────────┐                            │
 │                           │   Ground ESP32 (ESP-NOW LR)   │                            │
 │                           └───────────────┬───────────────┘                            │
 └───────────────────────────────────────────┼────────────────────────────────────────────┘
                                             │ 2.4 GHz ESP-NOW Wireless Link
                                             │ (Raw MTU: 250 Bytes)
 ┌───────────────────────────────────────────┼────────────────────────────────────────────┐
 │                                           ▼                                            │
 │                           ┌───────────────────────────────┐                            │
 │                           │    Air ESP32 (ESP-NOW LR)     │                            │
 │                           └───────────────┬───────────────┘                            │
 │                                           │ UART Serial (115200 baud)                  │
 │                                           ▼                                            │
 │   ┌────────────────────────────────────────────────────────────────────────────────┐   │
 │   │                     Raspberry Pi 5 (Companion Computer)                        │   │
 │   │                                                                                │   │
 │   │   ┌───────────────────────────┐           ┌────────────────────────────────┐   │   │
 │   │   │   mavlink-router Daemon   │ ◄────────►│     Autonomy FSM Daemon        │   │   │
 │   │   │ (Local Hardware MUX on Pi)│           │ (Unpacks mission, checks fence,│   │   │
 │   │   └─────────────┬─────────────┘           │  streams GUIDED goto targets)  │   │   │
 │   │                 │                         └────────────────────────────────┘   │   │
 │   └─────────────────┼──────────────────────────────────────────────────────────────┘   │
 │                     │ High-Speed USB CDC / Telem UART (921600 baud)                    │
 │                     ▼                                                                  │
 │   ┌────────────────────────────────────────────────────────────────────────────────┐   │
 │   │                     Cube Orange+ Autopilot (ArduPilot)                         │   │
 │   │   • High-rate attitude stabilization loops (400 Hz)                            │   │
 │   │   • Mode: GUIDED                                                               │   │
 │   │   • Steering and motor outputs to ESCs                                         │   │
 │   └────────────────────────────────────────────────────────────────────────────────┘   │
 └────────────────────────────────────────────────────────────────────────────────────────┘
2. Why Split the COM Port into Two Network Ports?
Reason 1: OS-Level Exclusive Serial Port Locking

Operating systems (both Windows and Linux) lock hardware serial ports exclusively upon opening.

If Mission Planner opened COM5 directly, no other script or web interface could read telemetry or send commands.
If your ground script opened COM5, Mission Planner would be blocked from connecting.
The Solution: The Python Ground Router (ground_router.py) acts as the single master process that claims the physical COM port. It then multiplexes and redistributes the data over loopback network sockets.
Reason 2: Browser Security Sandbox (WebSockets vs. UDP/Serial)
Web browsers running modern web applications (React, Leaflet) run inside a sandbox. A browser cannot open a raw serial COM port or a raw UDP socket.
Browsers can only communicate via HTTP or WebSockets.
The Solution: The Ground Router hosts an asyncio WebSocket server on ws://127.0.0.1:8080. It translates binary MAVLink telemetry into JSON messages ({kind: "position", lat: ..., lon: ..., alt: ...}) that JavaScript running in the browser can render on the map in real time.
Reason 3: Standard GCS Interoperability (UDP 14550)
Mission Planner is designed to connect either to a COM port or to a network endpoint via UDP 14550.
By relaying raw MAVLink packets back and forth between the serial port and UDP 14550, Mission Planner has no idea it is not plugged directly into the drone. You get full telemetry, audio callouts, parameter tuning, and flight mode displays without interfering with the custom web app.
3. The Two Data Streams Inside ground_router.py

The Ground Router runs three concurrent worker threads:

Serial / Link Reader (_pump_link):

Reads raw bytes arriving from the Ground ESP32.
Path A (to Mission Planner): Every valid MAVLink frame received from the drone is immediately forwarded via UDP socket to 127.0.0.1:14550.
Path B (to Web UI): Extracts specific messages needed by the web map:
GLOBAL_POSITION_INT 
→
→ Throttled to ~4 Hz and pushed over WebSocket as JSON (lat, lon, alt, hdg).
HEARTBEAT 
→
→ Pushed as JSON (mode, armed) so the UI displays live flight mode.
TUNNEL 
→
→ Decodes custom ACK, NACK, and FSM status messages from the Raspberry Pi and notifies the UI.

Mission Planner Uplink (_pump_mp):

Listens on UDP 14550 for packets transmitted by Mission Planner (e.g., parameter reads/writes, mode changes, calibrate sensors) and writes them straight out to the physical serial port toward the drone.

Web UI Mission Dispatcher (_pump_mission):

When an operator clicks Transmit Mission or Takeoff in the browser, the Web UI sends a JSON payload across the WebSocket.
The router serializes the waypoints into a compact, CRC-checked binary blob, splits it into 128-byte chunks, wraps each chunk into a standard MAVLink TUNNEL message (Message ID #385, private payload type 200), and injects it onto the serial link.
4. End-to-End Mission Journey: From Browser Click to Motor Spin
User action in Browser:
You click waypoints on the Leaflet map and hit Transmit Mission.
Web UI to Router:
Browser emits a WebSocket JSON frame: {"kind": "mission", "waypoints": [{lat, lon, alt}, ...]}.
Router Compression & Injection:
Router serializes the coordinates into a compact binary struct (8 bytes per coordinate, altitude, and CRC16).
It wraps the struct in a MAVLink TUNNEL packet and transmits it across the USB serial port to the Ground ESP32.
Over-the-Air Wireless Link:
Ground ESP32 transmits the payload across 2.4 GHz in ESP-NOW Long Range mode to the Air ESP32.
Onboard Raspberry Pi Ingest:
Air ESP32 forwards the bytes over UART to the Raspberry Pi 5.
The local autonomy/main.py daemon receives the TUNNEL frame, validates the CRC checksum, sends back a 4-byte ACK to the ground, and stores the waypoint queue in memory.
Takeoff & Guided Navigation:
When you click Start Mission, the Pi sets ArduPilot flight mode to GUIDED, verifies pre-arm GPS health, commands takeoff, and feeds position setpoints (SET_POSITION_TARGET_GLOBAL_INT) sequentially to the Cube Orange+.
Autopilot Execution:
The Cube Orange+ runs the low-level PID attitude controllers and EKF3 state estimator, steering the drone to each target coordinate in sequence, and finally engages autonomous LAND and motor disarm upon completing the mission.