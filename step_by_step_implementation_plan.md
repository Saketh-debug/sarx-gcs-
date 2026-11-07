# Comprehensive Implementation Plan: Autonomous Companion Drone System

This document outlines the step-by-step engineering roadmap to write, configure, and validate all software and firmware components for the autonomous companion drone architecture.

---

## 1. Project Directory Structure

Organize the project repository on the development host, ground laptop, and Raspberry Pi 5 as follows:

```
companion-drone/
├── common/
│   ├── protocol.py               # Shared binary struct pack/unpack & CRC16 logic
│   └── protocol_def.h            # C/C++ header for ESP32 struct compatibility
├── firmware/
│   ├── esp_now_ground/
│   │   └── esp_now_ground.ino    # Ground ESP32 bridge (USB Serial <-> ESP-NOW)
│   └── esp_now_air/
│       └── esp_now_air.ino       # Air ESP32 bridge (UART <-> ESP-NOW)
├── ground/
│   ├── router/
│   │   ├── ground_router.py      # Serial reader, UDP MAVLink switch, WS server
│   │   └── requirements.txt
│   └── web_ui/
│       ├── package.json
│       ├── src/
│       │   ├── App.jsx           # Mission planner interface (Leaflet map)
│       │   ├── components/
│       │   └── utils/mission.js  # Polygon packing & survey line generation
│       └── vite.config.js
├── onboard_pi/
│   ├── config/
│   │   ├── mavlink-router.conf   # MAVLink endpoint definitions
│   │   └── autonomy.service      # systemd unit file for autostart
│   ├── autonomy/
│   │   ├── main.py               # Finite State Machine entrypoint
│   │   ├── fsm.py                # State definitions (Boot, Idle, Arm, Guided, Land)
│   │   ├── grid_planner.py       # Boustrophedon grid coordinate generator
│   │   ├── geofence.py           # Ray-casting point-in-polygon validator
│   │   └── mav_interface.py      # pymavlink GUIDED setpoint & telemetry wrapper
│   └── requirements.txt
└── simulation/
    ├── sitl_runner.sh            # ArduPilot SITL quickstart wrapper
    └── mock_mission_sender.py    # Test script feeding mock binary packets
```

---

## 2. Module Breakdown & Development Sequence

```
 ┌────────────────────────────────────────────────────────────────────────┐
 │                    SYSTEM IMPLEMENTATION ROADMAP                       │
 ├────────────────────────────────────────────────────────────────────────┤
 │ Phase 1: Communication Bridge & Serialization                          │
 │   • Define 48-byte binary frame & CRC16 algorithm                      │
 │   • Write & flash Ground and Air ESP32 firmware (ESP-NOW LR)           │
 │   • Verify bidirectional loopback throughput                           │
 ├────────────────────────────────────────────────────────────────────────┤
 │ Phase 2: Ground Router & Bidirectional Multiplexer                     │
 │   • Develop Python Ground Router Daemon                                │
 │   • Route COM port to Mission Planner (UDP 14550) & Web UI (WS 8080)   │
 │   • Verify concurrent uplink/downlink without packet corruption        │
 ├────────────────────────────────────────────────────────────────────────┤
 │ Phase 3: Ground Web Dispatch Interface                                 │
 │   • Implement React Leaflet UI with live vehicle tracking marker       │
 │   • Add boundary polygon drawer & survey transect calculator           │
 │   • Serialize mission parameters into binary buffer for transmission   │
 ├────────────────────────────────────────────────────────────────────────┤
 │ Phase 4: Onboard Companion Stack (Raspberry Pi 5)                      │
 │   • Install & configure mavlink-router daemon                          │
 │   • Implement deterministic Python FSM & Ray-Casting validator         │
 │   • Implement pymavlink sequential setpoint streamer                   │
 ├────────────────────────────────────────────────────────────────────────┤
 │ Phase 5: ArduPilot Autopilot (Cube Orange+) Setup                      │
 │   • Configure high-speed UART telemetry & GUIDED watchdog              │
 │   • Configure hardware geofence and failsafe actions                   │
 ├────────────────────────────────────────────────────────────────────────┤
 │ Phase 6: Multi-Stage Testing & Verification                            │
 │   • Virtual SITL verification                                          │
 │   • Benchtop Hardware-in-the-Loop (props off)                          │
 │   • Failsafe stress validation & field flight trials                   │
 └────────────────────────────────────────────────────────────────────────┘
```

---

## Phase 1: Binary Protocol & ESP-NOW Firmware

### Step 1.1: Protocol Specification (`common/protocol.py` & `common/protocol_def.h`)
Define a fixed binary structure under 48 bytes (well within ESP-NOW's 250-byte MTU limit).

* **Byte Layout:**
  * `0x00` (`uint8`): Header / Message ID (`0xA1` for Mission Upload, `0xA2` for Telemetry Summary, `0x06` for ACK).
  * `0x01` (`uint8`): Polygon vertex count $N$ ($3 \le N \le 8$).
  * `0x02 - 0x21` ($8 \times \text{int32}$): Latitude & Longitude pairs ($\text{coord} \times 10^7$).
  * `0x22 - 0x29` ($2 \times \text{int32}$): Landing Target Latitude & Longitude ($\text{coord} \times 10^7$).
  * `0x2A` (`uint8`): Cruise Altitude (meters).
  * `0x2B` (`uint8`): Survey Speed (m/s).
  * `0x2C` (`uint8`): Lane Spacing (meters).
  * `0x2D` (`uint8`): Grid Angle (degrees, $0-180^\circ$).
  * `0x2E - 0x2F` (`uint16`): CRC-CCITT (0x1021) Checksum.

### Step 1.2: Ground ESP32 Firmware (`firmware/esp_now_ground/`)
* Initialize USB Virtual COM via hardware UART at `115200` baud.
* Initialize ESP-NOW in Long Range (LR) mode (`esp_wifi_set_protocol` with `WIFI_PROTOCOL_LR`).
* **Bidirectional Task:**
  1. **USB $\rightarrow$ ESP-NOW:** Read incoming serial bytes, frame into packets $\le 250$ bytes, and transmit to Air ESP32 MAC address.
  2. **ESP-NOW $\rightarrow$ USB:** On packet receive callback, write raw bytes directly to USB serial.

### Step 1.3: Air ESP32 Firmware (`firmware/esp_now_air/`)
* Bind hardware UART (pins `GPIO16`/`GPIO17` or default `TX0`/`RX0`) to the Raspberry Pi 5 at `115200` baud.
* Implement matching ESP-NOW LR configuration with Ground ESP32 MAC address whitelisted.
* Forward all over-the-air packets directly to the Pi's UART buffer.

### Verification Gate 1
* Connect Ground ESP32 to PC and Air ESP32 to FTDI adapter on PC.
* Run a Python test script streaming pseudo-random 100-byte packets at 10 Hz.
* Confirm $\ge 99.5\%$ packet delivery over a 5-minute continuous test.

---

## Phase 2: Ground Router & Bidirectional Multiplexer

### Step 2.1: Implement `ground/router/ground_router.py`
The router acts as the central link manager on the ground laptop.

1. **Serial Ingest Worker:**
   * Open the Ground ESP32 USB COM port (e.g., `COM3` or `/dev/ttyUSB0`) at `115200` baud using `pyserial`.
   * Continuously read the stream. Identify MAVLink packets by framing bytes (`0xFD` for MAVLink 2, `0xFE` for MAVLink 1) and custom binary packets (`0xA1`/`0xA2`).
2. **MAVLink Forwarder (Bidirectional UDP):**
   * Bind a non-blocking UDP socket to `127.0.0.1:14550` (Mission Planner default).
   * Any incoming MAVLink packets from serial are forwarded to UDP `14550`.
   * Any incoming UDP packets from Mission Planner are forwarded back to serial (uplink).
3. **WebSocket Server for Web UI:**
   * Run an `asyncio` WebSocket server on `ws://127.0.0.1:8080`.
   * Decode MAVLink message `GLOBAL_POSITION_INT` and broadcast JSON coordinates (`lat`, `lon`, `alt`, `hdg`) to connected browser clients at 4 Hz.
   * Listen for incoming binary mission blobs from the Web UI and queue them directly to the serial write buffer.

### Verification Gate 2
* Connect Mission Planner via UDP (`127.0.0.1:14550`).
* Feed simulated MAVLink heartbeats into the Ground ESP32 serial port.
* Confirm Mission Planner connects, sounds the heartbeat chime, and displays telemetry while the WebSocket server simultaneously broadcasts position updates.

---

## Phase 3: Ground Web UI Development

### Step 3.1: React & Leaflet Foundation (`ground/web_ui/`)
1. Setup a lightweight Vite + React application with `leaflet` and `react-leaflet`.
2. Connect to `ws://127.0.0.1:8080` to receive live vehicle position:
   * Render a custom drone icon rotating according to telemetry heading.
   * Auto-pan the map to the drone's position on first fix.

### Step 3.2: Polygon & Mission Planning Tools
1. **Interactive Bounding Box:**
   * Enable polygon drawing on the map (minimum 3, maximum 8 points).
   * Add a draggable marker for the target landing location.
2. **Survey Line Generator (`utils/mission.js`):**
   * Compute bounding box extrema: $Lat_{\min}, Lat_{\max}, Lon_{\min}, Lon_{\max}$.
   * Generate parallel transect lines spaced according to the lane-spacing input slider.
   * Clip lines against the defined polygon boundary.
3. **Binary Serializer:**
   * Pack vertices, landing coordinates, altitude, speed, lane spacing, and transect angle using `DataView` or an `ArrayBuffer`.
   * Compute the CRC16-CCITT checksum and append it.
   * Transmit the array buffer via the active WebSocket connection to the Ground Router.

### Verification Gate 3
* Draw an irregular 5-sided polygon on the map interface.
* Click "Transmit Mission".
* Verify in the Ground Router console that the received payload matches the exact byte sequence and passes CRC verification.

---

## Phase 4: Raspberry Pi 5 Onboard Software Stack

### Step 4.1: MAVLink Router Configuration (`/etc/mavlink-router/main.conf`)
The companion computer uses `mavlink-routerd` to bridge the autopilot hardware UART, local Python scripts, and the Air ESP32 bridge:

```ini
[General]
ReportStats = false
MavlinkDialect = ardupilotmega

[UartEndpoint cube]
Device = /dev/serial/by-id/usb-ArduPilot_CubeOrangePlus_...
Baud = 921600

[UartEndpoint air_esp32]
Device = /dev/ttyAMA0
Baud = 115200

[UdpEndpoint local_autonomy]
Mode = Normal
Address = 127.0.0.1
Port = 14552
```

### Step 4.2: Local Autonomy Engine (`onboard_pi/autonomy/`)
The autonomy program runs as a persistent service written in Python using `pymavlink`.

#### A. Protocol Ingest & Geofence Validator (`geofence.py`)
* Listen for binary packet `0xA1` received via the Air ESP32 link.
* Unpack parameters and run the **Ray-Casting Algorithm** to guarantee every calculated grid point is physically inside the polygon:
  $$\text{Inside} \iff \sum_{i=1}^{N} \text{intersects} \pmod 2 \equiv 1$$
* If any point lies outside or crosses an excluded zone, abort mission ingest and send a negative ACK (`0x15`).

#### B. Local Grid Planner (`grid_planner.py`)
* Mirror the GCS lawnmower path algorithm locally using the unpacked vertices and transect angle.
* Generate an ordered list of waypoints $[WP_1, WP_2, \dots, WP_k]$.

#### C. Autonomy State Machine (`main.py` & `fsm.py`)
Implement the states systematically:

1. **STATE_BOOT:**
   * Connect to `udpin:127.0.0.1:14552`.
   * Wait for MAVLink heartbeat (`HEARTBEAT`, type `MAV_TYPE_QUADROTOR`).
2. **STATE_IDLE:**
   * Monitor battery voltage (`SYS_STATUS`) and GPS fix (`GPS_RAW_INT`).
   * Await valid mission upload.
3. **STATE_PREARM:**
   * Verify ArduPilot pre-arm flags are zero.
   * Verify GPS fix $\ge 3$, satellites visible $\ge 10$, $\text{HDOP} \le 1.2$.
4. **STATE_TAKEOFF:**
   * Send `MAV_CMD_DO_SET_MODE` to `GUIDED` (mode 4).
   * Send `MAV_CMD_COMPONENT_ARM_DISARM` (`param1=1`).
   * Send `MAV_CMD_NAV_TAKEOFF` with target altitude $Z_{\text{takeoff}}$.
   * Block in monitoring loop until `relative_alt` reaches $\ge 0.95 \times Z_{\text{takeoff}}$.
5. **STATE_EXPLORATION:**
   * For each waypoint $WP_i$ in queue:
     * Emit `SET_POSITION_TARGET_GLOBAL_INT`:
       * `coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`
       * `type_mask = 0b0000111111111000` (Position valid; Velocity/Accel ignored)
       * `lat_int = WP_i.lat * 1e7`, `lon_int = WP_i.lon * 1e7`, `alt = WP_i.alt`
     * Convergence check loop (at 10 Hz):
       Calculate geodesic error $\Delta D$:
       $$\Delta D = \sqrt{(Lat_{\text{curr}} - Lat_{\text{target}})^2 + (Lon_{\text{curr}} - Lon_{\text{target}})^2} \times 111319.5$$
     * When $\Delta D \le 1.5\text{ m}$, hold position for configured survey dwell time (e.g., 2 seconds), then increment $i$.
6. **STATE_NAV_TO_LAND:**
   * Command `SET_POSITION_TARGET_GLOBAL_INT` pointing to designated landing marker.
   * Hold until $\Delta D \le 1.5\text{ m}$.
7. **STATE_LAND:**
   * Send `MAV_CMD_NAV_LAND`.
   * Confirm vehicle touchdown detection and auto-disarm (`base_mode & MAV_MODE_FLAG_SAFETY_ARMED == 0`).
   * Return to `STATE_IDLE`.

---

## Phase 5: Autopilot (Cube Orange+) Configuration

Set the following parameters in Mission Planner via Full Parameter Tree:

| Parameter | Value | Rationale |
| :--- | :--- | :--- |
| `SERIAL0_BAUD` | `921600` | High-rate communication over primary USB to Raspberry Pi. |
| `SERIAL2_PROTOCOL`| `2` | MAVLink 2 protocol on Telem 2 port. |
| `SERIAL2_BAUD` | `921600` | Baud rate for hardware serial companion link. |
| `FS_GCS_ENABLE` | `1` | Trigger Return-to-Launch (RTL) on telemetry link loss. |
| `FS_GCS_TIMEOUT`| `5.0` | 5 seconds timeout before failsafe activation. |
| `FS_THR_ENABLE` | `1` | Trigger RTL on RC link loss. |
| `FS_EKF_ACTION` | `1` | Transition to Land mode if EKF loses position certainty. |
| `BATT_FS_LOW_ACT`| `2` | Battery Low: Return to Launch. |
| `BATT_FS_CRT_ACT`| `1` | Battery Critical: Land immediately. |
| `FENCE_ENABLE` | `1` | Enable hardware boundary geofence. |
| `FENCE_TYPE` | `7` | Altitude + Circular + Inclusion Polygon fence. |
| `FENCE_ACTION` | `1` | RTL or loiter at fence boundary if breached. |
| `WP_YAW_BEHAVIOR`| `1` | Drone yaws toward the next target coordinate in Guided mode. |

---

## Phase 6: Virtual Validation via SITL (Software-in-the-Loop)

Execute all components virtually before touching hardware.

### Step 6.1: Launch ArduPilot SITL Instance
```bash
sim_vehicle.py -v ArduCopter -f quad --console --map
```
This initializes:
* MAVLink output on `127.0.0.1:14550` (Connect Mission Planner).
* MAVLink output on `127.0.0.1:14551` (Connect Autonomy Script / Router).

### Step 6.2: End-to-End Simulation Run
1. Start `ground_router.py` pointing to SITL UDP port `14551`.
2. Connect Mission Planner to `127.0.0.1:14550`.
3. Launch Ground Web UI in browser (`http://localhost:5173`).
4. Draw an arbitrary polygon boundary over the simulated location (e.g., Canberra Airport default).
5. Dispatch mission.
6. Verify the autonomy script arms SITL, takes off to cruise altitude, traverses each lawnmower waypoint in sequence on the SITL map, flies to the landing spot, and touches down.

---

## Phase 7: Benchtop Hardware-in-the-Loop Testing (Propellers Off)

**Mandatory Rule: REMOVE ALL PROPELLERS FROM MOTORS PRIOR TO BENCH TESTING.**

### Step 7.1: Physical Interface Verification
1. Connect Cube Orange+ to Pi 5 via USB CDC ACM cable.
2. Confirm persistent link:
   ```bash
   ls -l /dev/serial/by-id/usb-ArduPilot_CubeOrangePlus*
   ```
3. Start `mavlink-routerd` on the Pi and verify zero dropped packets in stats.

### Step 7.2: Motor & Attitude Response Verification
1. Move the drone near a window or set an EKF home origin to achieve 3D GPS lock.
2. Dispatch a mission via Ground Web UI over the physical ESP-NOW link.
3. Observe:
   * Autopilot mode shifts to `GUIDED`.
   * Motors arm and spool up to simulated takeoff RPM.
   * Autopilot advances through simulated waypoints; motors vary RPM as attitude compensation attempts to drive toward coordinates.

### Step 7.3: Safety Failsafe Verification
1. **Tier 4 Test (RC Override):** While motors are spinning in `GUIDED`, switch the physical RC transmitter switch from Position 3 (`Guided`) to Position 2 (`Loiter`).
   * *Pass Criteria:* Flight controller drops `GUIDED` instantly, ignoring script setpoints.
2. **Tier 3 Test (Companion Hang):** Force kill the Python script (`kill -9`) during guided navigation.
   * *Pass Criteria:* After exactly 3.0 seconds, the Cube Orange+ halts trajectory progression and enters a stationary Loiter hold.
3. **Tier 2 Test (Telemetry Drop):** Unplug the Ground ESP32 module.
   * *Pass Criteria:* After 5.0 seconds (`FS_GCS_TIMEOUT`), Cube Orange+ initiates RTL.

---

## Phase 8: Field Execution & Controlled Flight Protocol

Once all bench and SITL verification gates pass:

1. **Tethered / Low-Altitude Test (Field Step 1):**
   * Clear 20 m radius.
   * Command a $5\text{ m} \times 5\text{ m}$ square pattern at $3\text{ m}$ altitude.
   * Verify waypoint arrival accuracy, loiter time, and heading alignment.
2. **Full Survey Deployment (Field Step 2):**
   * Upload an operational geofenced perimeter.
   * Initiate automated takeoff, survey line coverage, transit to landing coordinate, and touchdown.
   * Keep pilot fingers on the RC flight mode switch at all times for manual takeover authority.