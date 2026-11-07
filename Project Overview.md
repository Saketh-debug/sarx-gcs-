# **Autonomous Companion Drone Architecture & Implementation Plan**

## **1\. Premise & Core Objectives**

### **The Premise**

Standard autonomous multirotor flights rely on ArduPilot's native **AUTO Mode**, where static waypoint coordinates are pre-compiled and written directly into the flight controller's internal EEPROM. While reliable for pre-planned mapping, this architecture is rigid: the flight controller cannot dynamically alter its flight path based on real-time computer vision, onboard sensor inputs, or runtime boundary modifications without pausing the mission and re-flashing waypoint lists over telemetry.

### **The Objective**

To build a companion-computer-driven autonomous system where a **Raspberry Pi 5** acts as the high-level navigator and mission brain, commanding a **Cube Orange+** flight controller in real time via ArduPilot's **GUIDED Mode**.

The system achieves:

1. **Interactive Mission Dispatch:** An operator defines a geofenced search boundary, automated search grid parameters, and a landing coordinate via an interactive ground web interface displaying the drone's live GPS origin.  
2. **Bandwidth-Optimized Bridge:** Mission parameters are compressed to fit within the strict 250-byte transmission limit of an **ESP-NOW Long Range (LR)** point-to-point wireless bridge.  
3. **Sequential Target Execution:** The Raspberry Pi unpacks the mission, validates geofence constraints, executes an autonomous Guided takeoff, and streams discrete position setpoints one by one to the Cube Orange+, tracking completion criteria before advancing.  
4. **Dual-GCS Multiplexing:** Mission Planner runs concurrently with the custom Web UI over the same physical ESP-NOW link for monitoring, HUD tracking, and telemetry backup.  
5. **Fail-Safe Operation:** A multi-layered safety architecture ensures hardware, communication, or software failures trigger deterministic emergency responses, with instant manual RC takeover at any time.

## **2\. System Architecture & Topology**

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                                  GROUND CONTROL SYSTEM                                  │
│                                                                                         │
│   ┌───────────────────────────────┐           ┌──────────────────────────────────────┐  │
│   │   Custom Ground Web UI        │           │     Mission Planner (GCS)            │  │
│   │   (React / Leaflet Map)       │           │     (Full Telemetry & Safety HUD)    │  │
│   └───────────────┬───────────────┘           └──────────────────┬───────────────────┘  │
│                   │ HTTP / WebSocket (JSON)                      │ UDP :14550           │
│                   ▼                                              ▼                      │
│   ┌──────────────────────────────────────────────────────────────────────────────────┐  │
│   │               Ground GCS Router Daemon (Python / MAVProxy)                       │  │
│   │   • Splits telemetry to Mission Planner (UDP 14550) & Web UI (WebSocket 8080)   │  │
│   │   • Encapsulates mission JSON into binary frames (<= 200 Bytes)                  │  │
│   └───────────────────────────────────────┬──────────────────────────────────────────┘  │
│                                           │ USB Virtual COM / UART (57600 baud)         │
│                                           ▼                                             │
│                       ┌───────────────────────────────────────┐                         │
│                       │   Ground ESP32 (ESP-NOW LR Master)    │                         │
│                       └───────────────────┬───────────────────┘                         │
└───────────────────────────────────────────┼─────────────────────────────────────────────┘
                                            │
                                            │ 2.4 GHz ESP-NOW Wireless Bridge
                                            │ (Raw Frame MTU: 250 Bytes)
                                            │
┌───────────────────────────────────────────┼─────────────────────────────────────────────┐
│                                           ▼                                             │
│                       ┌───────────────────────────────────────┐                         │
│                       │     Air ESP32 (ESP-NOW LR Slave)      │                         │
│                       └───────────────────┬───────────────────┘                         │
│                                           │ UART Serial (57600 / 115200 baud)           │
│                                           ▼                                             │
│   ┌──────────────────────────────────────────────────────────────────────────────────┐  │
│   │                         Raspberry Pi 5 (Companion Computer)                      │  │
│   │                                                                                  │  │
│   │   ┌─────────────────────────┐              ┌─────────────────────────────────┐   │  │
│   │   │  ESP-NOW Serial Ingest  │ ──Decode──►  │    Autonomy State Machine       │   │  │
│   │   │  & Forwarding Proxy     │              │    • Mission boundary validator │   │  │
│   │   └────────────┬────────────┘              │    • Grid sequence controller   │   │  │
│   │                │                           │    • Vision / payload hook      │   │  │
│   │                │ Raw MAVLink               └────────────────┬────────────────┘   │  │
│   │                ▼                                            │ pymavlink          │  │
│   │   ┌─────────────────────────┐                               │ Setpoints          │  │
│   │   │ MAVLink Multiplexer     │ ◄─────────────────────────────┘                    │  │
│   │   │ (mavlink-router daemon) │                                                    │  │
│   │   └────────────┬────────────┘                                                    │  │
│   └────────────────┼─────────────────────────────────────────────────────────────────┘  │
│                    │ USB CDC ACM (/dev/serial/by-id/...) or Telem UART @ 921600 baud     │
│                    ▼                                                                    │
│   ┌──────────────────────────────────────────────────────────────────────────────────┐  │
│   │                     Cube Orange+ Autopilot (ArduPilot Copter)                    │  │
│   │                                                                                  │  │
│   │   • EKF3 State Estimation (GPS, Compass, Barometer, IMU)                         │  │
│   │   • Flight Mode: GUIDED                                                          │  │
│   │   • Position Target Controller (PIDs, Trajectory Shaping, Velocity Clamping)     │  │
│   │   • Motor Mixing & Actuation Outputs (DShot / PWM to ESCs)                       │  │
│   └──────────────────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

## **3\. Operational Plan & Execution Logic**

### **3.1 Division of Responsibilities**

| Subsystem | Primary Responsibilities | Execution Boundaries |
| ----- | ----- | ----- |
| **Ground Web UI** | • Renders live vehicle position. • Collects boundary polygon coordinates & landing marker. • Computes survey lines for visual inspection. • Packs compact binary parameters. | Does **not** control flight motors; acts strictly as an input and visualization layer. |
| **Ground Python Router** | • Reads ground ESP32 serial data. • Forwards telemetry to Mission Planner (UDP) and Web UI (WebSocket). • Transmits validated mission packets to the Air ESP32. | Transparent routing; does not compute flight paths. |
| **Raspberry Pi 5** | • Runs autonomy state machine. • Computes discrete grid coordinates locally. • Monitors waypoint acceptance radius (R≤1.5 m). • Streams sequential targets via `SET_POSITION_TARGET_GLOBAL_INT`. • Hosts vision/payload triggers. | Relies entirely on Cube Orange+ for real-time motor stabilization and attitude balance. |
| **Cube Orange+** | • Executes high-rate PID loops (400 Hz). • Runs EKF3 sensor fusion (100 Hz). • Enforces internal hardware safety, geofencing, and descent slopes. | Blind to future waypoints; navigates only toward the active target setpoint. |

### **3.2 The Onboard Autonomy State Machine**

The Python autonomy daemon running on the Raspberry Pi 5 operates as a deterministic finite-state machine (FSM):

```
                   ┌──────────────┐
                   │  01. BOOT    │
                   └──────┬───────┘
                          │ MAVLink Connected & Heartbeat Detected
                          ▼
                   ┌──────────────┐
                   │  02. IDLE    │ ◄───────────────────────────────┐
                   └──────┬───────┘                                 │
                          │ Mission Packet Received from GCS        │
                          ▼                                         │
               ┌───────────────────────┐                            │
               │  03. PRE-ARM CHECKS   │                            │
               └──────────┬────────────┘                            │
                          │ Checks Passed (GPS 3D Fix, Voltage OK)  │
                          ▼                                         │
               ┌───────────────────────┐                            │
               │  04. GUIDED TAKEOFF   │                            │
               └──────────┬────────────┘                            │
                          │ Target Altitude Achieved (Alt >= 95%)   │
                          ▼                                         │
               ┌───────────────────────┐                            │
               │  05. EXPLORATION LOOP │                            │
               └──────────┬────────────┘                            │
                          │ All Grid Targets Exhausted              │
                          ▼                                         │
               ┌───────────────────────┐                            │
               │  06. NAVIGATE TO LAND │                            │
               └──────────┬────────────┘                            │
                          │ Arrived at Landing Site (< 1.5m)        │
                          ▼                                         │
               ┌───────────────────────┐                            │
               │  07. ARDUPILOT LAND   │                            │
               └──────────┬────────────┘                            │
                          │ Touchdown Detected & Motors Disarmed    │
                          └─────────────────────────────────────────┘
```

#### **Detailed State Actions**

1. **BOOT & INITIALIZATION:** Establish connection to `mavlink-router` endpoint (`udpin:127.0.0.1:14550`). Confirm MAVLink heartbeat from `System 1, Component 1` (Cube Orange).  
2. **IDLE:** Stream `GLOBAL_POSITION_INT` coordinates back to the GCS over ESP-NOW. Wait for mission configuration.  
3. **PRE-ARM CHECKS:**  
   * Verify GPS lock status (`fix_type >= 3`, satellites visible ≥10\\ge 10≥10, HDOP ≤1.2\\le 1.2≤1.2).  
   * Confirm ArduPilot reports zero pre-arm warnings.  
   * Verify all computed exploration waypoints lie strictly inside the boundary polygon using point-in-polygon tests (e.g., Ray-Casting algorithm).  
4. **GUIDED TAKEOFF:**  
   * Send `MAV_CMD_DO_SET_MODE` targeting ArduPilot Mode `GUIDED` (Mode 4).  
   * Send `MAV_CMD_COMPONENT_ARM_DISARM` (`param1 = 1`).  
   * Send `MAV_CMD_NAV_TAKEOFF` with target altitude ZtakeoffZ\_{takeoff}Ztakeoff​ (e.g., 15 m15\\text{ m}15 m).  
   * Poll `GLOBAL_POSITION_INT.relative_alt` until current altitude ≥0.95×Ztakeoff\\ge 0.95 \\times Z\_{takeoff}≥0.95×Ztakeoff​.  
5. **EXPLORATION LOOP (Sequential Feeding):**  
   * Fetch waypoint WPiWP\_iWPi​ from the mission queue.  
   * Issue `SET_POSITION_TARGET_GLOBAL_INT`:  
     * `coordinate_frame`: `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`  
     * `type_mask`: `0b0000111111111000` (Position enabled; velocity/accel ignored).  
     * `lat_int`: Lat×107\\text{Lat} \\times 10^7Lat×107  
     * `lon_int`: Lon×107\\text{Lon} \\times 10^7Lon×107  
     * `alt`: Target altitude in meters.  
   * Continuous convergence loop:  
     ΔDistance=(Latcurrent−Lattarget)2+(Loncurrent−Lontarget)2×111319.5\\Delta \\text{Distance} \= \\sqrt{(Lat\_{current} \- Lat\_{target})^2 \+ (Lon\_{current} \- Lon\_{target})^2} \\times 111319.5ΔDistance=(Latcurrent​−Lattarget​)2+(Loncurrent​−Lontarget​)2![]()​×111319.5  
   * Once ΔDistance≤1.5 m\\Delta \\text{Distance} \\le 1.5\\text{ m}ΔDistance≤1.5 m, pause for camera inspection/loiter delay, then advance i=i+1i \= i \+ 1i=i+1.  
6. **NAVIGATE TO LAND:** Send `SET_POSITION_TARGET_GLOBAL_INT` pointing to designated landing coordinates. Hold until arrival verification (≤1.5 m\\le 1.5\\text{ m}≤1.5 m).  
7. **ARDUPILOT LAND:** Send `MAV_CMD_NAV_LAND`. ArduPilot's internal land detector takes over descent, slows down near the ground, detects zero vertical velocity change on touchdown, and disarms motors automatically.

### **3.3 ESP-NOW Transmission Specification**

ESP-NOW imposes a hard packet limit of **250 bytes**. To guarantee zero packet fragmentation and complete compatibility with the serial bridge, mission parameters must be packed into a binary struct rather than bulky plain text or JSON strings.

#### **Mission Upload Binary Packet (Max 148 Bytes)**

```
[ Byte 0 ]     : Message Type ID (0xA1 = Mission Upload)
[ Byte 1 ]     : Polygon Vertex Count N (3 to 8 vertices)
[ Bytes 2-33 ] : Up to 8 vertices (Lat int32, Lon int32 -> 8 bytes per vertex = 64 bytes)
[ Bytes 34-41 ]: Landing Point (Lat int32, Lon int32 = 8 bytes)
[ Byte 42 ]    : Cruise Altitude in meters (uint8 = 1 byte)
[ Byte 43 ]    : Survey Speed in m/s (uint8 = 1 byte)
[ Byte 44 ]    : Lane Spacing in meters (uint8 = 1 byte)
[ Byte 45 ]    : Grid Transect Angle in degrees (uint8 = 1 byte)
[ Bytes 46-47 ]: CRC16 Checksum (2 bytes)
```

The Raspberry Pi unpacks these 48 bytes, recalculates the identical boustrophedon (lawnmower) path locally, and emits a single 4-byte ACK packet containing the generated waypoint count and verification checksum back to the GCS.

## **4\. Comprehensive Multi-Tier Failsafe Matrix**

The safety architecture contains zero single-points-of-failure. Every level of the flight stack enforces an independent safety loop:

```
┌────────────────────────────────────────────────────────────────────────┐
│                        COMPREHENSIVE FAILSAFE MATRIX                   │
├─────────┬──────────────────────────┬───────────────────┬───────────────┤
│ Tier    │ Trigger Condition        │ Detection Layer   │ Auto Action   │
├─────────┼──────────────────────────┼───────────────────┼───────────────┤
│ Tier 1: │ Battery Voltage Low      │ Cube Orange+      │ RTL to Home   │
│ Hard-   │ (e.g., < 14.4V / 3.6Vpc) │ Power Module      │               │
│ ware    ├──────────────────────────┼───────────────────┼───────────────┤
│         │ Battery Critical         │ Cube Orange+      │ Emergency     │
│         │ (e.g., < 13.8V / 3.45Vpc)│ Power Module      │ Immediate Land│
│         ├──────────────────────────┼───────────────────┼───────────────┤
│         │ GPS Loss / EKF Glitch    │ Cube Orange+      │ Fallback to   │
│         │ (Compass/Sat fail)       │ EKF3 Core         │ Land / AltHold│
│         ├──────────────────────────┼───────────────────┼───────────────┤
│         │ Hardware Geofence Breach │ Cube Orange+      │ Stop at fence │
│         │ (Ceiling or Perimeter)   │ FENCE Engine      │ or RTL        │
├─────────┼──────────────────────────┼───────────────────┼───────────────┤
│ Tier 2: │ ESP-NOW Telemetry Link   │ Cube Orange+      │ RTL or        │
│ Comms   │ Loss (> 5 seconds)       │ (FS_GCS_ENABLE)   │ Continue *    │
│ Link    ├──────────────────────────┼───────────────────┼───────────────┤
│         │ RC Transmitter Loss      │ FlySky Receiver / │ Trigger RTL   │
│         │ (Out of range / Off)     │ ArduPilot RC Fails│ immediately   │
├─────────┼──────────────────────────┼───────────────────┼───────────────┤
│ Tier 3: │ Autonomy Script Crash /  │ Cube Orange+      │ Stationary    │
│ Compan- │ Pi Hang (> 3s no target) │ Guided Watchdog   │ Loiter Hold   │
│ ion Pi  ├──────────────────────────┼───────────────────┼───────────────┤
│         │ Waypoint Boundary Breach │ Pi Autonomy       │ Abort Mission │
│         │ (Target outside polygon) │ Validation Engine │ & RTL         │
├─────────┼──────────────────────────┼───────────────────┼───────────────┤
│ Tier 4: │ Physical Switch Toggle   │ Pilot on FlySky   │ Instant Manual│
│ Pilot   │ (Switch out of Guided)   │ Transmitter       │ Authority     │
└─────────┴──────────────────────────┴───────────────────┴───────────────┘
```

*\* Configured according to operational constraints: set `FS_GCS_ENABLE = 1` (RTL) for line-of-sight testing, or `FS_GCS_ENABLE = 0` if relying entirely on onboard companion autonomy.*

### **Failsafe Implementation Specifics**

1. **RC Override Priority (Tier 4):**  
   * The physical flight mode switch on the transmitter (FlySky FS-i6S) is mapped via ArduPilot's `FLTMODE_CH`.  
   * Switch positions: `Position 1 = AltHold`, `Position 2 = Loiter`, `Position 3 = Guided`.  
   * **The hardware rule:** Moving the flight mode switch to **Loiter** or **AltHold** instantly revokes Guided mode. ArduPilot strips all control authority from the companion computer and grants exclusive command to the physical transmitter sticks within a single clock cycle (2.5 ms2.5\\text{ ms}2.5 ms).  
2. **ArduPilot Guided Watchdog (Tier 3):**  
   * ArduPilot features an internal timeout for companion-directed flight. When in Guided mode, if no new velocity setpoint or position target is received for **3.0 seconds**, ArduPilot ceases forward translation and enters a **stationary Loiter hold**. It will hover in place indefinitely until a new command arrives, battery failsafe triggers, or the pilot takes manual control.  
3. **Geofencing Containment (Tier 1):**  
   * An ArduPilot hardware fence is stored directly in the Cube Orange+ memory (`FENCE_TYPE = 7` for Altitude \+ Circle \+ Inclusion Polygon).  
   * Even if a software bug on the Pi outputs an erroneous target coordinate outside the designated zone, the flight controller's internal collision-prevention engine rejects the command before motor outputs are modified.

## **5\. Required ArduPilot (Cube Orange+) Parameter Configuration**

Configure these parameters in Mission Planner before initiating companion-guided flight:

| Parameter | Recommended Value | Description |
| ----- | ----- | ----- |
| `SERIAL0_BAUD` | `115200` or `921600` | USB connection baud rate between Cube Orange+ and Pi 5\. |
| `SERIAL2_PROTOCOL` | `2` | MAVLink 2 on Telem 2 (alternative hardware UART link). |
| `SERIAL2_BAUD` | `921600` | High-speed baud rate for companion communications. |
| `FS_GCS_ENABLE` | `1` | Enables GCS failsafe (triggers RTL if telemetry link drops). |
| `FS_GCS_TIMEOUT` | `5.0` | Seconds of telemetry silence before GCS failsafe initiates. |
| `FS_THR_ENABLE` | `1` | Enables RC throttle failsafe (triggers RTL on transmitter signal loss). |
| `FS_EKF_ACTION` | `1` | Switches to Land mode if EKF loses position certainty. |
| `BATT_FS_LOW_ACT` | `2` | Battery low action: Return to Launch (RTL). |
| `BATT_FS_CRT_ACT` | `1` | Battery critical action: Land immediately in place. |
| `DISARM_DELAY` | `10` | Auto-disarm 10 seconds after touchdown detection. |
| `WP_YAW_BEHAVIOR` | `1` | Face next waypoint during Guided navigation. |

## **6\. Implementation & Validation Roadmap**

### **Phase 1: Bench Simulation & MAVLink Multiplexing (Hardware on Desk)**

1. Mount the Raspberry Pi 5 to the Cube Orange+ via USB CDC ACM. Verify the interface connects persistently under `/dev/serial/by-id/`.  
2. Configure `mavlink-router` on the Pi to multiplex the incoming MAVLink stream between the local autonomy Python process (`udp:127.0.0.1:14550`) and the Air ESP32 UART interface.  
3. Establish communication between Ground ESP32 and Air ESP32 in ESP-NOW Long Range mode. Open Mission Planner on the ground laptop, connect via UDP, and verify live parameter loading and attitude horizon updates.

### **Phase 2: Ground UI & Protocol Verification (Propellers Off)**

1. Launch the local Ground Web UI with the Leaflet map component. Confirm the live position marker aligns with the GPS coordinates streamed from the Cube Orange+.  
2. Draw a test polygon boundary. Trigger packet generation and verify via terminal logs that the binary payload is ≤200 bytes\\le 200\\text{ bytes}≤200 bytes.  
3. Transmit the packet across the ESP-NOW bridge. Confirm the Pi 5 parses the coordinates, validates the bounding box, and computes the correct waypoint grid array.

### **Phase 3: Hardware Guided Bench Test (Propellers Off)**

1. Arm the vehicle indoors (or with GPS spoofing/EKF origin set in SITL) with **propellers removed**.  
2. Run the Python autonomy script to command Guided Mode and trigger Takeoff. Verify the Cube Orange+ changes flight mode to `GUIDED` and motors spool up to simulated takeoff throttle.  
3. Feed simulated GPS target waypoints. Observe motor speed variations as the autopilot attempts to steer toward the targets.  
4. Flip the RC transmitter flight mode switch to **Loiter** mid-test. Verify that Guided mode immediately disengages.

### **Phase 4: Controlled Flight Field Testing (In-Flight)**

1. **Test 1 \- Box Maneuver:** Command a small 5 m×5 m5\\text{ m} \\times 5\\text{ m}5 m×5 m square at 5 m5\\text{ m}5 m altitude in an open field via the Pi script. Verify waypoint acceptance radius and arrival loiter times.  
2. **Test 2 \- Failsafe Verification:** Disconnect the ground telemetry bridge during flight. Verify that the Cube Orange+ transitions to RTL after the 5-second timeout.  
3. **Test 3 \- Full Survey Deployment:** Draw an operational boundary on the Ground Web UI, transmit the mission over ESP-NOW, and verify hands-off execution: Guided Takeoff →\\rightarrow→ Boundary Lawn-mower Search →\\rightarrow→ Transit to Landing Coordinate →\\rightarrow→ Autonomous Touchdown & Disarm.

