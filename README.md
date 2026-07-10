# RealSense D4xx → PX4 Collision Prevention

> Standalone Python script that reads depth frames from an Intel RealSense D435/D435i camera and sends `OBSTACLE_DISTANCE` MAVLink messages to a **PX4** flight controller for real-time collision prevention.
>
> Adapted from [ArduPilot's d4xx_to_mavlink.py](https://discuss.ardupilot.org/t/gsoc-2020-integration-of-ardupilot-and-realsense-d4xx/57579) — rewritten to use `pymavlink` instead of `dronekit` for full PX4 compatibility.

---

## How It Works

```
RealSense D435i
   └─ USB 3.0 ──► Companion Computer (Jetson / RPi / UP2)
                      └─ runs d4xx_to_px4.py
                            │
                            │ depth frame (640×480 @ 30fps)
                            ▼
                      Post-processing filters
                      (decimation → threshold → spatial → temporal)
                            │
                            │ 72-bin distance array (cm)
                            ▼
                      OBSTACLE_DISTANCE MAVLink msg (#330)
                            │
                            │ UART @ 921600 baud
                            ▼
                      Pixhawk (PX4 Firmware)
                      └─ Collision Prevention (CP_DIST)
                            └─ brakes drone before hitting obstacle
```

The camera's **87° horizontal FOV** is divided into **72 sectors** (5° each). For each sector, the minimum depth reading is extracted from the middle row of the depth image and sent to PX4 at **15 Hz**.

---

## Hardware Requirements

| Component | Specification | Notes |
|---|---|---|
| **Camera** | Intel RealSense D435 or D435i | D435i has built-in IMU |
| **Companion Computer** | Jetson Orin / RPi 4 / UP Squared | Needs USB 3.0 port |
| **Flight Controller** | Any Pixhawk running PX4 firmware | Tested on Pixhawk 6C |
| **Serial Link** | FTDI USB-UART adapter or Pixhawk TELEM2 | 921600 baud |
| **Cable** | USB 3.0 Type-A to Type-C | Camera to companion |
| **Power** | Companion computer powered separately | Not from USB |

### Wiring Diagram

```
RealSense D435i
   USB 3.0
      │
      ▼
Companion Computer
   (e.g. Jetson Orin Nano)
   UART TX ──────────────────► Pixhawk TELEM2 RX
   UART RX ◄────────────────── Pixhawk TELEM2 TX
   GND ──────────────────────── Pixhawk GND
   (NO 5V — power companion separately)
```

> ⚠️ **Do NOT power the companion computer from the Pixhawk's 5V rail.** Use a dedicated BEC or power module rated for your companion board's current draw.

---

## Software Requirements

| Software | Version | Install |
|---|---|---|
| Ubuntu | 20.04 / 22.04 LTS | — |
| Python | 3.8+ | `sudo apt install python3` |
| librealsense2 | ≥ 2.54 | [Intel install guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) |
| pyrealsense2 | ≥ 2.54 | `pip install pyrealsense2` |
| pymavlink | ≥ 2.4.41 | `pip install pymavlink` |
| numpy | ≥ 1.24 | `pip install numpy` |
| PX4 Firmware | v1.13+ | Collision Prevention stable from v1.13 |

---

## Installation

```bash
# 1. Clone this repo
git clone https://github.com/jenarohi/px4-realsense-avoidance.git
cd px4-realsense-avoidance

# 2. Install Python dependencies
pip3 install -r requirements.txt

# 3. (Optional) Add udev rules for RealSense USB permissions
sudo cp 99-realsense.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## PX4 Parameter Setup

Set these in **QGroundControl → Parameters**:

### Serial Port (TELEM2)

| Parameter | Value | Purpose |
|---|---|---|
| `MAV_1_CONFIG` | 102 | Route MAVLink instance 1 to TELEM2 |
| `MAV_1_MODE` | 2 | Onboard companion mode |
| `SER_TEL2_BAUD` | 921600 | Must match script baud rate |

### Collision Prevention

| Parameter | Recommended Value | Purpose |
|---|---|---|
| `CP_DIST` | `3.0` (metres) | **Enables** collision prevention. Drone stops at this distance. |
| `CP_DELAY` | `0.5` | Sensor delay compensation (seconds) |
| `CP_GUIDE_ANG` | `30` | Angle (°) to try guiding around obstacle. `0` = just stop. |

> ⚠️ Collision Prevention **only works in Multicopter Position mode** (POSCTL). It does not work in Altitude, Stabilised, or Offboard mode.

---

## ArduPilot vs PX4 — Parameter Comparison

| Purpose | ArduPilot Parameter | PX4 Parameter |
|---|---|---|
| Enable obstacle avoidance | `AVOID_ENABLE = 7` | `CP_DIST = 3.0` (any value > 0) |
| Proximity sensor type | `PRX_TYPE = 2` (MAVLink) | *(automatic — PX4 reads OBSTACLE_DISTANCE natively)* |
| Stop/brake distance | `AVOID_MARGIN = 1.5` | `CP_DIST = 3.0` |
| Avoidance behaviour | `AVOID_BEHAVE = 1` (stop) | `CP_GUIDE_ANG = 0` (stop) |
| Slide around obstacle | `AVOID_BEHAVE = 0` (slide) | `CP_GUIDE_ANG = 30` (guide angle) |
| UART port | `SERIAL5_PROTOCOL = 2` | `MAV_1_CONFIG = 102` |
| UART baud | `SERIAL5_BAUD = 921` | `SER_TEL2_BAUD = 921600` |
| Min distance alert | `AVOID_DIST_MAX` | `CP_DIST` (same param) |
| Companion library | `dronekit` | **`pymavlink`** (dronekit is ArduPilot-only) |
| MAVLink message | `OBSTACLE_DISTANCE` (#330) | `OBSTACLE_DISTANCE` (#330) — **same message** |

---

## Usage

### Basic (serial connection to Pixhawk)
```bash
python3 d4xx_to_px4.py
```

### Custom port / baud
```bash
python3 d4xx_to_px4.py --connect /dev/ttyUSB0 --baud 921600
```

### SITL testing (software in the loop — no real Pixhawk needed)
```bash
# Start PX4 SITL in another terminal first
python3 d4xx_to_px4.py --connect udp:127.0.0.1:14550
```

### Debug mode (print distance readings)
```bash
python3 d4xx_to_px4.py --debug
```

---

## Implementing on a Drone — Step by Step

### Step 1 — Mount the camera
- Mount the RealSense D435i on the **front** of the drone, facing forward.
- Tilt: **0° (horizontal)**. Keep the camera level with the horizon.
- Ensure the camera FOV (87° horizontal) is **not blocked** by arms or props.

### Step 2 — Wire it up
- USB 3.0: Camera → Companion Computer
- UART: Companion TX → Pixhawk TELEM2 RX, Companion RX → TELEM2 TX, GND → GND

### Step 3 — Set PX4 parameters
In QGroundControl, set all parameters from the table above. **Reboot** after saving.

### Step 4 — Ground test
```bash
# Terminal 1: run the script
python3 d4xx_to_px4.py --debug

# Check in QGroundControl:
# Widgets → MAVLink Inspector → search OBSTACLE_DISTANCE
# You should see the message arriving at ~15 Hz
```

### Step 5 — Verify in QGroundControl
- Open **Widgets → MAVLink Inspector**
- Search for `OBSTACLE_DISTANCE` — confirm it's being received
- Move an object in front of the camera — values should change

### Step 6 — Flight test
1. Arm in **Position mode (POSCTL)**
2. Take off and hover at ~1.5m
3. Slowly fly toward a wall
4. The drone should **stop** at `CP_DIST` metres from the wall
5. Try pushing the stick harder — drone should hold its position

> 🛡️ Always have a **safety pilot** with RC override ready during first tests.

---

## Troubleshooting

| Problem | Likely Cause | Fix |
|---|---|---|
| `No depth frame received` | Camera not connected or USB 2.0 | Use USB 3.0 port |
| `Heartbeat timeout` | Wrong port / baud / not connected | Check cable and port name |
| Drone doesn't stop | `CP_DIST = 0` (disabled) | Set `CP_DIST > 0` |
| Drone stops too early | `CP_DIST` too large | Reduce `CP_DIST` |
| Message not received in QGC | Wrong TELEM port or baud | Verify `MAV_1_CONFIG` and `SER_TEL2_BAUD` |
| Camera permission denied | Missing udev rules | Apply `99-realsense.rules` |

---

## Differences from the ArduPilot Article

| Aspect | ArduPilot Article | This Project |
|---|---|---|
| Flight controller | ArduPilot (Copter) | **PX4** |
| Companion library | `dronekit` | **`pymavlink`** |
| MAVLink message | OBSTACLE_DISTANCE | OBSTACLE_DISTANCE (same) |
| Parameter set | AVOID_*, PRX_* | **CP_DIST, CP_DELAY** |
| Flight mode | Loiter / AltHold | **Position (POSCTL)** |
| ROS required | No | No |

---

## License

MIT License. Based on original work by [Thien Nguyen (LuckyBird)](https://discuss.ardupilot.org/t/gsoc-2020-integration-of-ardupilot-and-realsense-d4xx/57579) under ArduPilot GSoC 2020.
