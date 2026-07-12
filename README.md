# RealSense D4xx → PX4 Collision Prevention

Standalone Python script that reads depth frames from an Intel RealSense D435/D435i camera and sends `OBSTACLE_DISTANCE` MAVLink messages to a PX4 flight controller for real-time collision prevention.

---

## How It Works

```
RealSense D435 / D435i
   └─ USB 3.0 ──► Companion Computer (Jetson / RPi / UP2)
                      └─ runs d4xx_to_px4.py
                            │
                            │ depth frame (640×480 @ 30 fps)
                            ▼
                      Post-processing filter chain
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

The camera's 87° horizontal FOV is divided into 72 sectors (5° each).
For each sector, the minimum depth reading is taken from the middle row of the depth image and sent to PX4 at 15 Hz.

---

## Hardware Requirements

| Component | Specification | Notes |
|---|---|---|
| Camera | Intel RealSense D435 or D435i | USB 3.0 required |
| Companion Computer | Jetson Orin / RPi 4  | Needs USB 3.0 port |
| Flight Controller | Pixhawk running PX4 firmware | Tested on Pixhawk 6C |
| Serial Link | FTDI USB-UART adapter | TELEM2 port, 921600 baud |
| Cable | USB 3.0 Type-A to Type-C | Camera to companion |

---

## Wiring

```
RealSense D435 / D435i
   USB 3.0
      │
      ▼
Companion Computer (Jetson Orin Nano)
   UART TX ──────────────────► Pixhawk TELEM2 RX
   UART RX ◄────────────────── Pixhawk TELEM2 TX
   GND      ──────────────────── Pixhawk GND
```

> **Note:** Do NOT power the companion computer from the Pixhawk's 5V rail. Use a dedicated BEC or power module.

---

## Software Requirements

| Software | Version |
|---|---|
| Ubuntu | 20.04 / 22.04 LTS |
| Python | 3.8+ |
| librealsense2 | ≥ 2.54 |
| pyrealsense2 | ≥ 2.54 |
| pymavlink | ≥ 2.4.41 |
| numpy | ≥ 1.24 |
| PX4 Firmware | v1.13+ |

---

## Installation

```bash
# 1. Clone this repo
git clone https://github.com/jenarohi/px4-realsense-avoidance.git
cd px4-realsense-avoidance

# 2. Install Python dependencies
pip3 install -r requirements.txt

# 3. Install udev rules for RealSense USB permissions
sudo cp 99-realsense.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## PX4 Parameter Setup

Set these in **QGroundControl → Parameters**:

### Serial Port (TELEM2)

| Parameter | Value | Purpose |
|---|---|---|
| `MAV_1_CONFIG` | `102` | Route MAVLink instance 1 to TELEM2 |
| `MAV_1_MODE` | `2` | Onboard companion mode |
| `SER_TEL2_BAUD` | `921600` | Must match script baud rate |

### Collision Prevention

| Parameter | Recommended Value | Purpose |
|---|---|---|
| `CP_DIST` | `3.0` (metres) | Enables collision prevention. Drone stops at this distance. |
| `CP_DELAY` | `0.5` | Sensor delay compensation (seconds) |
| `CP_GUIDE_ANG` | `30` | Angle (°) to guide around obstacle. 0 = stop only. |

> **Collision Prevention only works in Multicopter Position mode (POSCTL).** It does not work in Altitude, Stabilised, or Offboard mode.

---

## ArduPilot vs PX4 Parameter Comparison

| Purpose | ArduPilot Parameter | PX4 Parameter |
|---|---|---|
| Enable obstacle avoidance | `AVOID_ENABLE = 7` | `CP_DIST = 3.0` (any value > 0) |
| Stop/brake distance | `AVOID_MARGIN = 1.5` | `CP_DIST = 3.0` |
| Avoidance behaviour — stop | `AVOID_BEHAVE = 1` | `CP_GUIDE_ANG = 0` |
| Avoidance behaviour — slide | `AVOID_BEHAVE = 0` | `CP_GUIDE_ANG = 30` |
| UART port | `SERIAL5_PROTOCOL = 2` | `MAV_1_CONFIG = 102` |
| UART baud | `SERIAL5_BAUD = 921` | `SER_TEL2_BAUD = 921600` |
| MAVLink message | `OBSTACLE_DISTANCE (#330)` | `OBSTACLE_DISTANCE (#330)` |

---

## Usage

```bash
# Basic (default: /dev/ttyUSB0 @ 921600 baud)
python3 d4xx_to_px4.py

# Custom port / baud
python3 d4xx_to_px4.py --connect /dev/ttyUSB0 --baud 921600


# Debug mode (print live distance readings)
python3 d4xx_to_px4.py --debug
```

---

## Ground Test Procedure

```bash
# Terminal 1 — run the script
python3 d4xx_to_px4.py 

# In QGroundControl:
# Widgets → MAVLink Inspector → search OBSTACLE_DISTANCE
# Confirm it arrives at ~15 Hz
# Move an object in front of the camera — values should update
```

---

## Flight Test Procedure

1. Arm in **Position mode (POSCTL)**
2. Take off and hover at ~1.5 m
3. Slowly fly toward a wall
4. Drone should stop at `CP_DIST` metres from the wall
5. Push stick harder — drone should hold its position

> Always have a safety pilot with RC override ready during first tests.

---

## Troubleshooting

| Problem | Likely Cause | Fix |
|---|---|---|
| No depth frame | Camera not connected or USB 2.0 port | Use USB 3.0 (blue) port |
| Heartbeat timeout | Wrong port / baud / cable | Check `--connect` and `--baud` args |
| Drone doesn't stop | `CP_DIST = 0` (disabled) | Set `CP_DIST > 0` in QGroundControl |
| Drone stops too early | `CP_DIST` too large | Reduce `CP_DIST` |
| Message not in QGC | Wrong TELEM port config | Verify `MAV_1_CONFIG` and `SER_TEL2_BAUD` |
| Camera permission denied | Missing udev rules | Run `sudo cp 99-realsense.rules /etc/udev/rules.d/` |

---

## License

MIT License.
