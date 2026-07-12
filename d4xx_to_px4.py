"""
RealSense D4xx → PX4 Obstacle Avoidance
========================================
Converts RealSense D435/D435i depth frames into OBSTACLE_DISTANCE
MAVLink messages consumed by PX4's Collision Prevention module.

Usage:
    python3 d4xx_to_px4.py
    python3 d4xx_to_px4.py --connect /dev/ttyUSB0 --baud 921600
    python3 d4xx_to_px4.py --connect udp:127.0.0.1:14550  # SITL testing

Hardware:
    Companion computer (Jetson / RPi / UP2)
      ├─ USB 3.0 ──► RealSense D435 / D435i
      └─ UART ─────► Pixhawk TELEM2 @ 921600 baud

PX4 parameters to set (QGroundControl):
    CP_DIST   = 3.0   # metres – enables Collision Prevention
    CP_DELAY  = 0.5   # seconds
    SER_TEL2_BAUD = 921600
    MAV_1_CONFIG  = 102   # TELEM2
    MAV_1_MODE    = 2     # Onboard
"""

import time
import argparse
import threading
import sys
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit("[ERROR] pyrealsense2 not found. Run: pip install pyrealsense2")

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("[ERROR] pymavlink not found. Run: pip install pymavlink")


# ── Default connection ─────────────────────────────────────────────────────────
CONNECTION_STRING  = "/dev/ttyUSB0"
CONNECTION_BAUD    = 921600

# ── Camera settings ───────────────────────────────────────────────────────────
DEPTH_WIDTH  = 640
DEPTH_HEIGHT = 480
DEPTH_FPS    = 30

# ── Obstacle distance settings ────────────────────────────────────────────────
OBSTACLE_DISTANCE_HZ = 15          # Send rate (Hz) — PX4 needs ≥10 Hz
MIN_DEPTH_M          = 0.20        # D435i min reliable range (m)
MAX_DEPTH_M          = 10.0        # Clamp beyond this (m)
FOV_H_DEG            = 87.0        # D435 / D435i horizontal FOV (degrees)
NUM_BINS             = 72          # OBSTACLE_DISTANCE array size (MAVLink max)

# ── Filter toggle ─────────────────────────────────────────────────────────────
USE_FILTERS  = True
DEBUG_ENABLE = False               # Set True to print distances each cycle


# ─────────────────────────────────────────────────────────────────────────────
# RealSense pipeline
# ─────────────────────────────────────────────────────────────────────────────
def init_realsense():
    """Start RealSense depth stream. Returns (pipeline, depth_scale)."""
    pipe    = rs.pipeline()
    cfg     = rs.config()
    cfg.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, DEPTH_FPS)
    profile = pipe.start(cfg)

    sensor      = profile.get_device().first_depth_sensor()
    depth_scale = sensor.get_depth_scale()
    print(f"[RealSense] Started. Depth scale: {depth_scale:.6f} m/unit")
    return pipe, depth_scale


def build_filters():
    """Build the RealSense post-processing filter chain."""
    return [
        rs.decimation_filter(),
        rs.threshold_filter(),
        rs.disparity_transform(True),   # depth → disparity
        rs.spatial_filter(),
        rs.temporal_filter(),
        rs.disparity_transform(False),  # disparity → depth
    ]


# ─────────────────────────────────────────────────────────────────────────────
# Depth → 72-bin distance array
# ─────────────────────────────────────────────────────────────────────────────
def depth_frame_to_obstacle_distances(depth_frame, depth_scale, filters):
    """
    Convert a RealSense depth frame into a 72-element uint16 array (cm).
    Unknown / out-of-range bins are set to 65535 (UINT16_MAX).

    The camera's horizontal FOV is divided into NUM_BINS angular sectors.
    The minimum distance in each sector is taken from the middle row of the
    depth image so distances are on a consistent horizontal plane.
    """
    if USE_FILTERS:
        for f in filters:
            depth_frame = f.process(depth_frame)

    depth_image = np.asanyarray(depth_frame.get_data())

    # Sample the middle row only (horizontal distances)
    mid_row = depth_image[depth_image.shape[0] // 2, :]

    # Convert raw units → metres
    row_m = mid_row.astype(np.float32) * depth_scale

    # Zero pixels mean no return — set to max so they are ignored
    row_m[(row_m == 0) | (row_m < MIN_DEPTH_M) | (row_m > MAX_DEPTH_M)] = MAX_DEPTH_M

    # Initialise all bins as unknown (65535 = UINT16_MAX)
    obstacles_cm = np.full(NUM_BINS, 65535, dtype=np.uint16)

    increment_deg = 360.0 / NUM_BINS          # 5° per bin
    fov_start     = -FOV_H_DEG / 2            # e.g. −43.5°

    num_cols = row_m.shape[0]

    for col in range(num_cols):
        # Map pixel column → angle relative to camera centre (negative = left)
        angle_deg = fov_start + (col / num_cols) * FOV_H_DEG

        # Map angle to bin index (0° = forward, clockwise positive)
        bin_idx = int(round(angle_deg % 360 / increment_deg)) % NUM_BINS

        dist_cm = int(row_m[col] * 100)       # metres → centimetres
        dist_cm = min(dist_cm, 65534)          # clamp below UINT16_MAX

        # Keep the closest reading per bin
        if dist_cm < obstacles_cm[bin_idx]:
            obstacles_cm[bin_idx] = dist_cm

    return obstacles_cm


# ─────────────────────────────────────────────────────────────────────────────
# MAVLink: OBSTACLE_DISTANCE
# ─────────────────────────────────────────────────────────────────────────────
def send_obstacle_distance(conn, distances_cm):
    """
    Send MAVLink OBSTACLE_DISTANCE (#330) to PX4.

    PX4 Collision Prevention reads this message and brakes the vehicle
    when it enters the CP_DIST envelope.

    distances_cm : uint16[72] – distance per sector in cm (65535 = unknown)
    """
    increment_deg = 360.0 / NUM_BINS          # 5.0°
    angle_offset  = -FOV_H_DEG / 2            # −43.5° (left edge of FOV)

    conn.mav.obstacle_distance_send(
        int(time.time() * 1e6),               # time_usec
        0,                                    # sensor_type: MAV_DISTANCE_SENSOR_LASER
        distances_cm.tolist(),                # distances[72] in cm
        0,                                    # increment (uint8, legacy — use increment_f)
        int(MIN_DEPTH_M * 100),               # min_distance cm
        int(MAX_DEPTH_M * 100),               # max_distance cm
        increment_deg,                        # increment_f (float, MAVLink 2)
        angle_offset,                         # angle_offset (float, MAVLink 2)
        12,                                   # frame: MAV_FRAME_BODY_FRD
    )


# ─────────────────────────────────────────────────────────────────────────────
# MAVLink connection
# ─────────────────────────────────────────────────────────────────────────────
def connect_mavlink(connection_string, baud):
    """Connect to PX4 and wait for heartbeat."""
    print(f"[MAVLink] Connecting to {connection_string} @ {baud} baud ...")
    conn = mavutil.mavlink_connection(connection_string, baud=baud)
    conn.wait_heartbeat(timeout=15)
    print(f"[MAVLink] Heartbeat received from system {conn.target_system}, "
          f"component {conn.target_component}")
    return conn


# ─────────────────────────────────────────────────────────────────────────────
# Main loop
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="RealSense D4xx → PX4 Collision Prevention"
    )
    parser.add_argument(
        "--connect", default=CONNECTION_STRING,
        help="MAVLink connection string (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--baud", type=int, default=CONNECTION_BAUD,
        help="Serial baud rate (default: 921600)"
    )
    parser.add_argument(
        "--hz", type=float, default=OBSTACLE_DISTANCE_HZ,
        help="OBSTACLE_DISTANCE publish rate in Hz (default: 15)"
    )
    parser.add_argument(
        "--debug", action="store_true",
        help="Print distance array each cycle"
    )
    args = parser.parse_args()

    debug = args.debug
    period = 1.0 / args.hz

    # ── Init hardware ──────────────────────────────────────────────────────
    pipe, depth_scale = init_realsense()
    filters = build_filters()
    conn    = connect_mavlink(args.connect, args.baud)

    print(f"[Main] Sending OBSTACLE_DISTANCE at {args.hz} Hz. Press Ctrl+C to stop.")

    try:
        while True:
            t_start = time.time()

            # Grab depth frame
            frames       = pipe.wait_for_frames(timeout_ms=1000)
            depth_frame  = frames.get_depth_frame()

            if not depth_frame:
                print("[WARN] No depth frame received.")
                continue

            # Convert → 72-bin distance array
            distances_cm = depth_frame_to_obstacle_distances(
                depth_frame, depth_scale, filters
            )

            # Send to PX4
            try:
                send_obstacle_distance(conn, distances_cm)
            except Exception as e:
                print(f"[WARN] MAVLink send failed: {e}. Retrying next cycle.")

            if debug:
                valid = distances_cm[distances_cm < 65535]
                if valid.size:
                    print(f"[DEBUG] Min: {valid.min()/100:.2f}m  "
                          f"Max: {valid.max()/100:.2f}m  "
                          f"Valid bins: {valid.size}/{NUM_BINS}")

            # Maintain send rate
            elapsed = time.time() - t_start
            sleep   = period - elapsed
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n[Main] Stopped by user.")
    finally:
        pipe.stop()
        print("[Main] RealSense pipeline stopped.")


if __name__ == "__main__":
    main()
