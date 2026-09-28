"""
Real-Time Camera-Based Color Object Detection System
----------------------------------------------------
Camera Input -> Frame Capture -> Preprocessing -> Color Detection
             -> Color Classification -> Result / Action

Detects RED, GREEN, BLUE objects from a laptop webcam or external USB webcam
(Logitech etc.) and converts the detected color into a command:

    RED   -> Command 1
    GREEN -> Command 2
    BLUE  -> Command 3

Install:
    pip install opencv-python numpy
    pip install pyserial          (only if you want to send commands to ESP32/Arduino)

Run:
    python color_detector.py                  # default camera (index 0)
    python color_detector.py --camera 1       # external Logitech webcam
    python color_detector.py --serial COM3    # also send command over serial
    python color_detector.py --list           # find which camera indexes work

Keys (in the video window):
    q / ESC : quit
    m       : toggle mask view (debug)
    c       : switch to next camera
"""

import argparse
import sys
import time

import cv2
import numpy as np

# --------------------------------------------------------------------------
# CONFIG
# --------------------------------------------------------------------------

# HSV ranges (OpenCV: H 0-179, S 0-255, V 0-255)
# Red wraps around hue 0/180, so it needs two ranges.
COLOR_RANGES = {
    "RED": [
        (np.array([0, 120, 70]), np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([179, 255, 255])),
    ],
    "GREEN": [
        (np.array([40, 70, 70]), np.array([85, 255, 255])),
    ],
    "BLUE": [
        (np.array([95, 120, 70]), np.array([130, 255, 255])),
    ],
}

# Color -> command mapping (change these for your main project)
COLOR_COMMANDS = {
    "RED": 1,
    "GREEN": 2,
    "BLUE": 3,
}

# BGR colors used only for drawing
DRAW_COLORS = {
    "RED": (0, 0, 255),
    "GREEN": (0, 200, 0),
    "BLUE": (255, 0, 0),
}

MIN_AREA_RATIO = 0.01     # object must cover at least 1% of the frame
STABLE_FRAMES = 5         # same color for N frames before triggering action
KERNEL = np.ones((5, 5), np.uint8)


# --------------------------------------------------------------------------
# ACTION HOOK  -> plug your main project here
# --------------------------------------------------------------------------

def on_command(color, command, serial_port=None):
    """Called ONCE each time a new stable color is detected.
    Put your automation / robot / IoT logic here."""
    print(f"[ACTION] {color} detected -> Command {command}")

    if serial_port is not None:
        try:
            serial_port.write(f"{command}\n".encode())
        except Exception as e:  # noqa: BLE001
            print(f"[SERIAL] write failed: {e}")


# --------------------------------------------------------------------------
# CORE DETECTION
# --------------------------------------------------------------------------

def build_mask(hsv, ranges):
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, lo, hi)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
    return mask


def detect_color(frame, min_area_ratio=MIN_AREA_RATIO):
    """Return (color_name or None, bounding_box or None, combined_mask)."""
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)          # preprocessing
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    min_area = frame.shape[0] * frame.shape[1] * min_area_ratio
    best_color, best_area, best_box = None, 0, None
    combined = np.zeros(frame.shape[:2], dtype=np.uint8)

    for name, ranges in COLOR_RANGES.items():
        mask = build_mask(hsv, ranges)
        combined = cv2.bitwise_or(combined, mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area >= min_area and area > best_area:
            best_color, best_area, best_box = name, area, cv2.boundingRect(c)

    return best_color, best_box, combined


# --------------------------------------------------------------------------
# CAMERA HELPERS
# --------------------------------------------------------------------------

def open_camera(index):
    # CAP_DSHOW is faster/more reliable for USB webcams on Windows
    backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap


def list_cameras(max_index=5):
    print("Scanning cameras...")
    found = []
    for i in range(max_index):
        cap = open_camera(i)
        if cap is not None:
            ok, _ = cap.read()
            cap.release()
            if ok:
                found.append(i)
                print(f"  camera index {i}: OK")
    if not found:
        print("  no camera found")
    return found


# --------------------------------------------------------------------------
# MAIN LOOP
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Real-time R/G/B color object detection")
    ap.add_argument("--camera", type=int, default=0, help="camera index (0 = laptop, 1 = external)")
    ap.add_argument("--serial", type=str, default=None, help="serial port, e.g. COM3 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--list", action="store_true", help="list working camera indexes and exit")
    args = ap.parse_args()

    if args.list:
        list_cameras()
        return

    serial_port = None
    if args.serial:
        try:
            import serial  # pyserial
            serial_port = serial.Serial(args.serial, args.baud, timeout=1)
            time.sleep(2)  # let ESP32/Arduino reset
            print(f"[SERIAL] connected on {args.serial} @ {args.baud}")
        except Exception as e:  # noqa: BLE001
            print(f"[SERIAL] could not open {args.serial}: {e} (continuing without serial)")

    cam_index = args.camera
    cap = open_camera(cam_index)
    if cap is None:
        print(f"Cannot open camera {cam_index}. Try: python color_detector.py --list")
        return
    print(f"Camera {cam_index} started. Press q to quit.")

    show_mask = False
    last_seen, streak, last_triggered = None, 0, None
    prev_t = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed.")
            break

        frame = cv2.flip(frame, 1)  # mirror view
        color, box, mask = detect_color(frame)

        # ---- stability filter (avoid flicker) ----
        if color == last_seen:
            streak += 1
        else:
            last_seen, streak = color, 1

        if color is None:
            last_triggered = None            # object removed -> ready for next one
        elif streak >= STABLE_FRAMES and color != last_triggered:
            last_triggered = color
            on_command(color, COLOR_COMMANDS[color], serial_port)

        # ---- draw result ----
        if color and box:
            x, y, w, h = box
            cv2.rectangle(frame, (x, y), (x + w, y + h), DRAW_COLORS[color], 3)
            cv2.putText(frame, color, (x, max(y - 10, 25)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, DRAW_COLORS[color], 2)
            status = f"Detected: {color}  (Command {COLOR_COMMANDS[color]})"
        else:
            status = "Detected: none"

        now = time.time()
        fps = 1.0 / max(now - prev_t, 1e-6)
        prev_t = now

        cv2.rectangle(frame, (0, 0), (frame.shape[1], 36), (30, 30, 30), -1)
        cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"{fps:.0f} FPS", (frame.shape[1] - 90, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow("Color Object Detection", frame)
        if show_mask:
            cv2.imshow("Mask (debug)", mask)
        else:
            try:
                cv2.destroyWindow("Mask (debug)")
            except cv2.error:
                pass

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("m"):
            show_mask = not show_mask
        elif key == ord("c"):
            cap.release()
            cam_index = (cam_index + 1) % 4
            cap = open_camera(cam_index)
            if cap is None:
                cam_index = 0
                cap = open_camera(0)
            print(f"Switched to camera {cam_index}")

    cap.release()
    cv2.destroyAllWindows()
    if serial_port:
        serial_port.close()


if __name__ == "__main__":
    main()
