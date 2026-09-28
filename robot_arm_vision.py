"""
Robot Arm Vision - Button -> Color -> Camera finds ball -> Arm picks it
-----------------------------------------------------------------------
Flow:
    Arm button pressed  ->  "BTN1" / "BTN2" / "BTN3" sent to laptop over USB serial
    Laptop maps button to color  (BTN1=RED, BTN2=BLUE, BTN3=GREEN)
    Webcam searches for a ball of THAT color
    Ball found & stable  ->  laptop sends  "PICK,<COLOR>,<x>,<y>"  to the arm
    Ball not found       ->  laptop sends  "NOTFOUND,<COLOR>"
    Arm finishes pick    ->  arm sends "DONE"  (ready for next button)

Install:
    pip install opencv-python numpy pyserial

Run:
    py robot_arm_vision.py                       # test without arm (keys 1/2/3 = buttons)
    py robot_arm_vision.py --serial COM3         # with the real arm
    py robot_arm_vision.py --serial COM3 --camera 1   # Logitech USB cam

Keys (video window):
    1 / 2 / 3 : simulate button 1 / 2 / 3  (for testing without the arm)
    m         : toggle mask view
    q / ESC   : quit
"""

import argparse
import sys
import time

import cv2
import numpy as np

# --------------------------------------------------------------------------
# CONFIG
# --------------------------------------------------------------------------

# Button -> color (change here if your buttons change)
BUTTON_TO_COLOR = {
    "BTN1": "RED",
    "BTN2": "BLUE",
    "BTN3": "GREEN",
}
KEY_TO_BUTTON = {ord("1"): "BTN1", ord("2"): "BTN2", ord("3"): "BTN3"}

# HSV ranges (OpenCV: H 0-179, S 0-255, V 0-255). Red needs two ranges.
COLOR_RANGES = {
    "RED": [
        (np.array([0, 120, 70]), np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([179, 255, 255])),
    ],
    "GREEN": [(np.array([40, 70, 70]), np.array([85, 255, 255]))],
    "BLUE": [(np.array([95, 120, 70]), np.array([130, 255, 255]))],
}
DRAW_COLORS = {"RED": (0, 0, 255), "GREEN": (0, 200, 0), "BLUE": (255, 0, 0)}

MIN_AREA_RATIO = 0.004      # ball must cover >= 0.4% of frame (balls are small)
SEARCH_TIMEOUT = 6.0        # seconds to look for the ball after button press
STABLE_FRAMES = 8           # ball must be seen this many frames in a row
ARM_TIMEOUT = 30.0          # if arm never says DONE, auto-ready after this
KERNEL = np.ones((5, 5), np.uint8)


# --------------------------------------------------------------------------
# DETECTION
# --------------------------------------------------------------------------

def build_mask(hsv, ranges):
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, lo, hi)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
    return mask


def find_balls(frame, min_area_ratio=MIN_AREA_RATIO):
    """Return ({color: (cx, cy, area, box)}, combined_mask) for every color seen."""
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    min_area = frame.shape[0] * frame.shape[1] * min_area_ratio

    found = {}
    combined = np.zeros(frame.shape[:2], dtype=np.uint8)
    for name, ranges in COLOR_RANGES.items():
        mask = build_mask(hsv, ranges)
        combined = cv2.bitwise_or(combined, mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        cx, cy = int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])
        found[name] = (cx, cy, area, cv2.boundingRect(c))
    return found, combined


# --------------------------------------------------------------------------
# PICK CONTROLLER  (button -> search -> send command -> wait for arm)
# --------------------------------------------------------------------------

class PickController:
    IDLE, SEARCHING, WAITING_ARM = "IDLE", "SEARCHING", "WAITING_ARM"

    def __init__(self, search_timeout=SEARCH_TIMEOUT, stable_frames=STABLE_FRAMES,
                 arm_timeout=ARM_TIMEOUT):
        self.search_timeout = search_timeout
        self.stable_frames = stable_frames
        self.arm_timeout = arm_timeout
        self.state = self.IDLE
        self.target = None
        self.history = []
        self.t0 = 0.0

    def press(self, button, now):
        color = BUTTON_TO_COLOR.get(button)
        if color is None:
            return
        if self.state == self.WAITING_ARM:
            print(f"[BUSY] arm is still picking, ignoring {button}")
            return
        self.target, self.state, self.history, self.t0 = color, self.SEARCHING, [], now
        print(f"[{button}] -> searching for {color} ball...")

    def arm_done(self):
        if self.state == self.WAITING_ARM:
            print("[ARM] DONE -> ready for next button")
        self.state, self.target, self.history = self.IDLE, None, []

    def update(self, balls, now):
        """Call once per frame. Returns a list of message strings to send to the arm."""
        out = []
        if self.state == self.SEARCHING:
            if self.target in balls:
                cx, cy, _, _ = balls[self.target]
                self.history.append((cx, cy))
            else:
                self.history = []                       # must be seen continuously

            if len(self.history) >= self.stable_frames:
                xs = [p[0] for p in self.history[-self.stable_frames:]]
                ys = [p[1] for p in self.history[-self.stable_frames:]]
                x, y = int(sum(xs) / len(xs)), int(sum(ys) / len(ys))
                out.append(f"PICK,{self.target},{x},{y}")
                print(f"[ACTION] {self.target} ball at ({x},{y}) -> PICK sent")
                self.state, self.t0 = self.WAITING_ARM, now
            elif now - self.t0 > self.search_timeout:
                out.append(f"NOTFOUND,{self.target}")
                print(f"[WARN] {self.target} ball not found")
                self.state, self.target, self.history = self.IDLE, None, []

        elif self.state == self.WAITING_ARM and now - self.t0 > self.arm_timeout:
            print("[WARN] arm did not send DONE, resetting")
            self.arm_done()
        return out


# --------------------------------------------------------------------------
# CAMERA / SERIAL HELPERS
# --------------------------------------------------------------------------

def open_camera(index):
    backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap


def read_serial_lines(port):
    """Non-blocking: return list of complete lines waiting on the port."""
    lines = []
    try:
        while port.in_waiting:
            raw = port.readline().decode(errors="ignore").strip().upper()
            if raw:
                lines.append(raw)
    except Exception as e:  # noqa: BLE001
        print(f"[SERIAL] read error: {e}")
    return lines


# --------------------------------------------------------------------------
# MAIN
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Robot arm color-pick vision")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--serial", type=str, default=None, help="e.g. COM3")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    port = None
    if args.serial:
        try:
            import serial
            port = serial.Serial(args.serial, args.baud, timeout=0.05)
            time.sleep(2)
            print(f"[SERIAL] connected {args.serial} @ {args.baud}")
        except Exception as e:  # noqa: BLE001
            print(f"[SERIAL] cannot open {args.serial}: {e}\n         running in keyboard-test mode")
    else:
        print("No --serial given: keyboard-test mode (press 1/2/3 in the video window)")

    cap = open_camera(args.camera)
    if cap is None:
        print(f"Cannot open camera {args.camera}")
        return

    ctl = PickController()
    show_mask = False

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed")
            break
        now = time.time()

        # ---- inputs: arm buttons (serial) ----
        if port:
            for line in read_serial_lines(port):
                if line in BUTTON_TO_COLOR:
                    ctl.press(line, now)
                elif line == "DONE":
                    ctl.arm_done()

        # ---- vision + controller ----
        balls, mask = find_balls(frame)
        for msg in ctl.update(balls, now):
            if port:
                port.write((msg + "\n").encode())

        # ---- draw ----
        for name, (cx, cy, _, (x, y, w, h)) in balls.items():
            is_target = (name == ctl.target)
            col = DRAW_COLORS[name]
            cv2.rectangle(frame, (x, y), (x + w, y + h), col, 4 if is_target else 1)
            cv2.putText(frame, name, (x, max(y - 8, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            if is_target:
                cv2.drawMarker(frame, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)

        status = f"{ctl.state}" + (f" | target: {ctl.target}" if ctl.target else "")
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 34), (30, 30, 30), -1)
        cv2.putText(frame, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Robot Arm Vision", frame)
        if show_mask:
            cv2.imshow("Mask (debug)", mask)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("m"):
            show_mask = not show_mask
            if not show_mask:
                try:
                    cv2.destroyWindow("Mask (debug)")
                except cv2.error:
                    pass
        elif key in KEY_TO_BUTTON:
            ctl.press(KEY_TO_BUTTON[key], now)
        elif key == ord("d"):           # simulate arm DONE in keyboard-test mode
            ctl.arm_done()

    cap.release()
    cv2.destroyAllWindows()
    if port:
        port.close()


if __name__ == "__main__":
    main()
