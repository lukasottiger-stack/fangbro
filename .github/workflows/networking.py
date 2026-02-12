import cv2
import numpy as np
import time
import tkinter as tk
import math
from PIL import ImageGrab

# ------------------- SETTINGS -------------------
LOWER_GREEN = np.array([35, 100, 100])
UPPER_GREEN = np.array([85, 255, 255])

MIN_SIZE = 60
MAX_SIZE = 90

CANVAS_SIZE = 600
PREDICTION_TIME = 0.8
MIN_LINE_LENGTH = 50
MAX_LINE_LENGTH = 350

DOT_RADIUS = 8
DOT_COLOR = "red"
LINE_COLOR = "lime"
LINE_WIDTH = 5

# Screenshot area: match your War Thunder window exactly
SCREEN_BBOX = (0, 0, 1920, 1080)

# Optical flow settings
FLOW_DOWNSCALE = 1       # full resolution for accuracy
FLOW_EVERY_N = 1          # compute flow every frame
MAX_BG_DELTA = 50         # clamp spikes

# Background flow smoothing factor
BG_SMOOTH = 0.7
# ------------------------------------------------

print("✅ Optimized green tracker (smoothed background compensation)")
print("   Press Ctrl+C to stop\n")

# ==================== OVERLAY CANVAS ====================
root = tk.Tk()
root.overrideredirect(True)
root.attributes("-topmost", True)
root.attributes("-transparentcolor", "black")
root.configure(bg="black")

canvas = tk.Canvas(root, width=CANVAS_SIZE, height=CANVAS_SIZE,
                   bg="black", highlightthickness=0)
canvas.pack()

center_offset = CANVAS_SIZE // 2
root.withdraw()
# ====================================================

prev_cx = None
prev_cy = None
prev_time = None

prev_gray = None
bg_dx = 0
bg_dy = 0

frame_count = 0
last_status = time.time()
last_detected = 0.0

try:
    while True:
        # ---------------- SCREENSHOT ----------------
        screenshot = ImageGrab.grab(bbox=SCREEN_BBOX)
        frame = np.array(screenshot)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # -------------------------------------------

        # ---------------- OPTICAL FLOW ----------------
        if prev_gray is None:
            prev_gray = gray_full.copy()
        elif frame_count % FLOW_EVERY_N == 0:
            flow = cv2.calcOpticalFlowFarneback(prev_gray, gray_full,
                                                None, 0.5, 3, 15, 3, 5, 1.2, 0)
            median_dx = np.median(flow[..., 0])
            median_dy = np.median(flow[..., 1])

            # weighted smoothing to avoid spikes
            bg_dx = BG_SMOOTH * bg_dx + (1 - BG_SMOOTH) * median_dx
            bg_dy = BG_SMOOTH * bg_dy + (1 - BG_SMOOTH) * median_dy

            # clamp
            if abs(bg_dx) > MAX_BG_DELTA: bg_dx = 0
            if abs(bg_dy) > MAX_BG_DELTA: bg_dy = 0

            prev_gray = gray_full.copy()

        # ---------------- GREEN BOX DETECTION ----------------
        # crop frame only for green detection for speed
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        detected_this_frame = False

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 60:
                continue

            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

            if len(approx) >= 4:
                x, y, w, h = cv2.boundingRect(approx)
                if abs(w - h) <= 20 and MIN_SIZE <= w <= MAX_SIZE:

                    # Correct coordinates relative to full screen
                    cx = x + w // 2 + SCREEN_BBOX[0]
                    cy = y + h // 2 + SCREEN_BBOX[1]
                    current_time = time.time()

                    vx = 0.0
                    vy = 0.0
                    speed = 0.0
                    dt = 0.0

                    if prev_cx is not None and prev_time is not None:
                        dt = current_time - prev_time
                        if dt > 0:
                            vx = (cx - prev_cx) / dt
                            vy = (cy - prev_cy) / dt
                            speed = math.hypot(vx, vy)

                            # subtract smoothed background flow
                            vx -= bg_dx / dt
                            vy -= bg_dy / dt
                            speed = math.hypot(vx, vy)

                    prev_cx = cx
                    prev_cy = cy
                    prev_time = current_time

                    fill_ratio = area / (w * h) if (w * h) > 0 else 0

                    # ---------------- DEBUG OUTPUT ----------------
                    print(f"DETECTED #{frame_count} | pos=({cx},{cy}) | size={w}x{h} | "
                          f"area={area:.0f} | fill={fill_ratio:.1%} | dt={dt:.3f}s | "
                          f"vx={vx:+.0f} | vy={vy:+.0f} | speed={speed:.0f} px/s | "
                          f"bg_dx={bg_dx:.2f} | bg_dy={bg_dy:.2f}")

                    # ---------------- DRAW OVERLAY ----------------
                    canvas.delete("all")
                    canvas.create_oval(center_offset - DOT_RADIUS,
                                       center_offset - DOT_RADIUS,
                                       center_offset + DOT_RADIUS,
                                       center_offset + DOT_RADIUS,
                                       fill=DOT_COLOR,
                                       outline="white",
                                       width=2)

                    if speed > 5:
                        dir_x = vx / speed
                        dir_y = vy / speed
                        line_length = max(MIN_LINE_LENGTH,
                                          min(speed * PREDICTION_TIME,
                                              MAX_LINE_LENGTH))
                        end_x = center_offset + dir_x * line_length
                        end_y = center_offset + dir_y * line_length

                        canvas.create_line(center_offset, center_offset,
                                           end_x, end_y,
                                           fill=LINE_COLOR,
                                           width=LINE_WIDTH,
                                           arrow=tk.LAST,
                                           arrowshape=(16,20,6))

                    root.geometry(f"{CANVAS_SIZE}x{CANVAS_SIZE}+{cx - center_offset}+{cy - center_offset}")
                    root.deiconify()
                    root.update()

                    detected_this_frame = True
                    last_detected = time.time()
                    break

        if time.time() - last_detected > 0.5:
            root.withdraw()

        frame_count += 1
        if time.time() - last_status > 5:
            print(f"Still running... ({frame_count} frames processed in last 5s)")
            frame_count = 0
            last_status = time.time()

except KeyboardInterrupt:
    print("\n🛑 Stopped.")
finally:
    cv2.destroyAllWindows()
    root.destroy()
