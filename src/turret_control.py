import cv2
import sys
import serial
import time
import numpy as np

SERIAL_PORT   = 'COM3'
BAUD_RATE     = 115200
SEND_INTERVAL = 0.02
CAMERA_INDEX  = 0
FRAME_WIDTH   = 640
FRAME_HEIGHT  = 480
SMOOTH        = 0.2

# Static zones kept purely for UI/visual distance tracking references
ZONES = [
    {"name": "ZONE 1 - VERY CLOSE", "min_w": 250, "max_w": 999, "color": (0,   0,   255), "box_size": 300},
    {"name": "ZONE 2 - CLOSE",      "min_w": 200, "max_w": 249, "color": (0,   128, 255), "box_size": 250},
    {"name": "ZONE 3 - NORMAL",     "min_w": 150, "max_w": 199, "color": (0,   255, 0  ), "box_size": 200},
    {"name": "ZONE 4 - FAR",        "min_w": 100, "max_w": 149, "color": (255, 255, 0  ), "box_size": 150},
    {"name": "ZONE 5 - VERY FAR",   "min_w": 60,  "max_w": 99,  "color": (255, 128, 0  ), "box_size": 100},
    {"name": "ZONE 6 - MAX RANGE",  "min_w": 0,   "max_w": 59,  "color": (255, 0,   0  ), "box_size": 60 },
]

def get_zone(face_width):
    for i, z in enumerate(ZONES):
        if z["min_w"] <= face_width <= z["max_w"]:
            return i + 1, z
    return 6, ZONES[5]

# Module-level state for temporal continuity across frames (mirrors the
# global-state pattern already used for `arduino` elsewhere in this file).
_last_laser_pos = None
_frames_since_laser = 0

def detect_laser(frame):
    """
    Ultra-relaxed filter to account for lens vignetting and blurring at the
    frame edges. Loosening V to 140 and S to 60 means absolute brightness
    can no longer be trusted to reject skin tones -- skin under dim/warm
    light easily crosses that floor, and the laser's actual target is the
    face, so we can't just exclude the face region either.

    Instead, each candidate blob is scored on LOCAL contrast against its
    own immediate surroundings (a real laser dot is a sharp hotspot
    relative to what's right next to it, even in a dark corner -- skin
    reads close to the same value as the skin around it) combined with
    circularity, plus a mild bias toward staying near the last known
    position since a real dot moves continuously rather than teleporting.
    """
    global _last_laser_pos, _frames_since_laser

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    v_channel = hsv[:, :, 2]
    h_frame, w_frame = v_channel.shape[:2]

    # HUE: Kept the same
    # SATURATION: Dropped to 60 (edges lose color accuracy and look more washed out)
    # VALUE: Dropped heavily to 140 to catch the laser in the darker corners of the lens
    lower_red1 = np.array([0,   60, 140])
    upper_red1 = np.array([10,  255, 255])
    lower_red2 = np.array([165, 60, 140])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask  = cv2.bitwise_or(mask1, mask2)

    # Clean up stray pixels
    mask = cv2.dilate(mask, None, iterations=1)

    cv2.imshow("Laser Mask Debug", mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        _frames_since_laser += 1
        return None, None

    best_score = -1.0
    best_point = None

    for c in contours:
        area = cv2.contourArea(c)
        # SIZE FILTER: Increased the max limit to 150.
        # When the laser gets to the edge of the lens, it blurs and appears larger!
        if area < 1 or area > 150:
            continue

        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue

        # Roundness: a laser dot stays roughly circular even when blurred at
        # the edges. Skin patches that slip through the loose threshold tend
        # to be irregular blobs. 1.0 = perfect circle.
        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity < 0.4:
            continue

        x, y, w, h = cv2.boundingRect(c)

        # Local "halo" region: a box roughly 3x the blob's size, centered on
        # it, used to measure what's immediately around the blob. Built as a
        # small local array (not full-frame) to keep this cheap per contour.
        x0 = max(0, x - w)
        y0 = max(0, y - h)
        x1 = min(w_frame, x + w + w)
        y1 = min(h_frame, y + h + h)

        local_w, local_h = x1 - x0, y1 - y0
        if local_w <= 0 or local_h <= 0:
            continue

        shifted_contour = c - np.array([x0, y0])
        blob_mask_local = np.zeros((local_h, local_w), dtype=np.uint8)
        cv2.drawContours(blob_mask_local, [shifted_contour], -1, 255, -1)

        local_v = v_channel[y0:y1, x0:x1]
        background_mask_local = cv2.bitwise_not(blob_mask_local)

        if cv2.countNonZero(background_mask_local) == 0:
            continue

        blob_mean_v = cv2.mean(local_v, mask=blob_mask_local)[0]
        bg_mean_v = cv2.mean(local_v, mask=background_mask_local)[0]
        contrast = blob_mean_v - bg_mean_v

        # Not meaningfully brighter than its own surroundings -- likely
        # skin or background lit roughly evenly, not a laser hotspot.
        if contrast < 15:
            continue

        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        score = contrast * circularity

        # Mild bonus for staying near where we last saw the laser. Only
        # applied with a recent lock (last 10 frames), so it can't trap
        # tracking on a stale position once the real dot has moved on.
        if _last_laser_pos is not None and _frames_since_laser < 10:
            dist = np.hypot(cx - _last_laser_pos[0], cy - _last_laser_pos[1])
            score *= max(0.5, 1.0 - dist / 200.0)

        if score > best_score:
            best_score = score
            best_point = (int(cx), int(cy))

    if best_point is None:
        _frames_since_laser += 1
        return None, None

    _last_laser_pos = best_point
    _frames_since_laser = 0
    return best_point

def connect_serial():
    try:
        s = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
        time.sleep(2)
        print("Serial connected!")
        return s
    except Exception as e:
        print(f"Serial not connected: {e}")
        return None

arduino = connect_serial()

def run():
    global arduino

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        sys.exit(1)

    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    if face_cascade.empty():
        print("Error: Could not load face cascade.")
        sys.exit(1)

    print("============================")
    print("  Laser Seeking Control Active")
    print("  ESC = Quit | U = Upload Mode | R = Reconnect")
    print("============================")

    last_send    = 0
    center_x     = FRAME_WIDTH  // 2
    center_y     = FRAME_HEIGHT // 2
    smooth_err_x = 0.0
    smooth_err_y = 0.0
    current_zone = 3
    zone_info    = ZONES[2]

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to grab frame.")
            break

        frame = cv2.flip(frame, 1)
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. Process Face and Laser Sensors
        lx, ly = detect_laser(frame)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Draw static alignment boxes
        for i, z in enumerate(ZONES):
            bs = z["box_size"]
            bx = center_x - bs // 2
            by = center_y - bs // 2
            cv2.rectangle(frame, (bx, by), (bx+bs, by+bs), z["color"], 1)

        face_cx = face_cy = None

        # 2. Extract Valid Face Data
        for (x, y, w, h) in faces:
            if y > (FRAME_HEIGHT - 160):
                continue

            face_cx = x + w // 2
            face_cy = y + h // 2
            current_zone, zone_info = get_zone(w)

            cv2.rectangle(frame, (x, y), (x+w, y+h), zone_info["color"], 3)
            cv2.circle(frame, (face_cx, face_cy), 5, (0, 255, 0), -1)
            cv2.putText(frame, zone_info["name"], (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, zone_info["color"], 2)
            break

        # 3. CONTROL LOOP PROCESSOR
        if face_cx is not None and lx is not None:
            # Mode 2: Fine Visual Tracking (Drive error between face and laser)
            mode = 2
            raw_err_x = face_cx - lx
            raw_err_y = face_cy - ly
            cv2.line(frame, (lx, ly), (face_cx, face_cy), (255, 0, 255), 2)

            smooth_err_x = SMOOTH * raw_err_x + (1 - SMOOTH) * smooth_err_x
            smooth_err_y = SMOOTH * raw_err_y + (1 - SMOOTH) * smooth_err_y
            ex = int(smooth_err_x)
            ey = int(smooth_err_y)
        else:
            # Mode 0: Sweep/Patrol (Either no face seen OR face seen but laser is still missing)
            mode = 0
            ex = 9999
            ey = 9999

        # Transmit structured command package
        now = time.time()
        if arduino and (now - last_send) >= SEND_INTERVAL:
            arduino.write(f"{ex},{ey},{mode}\n".encode())
            last_send = now

        # UI Diagnostic Elements
        if lx is not None:
            cv2.circle(frame, (lx, ly), 8, (0, 255, 255), -1)
            cv2.putText(frame, "LASER DETECTED", (lx+12, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        cv2.line(frame, (center_x-15, center_y), (center_x+15, center_y), (255, 255, 0), 1)
        cv2.line(frame, (center_x, center_y-15), (center_x, center_y+15), (255, 255, 0), 1)

        status = "CONNECTED" if arduino else "DISCONNECTED"
        status_color = (0, 255, 0) if arduino else (0, 0, 255)
        cv2.putText(frame, f"Serial: {status}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)

        mode_labels = ["SWEEPING (HUNTING LASER)", "UNUSED", "LASER LOCKED ON FACE"]
        mode_colors = [(0, 0, 255), (255, 165, 0), (0, 255, 0)]
        cv2.putText(frame, f"Control Loop: {mode_labels[mode]}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_colors[mode], 2)

        cv2.imshow("Turret Vision Loop", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == 27: # ESC
            break
        elif key == ord('u'):
            if arduino:
                arduino.close()
                arduino = None
                print("Port released safely.")
        elif key == ord('r'):
            arduino = connect_serial()

    cap.release()
    cv2.destroyAllWindows()
    if arduino:
        arduino.close()

if __name__ == "__main__":
    run()
