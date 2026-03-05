import serial
import numpy as np
import cv2
import struct
import time
import csv
import os

from picamera2 import Picamera2
from ultralytics import YOLO

# -----------------------------
# Configuration
# -----------------------------
PERSON_CLASS_ID = 0
FRAME_SIZE = (640, 480)

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 57600

IMAGE_WIDTH = 32
IMAGE_HEIGHT = 24
DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 240

SAVE_DIR = "detections"
os.makedirs(SAVE_DIR, exist_ok=True)

SAVE_INTERVAL = 3          # seconds
CAPTURE_WINDOW = 5 * 60    # 5 minutes

# -----------------------------
# Initialize YOLO + Camera
# -----------------------------
model = YOLO("yolo26n.onnx")

picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(
        {"format": "RGB888", "size": FRAME_SIZE}
    )
)
picam2.start()

# -----------------------------
# Utility Functions
# -----------------------------
def to_display(img_bytes):
    arr = np.frombuffer(img_bytes, dtype=np.uint8).reshape((IMAGE_HEIGHT, IMAGE_WIDTH))
    arr = np.flip(arr, axis=1)
    norm = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    col = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)
    return cv2.resize(col, (DISPLAY_WIDTH, DISPLAY_HEIGHT), interpolation=cv2.INTER_NEAREST)

def get_label_mask(step3_bytes, label):
    arr = np.frombuffer(step3_bytes, dtype=np.uint8).reshape((IMAGE_HEIGHT, IMAGE_WIDTH))
    arr_flip = np.flip(arr, axis=1)
    return (arr_flip == label).astype(np.uint8) * 255

def save_image(img):
    timestamp = int(time.time())
    filename = os.path.join(SAVE_DIR, f"frame_{timestamp}.jpg")
    cv2.imwrite(filename, img)

def save_csv(thermal_people, yolo_people):
    timestamp = int(time.time())
    filename = os.path.join(SAVE_DIR, "detections.csv")
    file_exists = os.path.isfile(filename)

    with open(filename, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(["timestamp", "thermal_count", "yolo_count", "thermal_data", "yolo_data"])

        writer.writerow([
            timestamp,
            len(thermal_people),
            len(yolo_people),
            str(thermal_people),
            str(yolo_people)
        ])

# -----------------------------
# Main Loop
# -----------------------------
def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return

    HEADER = b'\xFE\x01\xFE\x01'
    last_sync_time = time.time()

    # Saving timers
    last_save_time = 0
    capture_start_time = None

    while True:
        try:
            # -----------------------------
            # Sync to header
            # -----------------------------
            sync = b''
            while True:
                byte = ser.read(1)
                if not byte:
                    continue
                sync += byte
                sync = sync[-4:]
                if sync == HEADER:
                    break
                if time.time() - last_sync_time > 5:
                    print("Still syncing...")
                    last_sync_time = time.time()

            # -----------------------------
            # Read packet
            # -----------------------------
            size_bytes = ser.read(2)
            if len(size_bytes) != 2:
                ser.flushInput()
                continue

            packet_size = int.from_bytes(size_bytes, "little")
            packet_data = ser.read(packet_size)
            if len(packet_data) != packet_size:
                ser.flushInput()
                continue

            # -----------------------------
            # Parse packet
            # -----------------------------
            image_bytes  = packet_data[0:768]
            step1_bytes  = packet_data[768:1536]
            step2_bytes  = packet_data[1536:2304]
            step3_bytes  = packet_data[2304:3072]

            num_detected = packet_data[3072]
            idx_ptr = 3073

            thermal_people = []
            for _ in range(num_detected):
                if idx_ptr + 4 > len(packet_data):
                    thermal_people = []
                    break
                y, x, area = struct.unpack('<BBH', packet_data[idx_ptr:idx_ptr+4])
                thermal_people.append({"x": x, "y": y, "area": area})
                idx_ptr += 4

            # -----------------------------
            # Build thermal displays
            # -----------------------------
            display_orig  = to_display(image_bytes)
            display_step1 = to_display(step1_bytes)
            display_step2 = to_display(step2_bytes)
            display_step3 = to_display(step3_bytes)

            scale_x = DISPLAY_WIDTH / IMAGE_WIDTH
            scale_y = DISPLAY_HEIGHT / IMAGE_HEIGHT

            for idx, p in enumerate(thermal_people):
                label = idx + 1
                mask_small = get_label_mask(step3_bytes, label)
                mask_large = cv2.resize(mask_small, (DISPLAY_WIDTH, DISPLAY_HEIGHT), interpolation=cv2.INTER_NEAREST)

                overlay = display_orig.copy()
                overlay[mask_large > 0] = (0, 255, 0)
                display_orig = cv2.addWeighted(display_orig, 0.7, overlay, 0.3, 0)

                tx = int((32 - p["x"]) * scale_x)
                ty = int(p["y"] * scale_y)

                cv2.putText(display_orig, f"({p['x']},{p['y']})", (tx, ty),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
                cv2.putText(display_orig, f"Area:{p['area']}", (tx, ty+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

            cv2.putText(display_orig, "Raw Image", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_step1, "Background", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_step2, "Distance Map", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_step3, "Watershed", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            thermal_grid = np.vstack([
                np.hstack([display_orig, display_step1]),
                np.hstack([display_step2, display_step3])
            ])

            # -----------------------------
            # RGB + YOLO
            # -----------------------------
            with picam2.captured_request() as req:
                rgb = req.make_array("main")

            results = model(rgb, imgsz=320, verbose=False)

            yolo_people = []
            for r in results:
                for box in r.boxes:
                    if int(box.cls[0]) == PERSON_CLASS_ID:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        yolo_people.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2})
                        cv2.rectangle(rgb, (x1, y1), (x2, y2), (0,255,0), 2)

            rgb_resized = cv2.resize(rgb, (thermal_grid.shape[1], thermal_grid.shape[0]))
            cv2.putText(rgb_resized, "RGB + YOLO", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            combined = np.hstack((rgb_resized, thermal_grid))

            cv2.imshow("Thermal + RGB", combined)

            # -----------------------------
            # Saving logic
            # -----------------------------
            now = time.time()
            detected = (len(thermal_people) > 0) or (len(yolo_people) > 0)

            if detected and capture_start_time is None:
                capture_start_time = now
                print("Capture window started.")

            if capture_start_time is not None:
                if now - capture_start_time <= CAPTURE_WINDOW:
                    if now - last_save_time >= SAVE_INTERVAL:
                        save_image(combined)
                        save_csv(thermal_people, yolo_people)
                        last_save_time = now
                else:
                    capture_start_time = None
                    print("Capture window ended.")

            # CSV always saves every 3 seconds even outside capture window
            if now - last_save_time >= SAVE_INTERVAL:
                save_csv(thermal_people, yolo_people)
                last_save_time = now

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print("Error:", e)
            ser.flushInput()

    ser.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
