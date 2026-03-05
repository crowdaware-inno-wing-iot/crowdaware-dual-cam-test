import serial
import numpy as np
import cv2
import struct
import time

# ------------------------------
# Configuration
# ------------------------------
SERIAL_PORT = 'COM6'  # Change this to your serial port (e.g., 'COM3' on Windows)
BAUD_RATE = 57600
IMAGE_WIDTH = 32
IMAGE_HEIGHT = 24
DISPLAY_WIDTH = 320  # Resized width for display
DISPLAY_HEIGHT = 240 # Resized height for display

# ------------------------------
# Main Program
# ------------------------------
def main():
    try:
        # Initialize serial connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    # Calculate scaling factors for drawing on the resized image
    scale_x = DISPLAY_WIDTH / IMAGE_WIDTH
    scale_y = DISPLAY_HEIGHT / IMAGE_HEIGHT

    # blank image now twice width & height
    blank_image = np.zeros((DISPLAY_HEIGHT * 2, DISPLAY_WIDTH * 2, 3), dtype=np.uint8)
    cv2.putText(blank_image, "Waiting for data...", (DISPLAY_WIDTH // 2, DISPLAY_HEIGHT),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.imshow("Thermal Detection", blank_image)

    last_sync_time = time.time()

    while True:
        try:
            sync = b''
            while True:

                HEADER = b'\xFE\x01\xFE\x01'
                byte = ser.read(1)
                if not byte:
                    continue
                sync += byte
                if len(sync) > len(HEADER):
                    sync = sync[-len(HEADER):]
                if sync == HEADER:
                    break
                if time.time() - last_sync_time > 5:
                    print("Still syncing... No header detected.")
                    last_sync_time = time.time()
                time.sleep(0.01)

            # read packet size
            size_bytes = ser.read(2)
            if len(size_bytes) != 2:
                print(f"Warning: Failed to read packet size. Resyncing...")
                ser.flushInput()
                continue
            packet_size = int.from_bytes(size_bytes, byteorder='little')

            packet_data = ser.read(packet_size)
            if len(packet_data) != packet_size:
                print(f"Warning: Read {len(packet_data)} bytes, expected {packet_size}. Resyncing...")
                ser.flushInput()
                continue

            # parse four consecutive images
            image_bytes  = packet_data[0:768]
            step1_bytes  = packet_data[768:1536]
            step2_bytes  = packet_data[1536:2304]
            step3_bytes  = packet_data[2304:3072]

            num_detected = packet_data[3072]
            person_data_start = 3073

            detected_people = []
            for i in range(num_detected):
                if person_data_start + 4 > len(packet_data):
                    print(f"Warning: Incomplete person data for person {i}. Resyncing...")
                    ser.flushInput()
                    detected_people = []
                    break
                y, x, area = struct.unpack('<BBH', packet_data[person_data_start:person_data_start+4])
                detected_people.append({'x': x, 'y': y, 'area': area})
                person_data_start += 4

            if len(detected_people) != num_detected:
                continue

            # convert to images
            def to_display(img_bytes):
                arr = np.frombuffer(img_bytes, dtype=np.uint8).reshape((IMAGE_HEIGHT, IMAGE_WIDTH))
                arr_flip = np.flip(arr, axis=1)  # flip horizontally
                norm = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                col = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)
                return cv2.resize(col, (DISPLAY_WIDTH, DISPLAY_HEIGHT), interpolation=cv2.INTER_NEAREST)
            
            def get_label_mask(step3_bytes, label):
                arr = np.frombuffer(step3_bytes, dtype=np.uint8).reshape((IMAGE_HEIGHT, IMAGE_WIDTH))
                return (arr == label).astype(np.uint8) * 255

            display_orig  = to_display(image_bytes)
            display_step1 = to_display(step1_bytes)
            display_step2 = to_display(step2_bytes)
            display_step3 = to_display(step3_bytes)

            cv2.putText(display_orig, "Raw Image", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_step1, "Background", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_step2, "Distance Map", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_step3, "Watershed", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # draw detections on original only
            scale_x = DISPLAY_WIDTH / IMAGE_WIDTH
            scale_y = DISPLAY_HEIGHT / IMAGE_HEIGHT

            for idx, person in enumerate(detected_people):
                x_orig, y_orig, area = person['x'], person['y'], person['area']

                # Extract blob mask from step3
                label = idx + 1  # assuming labels are 1..N
                mask_small = get_label_mask(step3_bytes, label)

                # Resize mask to display size
                mask_large = cv2.resize(mask_small, (DISPLAY_WIDTH, DISPLAY_HEIGHT), interpolation=cv2.INTER_NEAREST)

                # Create colored overlay
                overlay = display_orig.copy()
                overlay[mask_large > 0] = (0, 255, 0)  # bright green blob

                # Blend overlay
                display_orig = cv2.addWeighted(display_orig, 0.7, overlay, 0.3, 0)

                # Improved readable text
                text_pos = (int(x_orig * scale_x), int(y_orig * scale_y))

                # Black outline
                cv2.putText(display_orig, f"({x_orig},{y_orig})", text_pos,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(display_orig, f"({x_orig},{y_orig})", text_pos,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)

                cv2.putText(display_orig, f"Area:{area}", (text_pos[0], text_pos[1] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(display_orig, f"Area:{area}", (text_pos[0], text_pos[1] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)

            # compose 2×2 grid
            top = np.hstack((display_orig, display_step1))
            bottom = np.hstack((display_step2, display_step3))
            combined = np.vstack((top, bottom))

            cv2.imshow("Thermal Detection", combined)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except serial.SerialTimeoutException:
            print("Serial read timeout. No data received. Displaying blank image.")
            ser.flushInput()
            cv2.imshow("Thermal Detection", blank_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except Exception as e:
            print(f"An unexpected error occurred: {e}. Attempting to resync...")
            ser.flushInput()
            time.sleep(0.1)
            cv2.imshow("Thermal Detection", blank_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Clean up
    ser.close()
    cv2.destroyAllWindows()
    print("Serial connection closed and OpenCV windows destroyed.")

if __name__ == "__main__":
    main()