# =============================================================================
# pi5_camera_receiver.py
#
# Uses the N6 OpenMV camera
# Each frame arrives over USB serial as:
#   1) one JSON text line (blob metadata + jpeg_len), ending in \n
#   2) exactly jpeg_len raw JPEG bytes
#
# This script reads that, decodes the JPEG, draws extra overlay info on top,
# and shows it with cv2.imshow(). Needs a display 
#
# =============================================================================

import json
import time

import cv2
import numpy as np
import serial

# -----------------------------------------------------------------------------
# Config
# -----------------------------------------------------------------------------
SERIAL_PORT = "/dev/ttyACM0"   
BAUD_RATE = 1000000            

HEADER_READ_TIMEOUT_S = 2.0    # how long to wait for a header line before giving up
PAYLOAD_READ_TIMEOUT_S = 2.0   # how long to wait for the full JPEG payload


def read_exact(ser, n):
    """Read exactly n bytes from the serial port, or raise TimeoutError."""
    buf = bytearray()
    deadline = time.time() + PAYLOAD_READ_TIMEOUT_S
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
        if time.time() > deadline:
            raise TimeoutError(f"only got {len(buf)}/{n} bytes")
    return bytes(buf)


def open_serial():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=HEADER_READ_TIMEOUT_S)
    ser.dtr = False
    time.sleep(2)  # let the board settle after opening the port
    ser.reset_input_buffer()
    print(f"[serial] connected on {SERIAL_PORT}")
    return ser


def read_frame(ser):
    """Read one (header_dict, jpeg_bytes) pair. Returns None on a bad/partial frame."""
    line = ser.readline()
    if not line:
        return None  # timed out waiting for a header line

    try:
        text = line.decode("utf-8", errors="ignore").strip()
        header = json.loads(text)
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None  # got garbage instead of a JSON header - resync next loop

    jpeg_len = header.get("jpeg_len", 0)
    if jpeg_len <= 0:
        return None

    try:
        jpeg_bytes = read_exact(ser, jpeg_len)
    except TimeoutError:
        return None

    return header, jpeg_bytes


def main():
    ser = None
    pi_fps = 0.0
    frame_count = 0
    fps_window_start = time.time()

    print("Press 'q' in the video window to quit.")

    while True:
        if ser is None:
            try:
                ser = open_serial()
            except serial.SerialException as e:
                print(f"[serial] error: {e} - retrying in 2s")
                time.sleep(2)
                continue

        try:
            result = read_frame(ser)
        except serial.SerialException as e:
            print(f"[serial] lost connection: {e} - reconnecting")
            ser.close()
            ser = None
            continue

        if result is None:
            continue  # partial/garbled frame, just try again on the next line

        header, jpeg_bytes = result

        frame = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue  # corrupt JPEG, skip this frame

        # --- Pi-side FPS  ---
        frame_count += 1
        elapsed = time.time() - fps_window_start
        if elapsed >= 1.0:
            pi_fps = frame_count / elapsed
            frame_count = 0
            fps_window_start = time.time()

        # --- extra overlay info ---
        overlay_lines = [
            f"Blobs: {header.get('blob_count', 0)}",
            f"N6 FPS: {header.get('fps', 0)}",
            f"Link FPS: {pi_fps:.1f}",
            time.strftime("%Y-%m-%d %H:%M:%S"),
        ]
        for i, text in enumerate(overlay_lines):
            y = 20 + i * 20
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # closest blob distance, if any, shown big for at-a-glance reading
        blobs = header.get("blobs", [])
        if blobs:
            closest = min(blobs, key=lambda b: b.get("dist_cm", 1e9))
            cv2.putText(frame, f"{closest['dist_cm']:.0f} cm",
                        (10, frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("N6 Live Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    if ser:
        ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()