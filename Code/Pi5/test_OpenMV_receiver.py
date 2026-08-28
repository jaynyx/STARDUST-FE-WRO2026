import cv2
import numpy as np
import imageio
import time
from openmv import Camera

script = """
import csi
csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QVGA)
while True:
    csi0.snapshot()
"""

def process_frame(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections = []  # dictionnairies storing the coordinates of the detected red objects

    for c in contours:
        area = cv2.contourArea(c)
        if area > 200:
            x, y, w, h = cv2.boundingRect(c)
            center_x = x + w // 2
            center_y = y + h // 2

            detections.append({         
                "color": "red",
                "x": x, "y": y, "w": w, "h": h,
                "center_x": center_x, "center_y": center_y,
                "area": area
            })

            cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img_bgr, f"red ({center_x},{center_y})", (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    return img_bgr, detections   # <-- was just "return img_bgr" before

filename = f"/Users/jacobsarni/Desktop/run_{int(time.time())}.mp4"
video_writer = imageio.get_writer(filename, fps=20)
print(f"Recording to: {filename}")

with Camera('/dev/tty.usbmodem101', baudrate=921600) as cam:
    cam.stop()
    cam.exec(script)
    cam.streaming(True)

    while True:
        frame = cam.read_frame()
        if frame is None:
            continue

        w, h = frame['width'], frame['height']
        img = np.frombuffer(frame['data'], dtype=np.uint8).reshape((h, w, 3))
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        img_bgr, detections = process_frame(img_bgr)

        # imageio wants RGB, not BGR — convert back before writing
        video_writer.append_data(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))

        cv2.imshow('OpenMV Live Feed', img_bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

video_writer.close()
cv2.destroyAllWindows()