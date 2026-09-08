import cv2
import numpy as np
from openmv import Camera

def nothing(x):
    pass

cv2.namedWindow("Trackbars")
cv2.createTrackbar("H min", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("H max", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("S min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("S max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("V max", "Trackbars", 255, 255, nothing)

with Camera(port='/dev/ttyACM0', baudrate=921600) as cam:
    cam.streaming(True)
    while True:
        frame = cam.read_frame()
        if frame is None:
            continue

        w, h = frame['width'], frame['height']
        img = np.frombuffer(frame['data'], dtype=np.uint8).reshape((h, w, 3))
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos("H min", "Trackbars")
        h_max = cv2.getTrackbarPos("H max", "Trackbars")
        s_min = cv2.getTrackbarPos("S min", "Trackbars")
        s_max = cv2.getTrackbarPos("S max", "Trackbars")
        v_min = cv2.getTrackbarPos("V min", "Trackbars")
        v_max = cv2.getTrackbarPos("V max", "Trackbars")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)

        result = cv2.bitwise_and(img_bgr, img_bgr, mask=mask)

        cv2.imshow("Original", img_bgr)
        cv2.imshow("Mask", mask)
        cv2.imshow("Result", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(f"Final range: (({h_min}, {s_min}, {v_min}), ({h_max}, {s_max}, {v_max}))")
            break

cv2.destroyAllWindows()