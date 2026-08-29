import cv2
import numpy as np

COLOR_RANGES = {
    "red":   [((0, 120, 70), (10, 255, 255))],
    "green": [((36, 80, 60), (85, 255, 255))],
    "blue":  [((94, 80, 60), (126, 255, 255))],
}

# walls_lines.py — add this alongside detect_line()

class LineCrossingCounter:
    def __init__(self, cooldown_frames=15):
        self.crossing_count = 0
        self.line_was_visible = False
        self.frames_since_last_crossing = cooldown_frames  # start ready to count
        self.cooldown_frames = cooldown_frames

    def update(self, line_info):
        line_visible_now = line_info is not None
        self.frames_since_last_crossing += 1

        # Rising edge: line just appeared + past last count cooldown
        if line_visible_now and not self.line_was_visible:
            if self.frames_since_last_crossing >= self.cooldown_frames:
                self.crossing_count += 1
                self.frames_since_last_crossing = 0

        self.line_was_visible = line_visible_now
        return self.crossing_count


def detect_pillars(img_bgr):
    """Finds colored pillars, draws boxes on img_bgr, returns detections list."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    detections = []

    for color_name, ranges in COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area > 200:
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w // 2, y + h // 2
                detections.append({"color": color_name, "x": x, "y": y, "w": w, "h": h,
                                    "center_x": cx, "center_y": cy, "area": area})
                cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(img_bgr, f"{color_name} ({cx},{cy})", (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    return img_bgr, detections




def detect_line(img_bgr, roi_bottom_ratio=0.3, dark_line=True):
    """
    Detects a ground-following line in the bottom strip of the frame.
    Returns the line's centroid and its offset from frame center (for steering).
    """
    h, w = img_bgr.shape[:2]
    roi_y_start = int(h * (1 - roi_bottom_ratio))
    roi = img_bgr[roi_y_start:h, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    thresh_type = cv2.THRESH_BINARY_INV if dark_line else cv2.THRESH_BINARY
    thresh_val = 60 if dark_line else 200
    _, thresh = cv2.threshold(blurred, thresh_val, 255, thresh_type)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    line_info = None
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 100:
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"]) + roi_y_start

                offset = cx - (w // 2)  # negative = line is left of center, positive = right

                line_info = {"center_x": cx, "center_y": cy, "offset": offset}

                cv2.circle(img_bgr, (cx, cy), 6, (0, 255, 0), -1)
                cv2.line(img_bgr, (w // 2, h), (cx, cy), (0, 255, 255), 2)

    return img_bgr, line_info


