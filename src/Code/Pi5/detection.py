import cv2
import numpy as np
import time



COLOR_RANGES = {
    "red":   [((0, 120, 70), (10, 255, 255))],
    "green": [((36, 80, 60), (85, 255, 255))]
}

LINE_COLOR_RANGES = {
    "blue":   [((94, 80, 60), (126, 255, 255))],
    "orange": [((5, 100, 100), (18, 255, 255))],
}

PARKING_COLOR_RANGES = {
    "pink": [((140, 60, 90), (172, 255, 255))],
}


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
    # Finds colored pillars, draws boxes on img_bgr, returns detections list.
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)  # converts from BGR to Hue Saturation Value
    detections = []

    # ADDING A COLOR WILL SIMPLY ADD AN ITERATION TO THAT MAIN FOR LOOP, CREATING ANOTHER MASK AND DRAWRING THE CORRESPONDING BLOB CONTOURS
    for color_name, ranges in COLOR_RANGES.items():  # for each colors, it will loop through all pixels with cv2.inRange(hsv, lower, upper) and add 255 (white) or 0 to the mask for that specific color, then add the corresponding contours and information to the dictionaries for further processing
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)  # creates a mask detecting all pixels falling in <ranges> for that iteration's color

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # creates a contour using that temporary mask
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



def detect_parking_markers(img_bgr, min_area=250):
    # Finds the pink parallel-parking walls
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    detections = []

    for color_name, ranges in PARKING_COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area > min_area:
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w // 2, y + h // 2
                detections.append({"color": color_name, "x": x, "y": y, "w": w, "h": h,
                                    "center_x": cx, "center_y": cy, "area": area})
                cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (255, 0, 255), 2)
                cv2.putText(img_bgr, f"{color_name} ({cx},{cy})", (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

    return img_bgr, detections


def pink_wall_ahead(img_bgr, min_total_area=2500):
    """
    Used once at startup, before the robot moves, to tell the two challenges apart:
    the obstacle challenge starts boxed in by pink parallel-parking lines directly
    ahead, the open challenge does not have any pink markings in view.
    Returns (bool, blobs).
    """
    _, blobs = detect_parking_markers(img_bgr)
    total_area = sum(b["area"] for b in blobs)
    return total_area >= min_total_area, blobs


def line_window_presence(img_bgr, half_width=45, top_ratio=0.55, bottom_ratio=0.9):
    """
    Checks a small window near the bottom-center of the frame for blue/orange
    presence -- used by LapTracker to detect the corner lines crossing under
    the robot, independently of the detect_line() used for the
    initial direction decision.
    """
    h, w = img_bgr.shape[:2]
    cx = w // 2
    y1, y2 = int(h * top_ratio), int(h * bottom_ratio)
    x1, x2 = max(0, cx - half_width), min(w, cx + half_width)
    window = img_bgr[y1:y2, x1:x2]
    hsv = cv2.cvtColor(window, cv2.COLOR_BGR2HSV)

    present = {}
    for color_name, ranges in LINE_COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)
        present[color_name] = bool(np.any(mask))
    return present


class LapTracker:
    """
    Counts corner passages. Every corner of the mat has both a blue line
    and an orange line.

    So rather than counting on either color, this only ever watches the
    one color that matches the current direction (blue when
    direction == "left", orange when direction == "right"). That way a
    single corner can't be double-counted from seeing the two lines at
    separate moments -- only the relevant one is ever looked at.
    4 corners = 1 lap, 12 counted passages = 3 laps.

    A cooldown after each counted passage stops that same line (or a
    wide/blurry one) from being counted twice as the robot crosses it.
    """
    CORNERS_PER_LAP = 4

    def __init__(self, direction, laps_goal=3, cooldown_s=0.6):
        assert direction in ("left", "right")
        self.direction = direction
        self.count_color = "blue" if direction == "left" else "orange"
        self.laps_goal = laps_goal
        self.cooldown_s = cooldown_s

        self.corners_this_lap = 0
        self.laps = 0
        self._line_was_visible = False
        self._last_count_time = 0.0

    def update(self, line_presence):
        # line_presence: dict from line_window_presence(). Returns current lap count.
        now = time.time()
        visible_now = bool(line_presence.get(self.count_color))

        if visible_now and not self._line_was_visible and (now - self._last_count_time) >= self.cooldown_s:
            self.corners_this_lap += 1
            self._last_count_time = now
            if self.corners_this_lap >= self.CORNERS_PER_LAP:
                self.corners_this_lap = 0
                self.laps += 1

        self._line_was_visible = visible_now
        return self.laps

    def approaching_final_corner(self):
        # True once the robot has entered the last corner of the last lap
        return self.laps == self.laps_goal - 1 and self.corners_this_lap == self.CORNERS_PER_LAP - 1

    def done(self):
        return self.laps >= self.laps_goal


def detect_line(img_bgr, roi_bottom_ratio=0.3):
    """
    Detects straight colored floor lines (blue/orange) using Hough line detection
    on the color mask, rather than blob/contour detection.

    The closest line is whichever segment reaches furthest down into the frame,
    i.e. has the largest y (closest to the robot/bottom of the image).
    """
    h, w = img_bgr.shape[:2]
    roi_y_start = int(h * (1 - roi_bottom_ratio))   # roi (region of interest) restricted to the bottom part of the image
    roi = img_bgr[roi_y_start:h, :]                 # the region of interest is from the h * (1 - roi_bottom_ratio) to the bottom of the image, and all columns, since h is 0 at the top of the image,
    """ view the initial matrix as 0 --- > h in terms of top to bottom, hence if we only want to kee the bottom 30% we need to start the region of interest at h * (1-0.3) and go all the way to h."""

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    best_line = None
    best_closeness = -1  # largest y reached by a segment (in full-frame coords) seen so far

    for color_name, ranges in LINE_COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)  # hsv.shape is 3d array and [0:2] drops the cahnnel count, using uint8 holds 8 bit ints without having to use float which saves memory
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)  # whenever the pixel's color falls in that iteration's color <range>, sets the mask pixel to 255 otherwise its 0


        # find straight line segments directly in the color mask
        lines = cv2.HoughLinesP(
            mask, 1, np.pi / 90, threshold=50,
            minLineLength=40, maxLineGap=15
        )
        """ np.pi / 90 :    checks lines at 2 degrees increment, higher number means faster but less precise line detection
            minLineLength:  can be used to remove short lines that are cause by the noise
            maxLineGap:     will probably be decreased since it is the maximum gap between aligned lines that considers them as the same and since on the mat we have very clearly drawn lines a small value could help with reliability
        """

        if lines is None:
            continue

        for line in lines:
            x1, y1, x2, y2 = line.flatten() # outputs a plain 1 d array and stores corresponding values into the variables
            length = np.hypot(x2 - x1, y2 - y1)
            closeness = max(y1, y2)  # how far down into the frame this segment's nearest point reaches

            if closeness > best_closeness:  # keep whichever segment (of either color) comes closest to the robot
                best_closeness = closeness
                best_line = {
                    "color": color_name,
                    "x1": x1, "y1": y1 + roi_y_start,
                    "x2": x2, "y2": y2 + roi_y_start,
                    "length": length
                }

    if best_line:  # simply draws the overlay for debugging
        color_draw = (255, 0, 0) if best_line["color"] == "blue" else (0, 165, 255)
        cv2.line(img_bgr, (best_line["x1"], best_line["y1"]),
                  (best_line["x2"], best_line["y2"]), color_draw, 2)
        cv2.putText(img_bgr, best_line["color"], (best_line["x1"], best_line["y1"] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_draw, 1)

    return img_bgr, best_line





WALL_COLOR_RANGES = {
    "wall": [((10, 0, 0), (70, 70, 70))],
}



def detect_walls(img_bgr, roi_top_ratio=0.25, dark_threshold=50, min_area=800,
                  approx_epsilon_ratio=0.02):
    """
    Detects black walls via brightness thresholding + morphological cleanup +
    contour detection. Optimized for reliability over raw speed.
    """
    h, w = img_bgr.shape[:2]
    roi_y_start = int(h * roi_top_ratio)
    roi = img_bgr[roi_y_start:h, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # stronger blur — smooths out floor texture/noise before thresholding
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    # from the internet:
    # Otsu's method auto-picks the best threshold value per-frame, instead of
    # relying on one fixed number that might not hold under changing lighting
    
    _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # morphological cleanup: remove small noise specks, then fill small gaps/holes
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)   # kills small noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)  # fills small gaps

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    PINK = (203, 192, 255)  # BGR

    walls = []
    for c in contours:
        area = cv2.contourArea(c)
        if area > min_area:
            perimeter = cv2.arcLength(c, True)
            epsilon = approx_epsilon_ratio * perimeter
            simplified = cv2.approxPolyDP(c, epsilon, True)

            x, y, bw, bh = cv2.boundingRect(c)

            # solidity check: real solid walls should mostly fill their bounding box;
            # a low solidity suggests a noisy/broken/non-wall-shaped blob
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0

            if solidity < 0.5:
                continue  # skip oddly-shaped detections, likely not a real wall

            walls.append({
                "x": x, "y": y + roi_y_start,
                "w": bw, "h": bh,
                "center_x": x + bw // 2,
                "area": area,
                "solidity": round(solidity, 2)
            })

            shifted = simplified.copy()
            shifted[:, :, 1] += roi_y_start
            cv2.drawContours(img_bgr, [shifted], -1, PINK, 2)

    return img_bgr, walls
