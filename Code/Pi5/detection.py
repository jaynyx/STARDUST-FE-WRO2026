import cv2
import numpy as np


""" to be calibrated while testing and at the competition """

COLOR_RANGES = {
    "red":   [((0, 120, 70), (10, 255, 255))],
    "green": [((36, 80, 60), (85, 255, 255))]
}

LINE_COLOR_RANGES = {
    "blue":   [((94, 80, 60), (126, 255, 255))],
    "orange": [((5, 100, 100), (18, 255, 255))],
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
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)  # converts from BGR 2 Hue Saturation Value
    detections = []

    """ ADDING A COLOR WILL SIMPLY ADD AN ITERATION TO THAT MAIN FOR LOOP, CREATING ANOTHER MASK AND DRAWRING THE CORRESPONDING BLOB CONTOURS """
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



def detect_line(img_bgr, roi_bottom_ratio=0.3):
    """
    Detects straight colored floor lines (blue/orange) using Hough line detection
    on the color mask, rather than blob/contour detection.
    """
    h, w = img_bgr.shape[:2]
    roi_y_start = int(h * (1 - roi_bottom_ratio))   # roi (region of interest) restricted to the bottom part of the image
    roi = img_bgr[roi_y_start:h, :]                 # the region of interest is from the h * (1 - roi_bottom_ratio) to the bottom of the image, and all columns, since h is 0 at the top of the image, 
    """ view the initial matrix as 0 --- > h in terms of top to bottom, hence if we only want to kee the bottom 30% we need to start the region of interest at h * (1-0.3) and go all the way to h."""
    
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    best_line = None
    best_length = 0

    for color_name, ranges in LINE_COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)  # hsv.shape is 3d array and [0:2] drops the cahnnel count, using uint8 holds 8 bit ints without having to use float which saves memory
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)  # whenever the pixel's color falls in that iteration's color <range>, sets the mask pixel to 255 otherwise its 0
    

        # find straight line segments directly in the color mask
        lines = cv2.HoughLinesP(
            mask, 1, np.pi / 90, threshold=50,
            minLineLength=40, maxLineGap=15
        )
        """ np.pi / 90 :    checks lines at 2 degrees increment, increase the number to have faster but less precise line detection
            minLineLength:  can be used to remove short lines that are cause by the noise
            maxLineGap:     will probably be decreased since it is the maximum gap between aligned lines that considers them as the same and since on the mat we have very clearly drawn lines a small value could help with reliability
        """

        if lines is None:
            continue

        for line in lines:
            x1, y1, x2, y2 = line.flatten() # outputs a plain 1 d array and stores corresponding values into the variables
            length = np.hypot(x2 - x1, y2 - y1)  # biggest line length

            if length > best_length:  # allows for keeping the biggest and most confident line instead of simply the most recently detected, this will make it so that after looping through all the frame's pixels, only the most confident line will be drawn
                best_length = length
                best_line = {
                    "color": color_name,
                    "x1": x1, "y1": y1 + roi_y_start,
                    "x2": x2, "y2": y2 + roi_y_start,
                    "length": length
                }

    #if best_line:  # simply draws the overlay for debugging
     #   color_draw = (255, 0, 0) if best_line["color"] == "blue" else (0, 165, 255)
      #  cv2.line(img_bgr, (best_line["x1"], best_line["y1"]),
      #            (best_line["x2"], best_line["y2"]), color_draw, 2)
      #  cv2.putText(img_bgr, best_line["color"], (best_line["x1"], best_line["y1"] - 5),
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_draw, 1)

    return img_bgr, best_line

# walls_lines.py

WALL_COLOR_RANGES = {
    # replace with your actual wall color(s) — tune these against your real mat/walls
    "wall": [((10, 0, 0), (70, 70, 70))],  # example: a blue-ish wall range
}

# walls_lines.py

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

    # Otsu's method auto-picks the best threshold value per-frame, instead of
    # relying on one fixed number that might not hold under changing lighting
    _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # morphological cleanup: remove small noise specks, then fill small gaps/holes
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)   # erode then dilate — kills small noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)  # dilate then erode — fills small gaps

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
