"""
main.py

Autonomous entry point for the STARDUST robot (Raspberry Pi 5 side).
Meant to be launched automatically on boot (see stardust-robot.service in
this folder)

Flow:
  0) Connect to the OpenMV camera and the Arduino 
  1) Look at a few frames before moving: if pink parallel-parking markings
     are seen directly ahead, this is the Obstacle Challenge (the robot
     starts boxed into the parking spot); otherwise it's the Open Challenge.
     This decision is made before the start button is pressed.
  2) Wait for the physical start button (wired to a Pi GPIO pin)
  3) Run the matching challenge to completion, then stop.

Both challenges share the same wall-following technique (see Decision.py);
the Obstacle Challenge additionally avoids red/green pillars and finishes
with a parallel-parking maneuver, while the Open Challenge just laps the
mat and stops back where it started.
"""

import os
import time
import cv2
import numpy as np
from openmv import Camera
from gpiozero import Button

from detection import (
    detect_pillars, detect_walls, detect_line,
    pink_wall_ahead, detect_parking_markers, line_window_presence, LapTracker,
)
from Decision import RobotController
from arduino_comm import envoyer_si_nouveau

SHOW_WINDOW = os.environ.get('DISPLAY') is not None  # forced off during competition runs

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
LAPS_GOAL = 3

BUTTON_GPIO_PIN = 17                 # Pin for the start button

CHALLENGE_VOTE_FRAMES = 10          # frames sampled at boot to decide pink-ahead
DIRECTION_VOTE_FRAMES = 8           # frames sampled to decide blue-vs-orange direction

EXIT_PARKING_STEER = 0.7
EXIT_PARKING_SPEED = 0.5
EXIT_PARKING_TIME_S = 1.1
EXIT_STRAIGHTEN_TIME_S = 0.5

PARK_APPROACH_SPEED = 0.4
PARK_PINK_CLOSE_AREA = 6000         # pink blob area (px^2) considered "arrived at the spot"
PARK_REVERSE_SPEED = -0.5
PARK_REVERSE_TURN_TIME_S = 1.3
PARK_STRAIGHTEN_BACK_TIME_S = 0.9


# ---------------------------------------------------------------------------
# Camera / Arduino helpers
# ---------------------------------------------------------------------------
def read_bgr_frame(cam):
    frame = cam.read_frame()
    if frame is None:
        return None
    w, h = frame['width'], frame['height']
    img = np.frombuffer(frame['data'], dtype=np.uint8).reshape((h, w, 3))
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def process_frame(img_bgr):
    img_bgr, pillars = detect_pillars(img_bgr)
    img_bgr, walls = detect_walls(img_bgr)
    return img_bgr, {"pillars": pillars, "walls": walls}


def drive(steering, speed):
    envoyer_si_nouveau(steering=steering, rpm=speed)


def stop():
    drive(0.0, 0.0)


def show(img_bgr):
    if SHOW_WINDOW and img_bgr is not None:
        cv2.imshow('STARDUST', img_bgr)
        cv2.waitKey(1)


# ---------------------------------------------------------------------------
# Startup: tell the two challenges apart
# ---------------------------------------------------------------------------
def detect_challenge(cam):
    """
    Looks at CHALLENGE_VOTE_FRAMES frames before the robot moves. If pink
    parallel-parking markings are seen directly ahead in most of them, this
    is the Obstacle Challenge (started boxed into the parking spot);
    otherwise it's the Open Challenge.
    """
    votes = 0
    seen = 0
    while seen < CHALLENGE_VOTE_FRAMES:
        img_bgr = read_bgr_frame(cam)
        if img_bgr is None:
            continue
        seen += 1
        is_pink, _ = pink_wall_ahead(img_bgr)
        if is_pink:
            votes += 1
    challenge = "obstacle" if votes > CHALLENGE_VOTE_FRAMES / 2 else "open"
    print(f"Challenge detected: {challenge} ({votes}/{seen} frames saw pink ahead)")
    return challenge


# ---------------------------------------------------------------------------
# Physical start button
# ---------------------------------------------------------------------------
def wait_for_start_button():
    print("Challenge decided. Waiting for start button press...")
    button = Button(BUTTON_GPIO_PIN)
    button.wait_for_press()
    print("Start button pressed, starting run.")


# ---------------------------------------------------------------------------
# Shared: direction decision from the blue/orange corner lines
# ---------------------------------------------------------------------------
def detect_direction(cam):
    """
    Once out of parking: if the closest corner line is blue, the loop
    direction is "left" (follow the left wall); if orange, "right" (follow
    the right wall). Voted over a few frames for robustness.
    """
    votes = {"left": 0, "right": 0}
    seen = 0
    while seen < DIRECTION_VOTE_FRAMES:
        img_bgr = read_bgr_frame(cam)
        if img_bgr is None:
            continue
        seen += 1
        _, line = detect_line(img_bgr)
        if line is None:
            continue
        votes["left" if line["color"] == "blue" else "right"] += 1
    direction = "left" if votes["left"] >= votes["right"] else "right"
    print(f"Direction detected: {direction} (blue={votes['left']}, orange={votes['right']})")
    return direction


# ---------------------------------------------------------------------------
# Obstacle Challenge: get out of the parallel parking spot
# ---------------------------------------------------------------------------
def exit_parking(cam):
    """
    Steers away from whichever side the pink boundary markings crowd more,
    drives forward briefly to clear the spot, then straightens out.
    """
    img_bgr = read_bgr_frame(cam)
    steer_away = 1.0
    if img_bgr is not None:
        h, w = img_bgr.shape[:2]
        _, blobs = detect_parking_markers(img_bgr)
        left_area = sum(b["area"] for b in blobs if b["center_x"] < w / 2)
        right_area = sum(b["area"] for b in blobs if b["center_x"] >= w / 2)
        steer_away = 1.0 if left_area > right_area else -1.0

    start = time.time()
    while time.time() - start < EXIT_PARKING_TIME_S:
        drive(steer_away * EXIT_PARKING_STEER, EXIT_PARKING_SPEED)
        show(read_bgr_frame(cam))

    start = time.time()
    while time.time() - start < EXIT_STRAIGHTEN_TIME_S:
        drive(0.0, EXIT_PARKING_SPEED)
        show(read_bgr_frame(cam))


# ---------------------------------------------------------------------------
# Shared lap-driving loop
# ---------------------------------------------------------------------------
def run_laps(cam, controller, lap_tracker, avoid_obstacles):
    """
    Drives using the wall-following controller (plus obstacle avoidance if
    avoid_obstacles is True) until lap_tracker reports the required number
    of laps completed.
    """
    while not lap_tracker.done():
        img_bgr = read_bgr_frame(cam)
        if img_bgr is None:
            continue

        img_bgr, detections = process_frame(img_bgr)
        if not avoid_obstacles:
            detections["pillars"] = []

        action = controller.decide(detections)
        drive(action["steering"], action["speed"])

        lap_tracker.update(line_window_presence(img_bgr))
        show(img_bgr)

    stop()


# ---------------------------------------------------------------------------
# Obstacle Challenge: approach the pink parking spot and park in it
# ---------------------------------------------------------------------------
def approach_and_park(cam, controller, direction):
    # Phase 1: keep wall-following/dodging at a crawl until the pink
    # boundary markings are close and large in view.
    while True:
        img_bgr = read_bgr_frame(cam)
        if img_bgr is None:
            continue

        img_bgr, detections = process_frame(img_bgr)
        is_close, _ = pink_wall_ahead(img_bgr, min_total_area=PARK_PINK_CLOSE_AREA)

        action = controller.decide(detections)
        drive(action["steering"], PARK_APPROACH_SPEED)
        show(img_bgr)

        if is_close:
            break
    stop()
    time.sleep(0.2)

    # Phase 2: parallel park -- reverse while turning into the spot, then
    # straighten and back the rest of the way in.
    turn_sign = 1.0 if direction == "left" else -1.0

    start = time.time()
    while time.time() - start < PARK_REVERSE_TURN_TIME_S:
        drive(turn_sign * 0.9, PARK_REVERSE_SPEED)
        show(read_bgr_frame(cam))

    start = time.time()
    while time.time() - start < PARK_STRAIGHTEN_BACK_TIME_S:
        drive(0.0, PARK_REVERSE_SPEED)
        show(read_bgr_frame(cam))

    stop()


# ---------------------------------------------------------------------------
# Challenge runners
# ---------------------------------------------------------------------------
def run_obstacle_challenge(cam):
    controller = RobotController()

    exit_parking(cam)

    direction = detect_direction(cam)
    controller.set_direction(direction)

    lap_tracker = LapTracker(direction, laps_goal=LAPS_GOAL)
    run_laps(cam, controller, lap_tracker, avoid_obstacles=True)

    approach_and_park(cam, controller, direction)


def run_open_challenge(cam):
    controller = RobotController()

    direction = detect_direction(cam)
    controller.set_direction(direction)

    # Quarter-counting starts right after this point, so finishing the
    # required number of laps naturally lands the robot back near here --
    # i.e. back in its starting spot.
    lap_tracker = LapTracker(direction, laps_goal=LAPS_GOAL)
    run_laps(cam, controller, lap_tracker, avoid_obstacles=False)


# ---------------------------------------------------------------------------
def main():
    with Camera(port='/dev/ttyACM0', baudrate=921600) as cam:
        print("Camera connected. Starting initialization...")
        cam.streaming(True)
        print("Starting streaming...")

        challenge = detect_challenge(cam)

        wait_for_start_button()

        try:
            if challenge == "obstacle":
                run_obstacle_challenge(cam)
            else:
                run_open_challenge(cam)
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            stop()
            if SHOW_WINDOW:
                cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
