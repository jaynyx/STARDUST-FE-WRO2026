import os
from detection import LineCrossingCounter, detect_pillars, detect_line, detect_walls
from Decision import RobotController
from arduino_comm import envoyer_si_nouveau
import cv2
import numpy as np
import imageio
import time
from openmv import Camera
import os #to be able to check if DISPLAY is set, to know if we can show a window or not


SHOW_WINDOW = os.environ.get('DISPLAY') is not None                                              # if on a headless system, don't try to show a window THIS IS SET TO FALSE DURING COMP
print("Starting recording. Press Ctrl+C to stop.")
filename = f"/home/poxi99/STARDUST-FE-WRO2026/Code/Pi5/recordings/run_{int(time.time())}.mp4"
video_writer = imageio.get_writer(filename, fps=30)

line_counter = LineCrossingCounter(cooldown_frames=15)       # creating line counter object 

def process_frame(img_bgr):
    img_bgr, pillars = detect_pillars(img_bgr)               # calls the detect pillar function from Detection.py
    img_bgr, walls = detect_walls(img_bgr)                   # calls the detect walls function from Detection.py
    img_bgr, lines = detect_line(img_bgr)                    # calls the detect lines function from Detection.py

    crossings = line_counter.update(lines)                   # updating the line counter with the current line detection result
    cv2.putText(img_bgr, f"Crossings: {crossings}", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return img_bgr, {"pillars": pillars, "lines": lines, "walls": walls}


controller = RobotController(frame_width=320)                                       # create once, before the loop


with Camera(port='/dev/ttyACM0', baudrate=921600) as cam:
    try:
        print("Camera connected. Starting initialization...")
        cam.streaming(True)                                                         # start the receiving of frames from the camera
        print("Starting Streaming...")
        
        while True:
            frame = cam.read_frame()
            if frame is None:
                continue

            w, h = frame['width'], frame['height']
            img = np.frombuffer(frame['data'], dtype=np.uint8).reshape((h, w, 3))   # transform the raw data into a numpy array with the correct shape of height h and width w, and 3 channels (RGB) so each pixel has 0-255 RGB intensity
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)               # simply flips the values from red green blue to blue gree red for OpenCV's sake


            img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

            
            img_bgr, detections = process_frame(img_bgr)  

            action = controller.decide(detections)                                  # <-- call it here
            print(f"steering={action['steering']:.2f}")  
            print(f"speed={action['speed']:.2f}")  # temporary, just to see it working
            envoyer_si_nouveau(steering=action["steering"], rpm=action["speed"])         

            # imageio wants RGB, not BGR — convert back before writing
            video_writer.append_data(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
            SHOW_WINDOW = False
            if SHOW_WINDOW:
                cv2.imshow('OpenMV Live Feed', img_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("Exiting...")                          # handles the case where no window is shown and the user wants to exit with Ctrl+C
    finally:
        video_writer.close()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        print(f"Video saved to: {filename}")