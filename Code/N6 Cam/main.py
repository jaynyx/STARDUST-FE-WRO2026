# =============================================================================
# main.py
#
# Each frame this sends over USB serial:
#   1) one JSON text line  (blob metadata + distance + jpeg_len), terminated by \n
#   2) exactly jpeg_len raw JPEG bytes (the actual video frame)
#
# The Pi reads the JSON line, learns how many JPEG bytes follow, reads exactly
# that many bytes, and decodes+displays the frame. 
#
# =============================================================================

import csi
# import image
import time
import json
from pyb import USB_VCP

# -----------------------------------------------------------------------------
# Camera setup
# -----------------------------------------------------------------------------
cam = csi.CSI()
cam.reset()
cam.pixformat(csi.RGB565)
cam.framesize((320, 240))
cam.framerate(60)

RES = (320, 240)

# -----------------------------------------------------------------------------
# Blob detection thresholds (LAB color space: L, A, B min/max)
# Tune with: OpenMV IDE -> Tools -> Machine Vision -> Threshold Editor
# -----------------------------------------------------------------------------
THRESHOLDS = [
    (30, 100, 15, 127, 15, 127),   # reddish/orange objects
]

# -----------------------------------------------------------------------------
# Distance estimation (simple pinhole-camera model)
#   distance_cm = (KNOWN_WIDTH_CM * FOCAL_LENGTH_PX) / blob_width_px
# Calibrate FOCAL_LENGTH_PX: place object at known distance D_cm, note blob
# width W_px, then FOCAL_LENGTH_PX = (W_px * D_cm) / KNOWN_WIDTH_CM
# -----------------------------------------------------------------------------
KNOWN_WIDTH_CM = 5.0
FOCAL_LENGTH_PX = 700.0

# -----------------------------------------------------------------------------
# JPEG quality 
# -----------------------------------------------------------------------------
JPEG_QUALITY = 60

usb = USB_VCP()

clock = time.clock()

while True:
    clock.tick()
    img = cam.snapshot()

    blobs = img.find_blobs(
        THRESHOLDS,
        pixels_threshold=150,
        area_threshold=150,
        merge=True,
    )

    blob_list = []
    for b in blobs:
        img.draw_rectangle(b.rect(), color=(0, 255, 0))
        img.draw_cross(b.cx(), b.cy(), color=(255, 0, 0))

        dist_cm = -1.0
        if b.w() > 0:
            dist_cm = (KNOWN_WIDTH_CM * FOCAL_LENGTH_PX) / b.w()

        blob_list.append({
            "x": b.x(), "y": b.y(), "w": b.w(), "h": b.h(),
            "cx": b.cx(), "cy": b.cy(),
            "pixels": b.pixels(),
            "dist_cm": round(dist_cm, 1),
        })

    # Compress the frame to JPEG
    img.compress(quality=JPEG_QUALITY)
    jpeg_len = img.size()

    header = {
        "ts_ms": time.ticks_ms(),
        "fps": round(clock.fps(), 1),
        "res": list(RES),
        "blob_count": len(blob_list),
        "blobs": blob_list,
        "jpeg_len": jpeg_len,
    }

    usb.write(json.dumps(header) + "\n")
    usb.write(img)   # Image objects behave like a bytes buffer when written
