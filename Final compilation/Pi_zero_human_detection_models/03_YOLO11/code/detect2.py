"""
Use terminal to run the code. Don't use thonnys built in run button

cd ~/Desktop/Pi_zero_human_detection_models/03_YOLO11/code
python detect2.py
"""
from ultralytics import YOLO
import cv2
import subprocess
import time
import os

model = YOLO("yolo11n.pt")

print("Starting camera...")

while True:

    # Capture one frame using Raspberry Pi camera
    subprocess.run(
        [
            "rpicam-still",
            "-n",
            "-t", "1",
            "--width", "640",
            "--height", "480",
            "-o", "/tmp/yolo_frame.jpg"
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    frame = cv2.imread("/tmp/yolo_frame.jpg")

    if frame is None:
        print("ERROR: Could not capture frame")
        continue

    start = time.time()

    results = model(
        frame,
        imgsz=320,
        verbose=False
    )

    inference = (time.time() - start) * 1000

    person_found = False
    confidence = 0

    for box in results[0].boxes:

        cls = int(box.cls[0])

        if cls == 0:

            conf = float(box.conf[0])

            if conf > 0.5:

                person_found = True
                confidence = conf
                break

    if person_found:
        print(
            f"PERSON DETECTED | "
            f"{confidence:.2f} | "
            f"{inference:.1f} ms"
        )
    else:
        print(
            f"NO PERSON | "
            f"{inference:.1f} ms"
        )