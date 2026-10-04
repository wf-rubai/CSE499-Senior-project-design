"""
Use terminal to run the code. Don't use thonnys built in run button

cd ~/Desktop/Pi_zero_human_detection_models/01_MobileNet_SSD/code
python detect2.py
"""
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import subprocess
import time


# -----------------------------
# Load TFLite model
# -----------------------------

interpreter = tflite.Interpreter(
    model_path="../model/detect.tflite"
)

interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

height = input_details[0]["shape"][1]
width = input_details[0]["shape"][2]

print(f"Model input: {width}x{height}")


# -----------------------------
# Start Pi Camera
# -----------------------------

cmd = [
    "rpicam-vid",
    "-n",
    "-t", "0",
    "--width", "640",
    "--height", "480",
    "--codec", "mjpeg",
    "-o", "-"
]

process = subprocess.Popen(
    cmd,
    stdout=subprocess.PIPE,
    stderr=subprocess.DEVNULL
)

buffer = b""


# -----------------------------
# Detection loop
# -----------------------------

while True:

    data = process.stdout.read(4096)

    if not data:
        break

    buffer += data

    jpg_start = buffer.find(b'\xff\xd8')
    jpg_end = buffer.find(b'\xff\xd9')

    if jpg_start == -1 or jpg_end == -1:
        continue

    jpg = buffer[jpg_start:jpg_end + 2]

    buffer = buffer[jpg_end + 2:]


    # Decode camera frame
    frame = cv2.imdecode(
        np.frombuffer(jpg, dtype=np.uint8),
        cv2.IMREAD_COLOR
    )

    if frame is None:
        continue


    # -----------------------------
    # Start detection timer
    # -----------------------------

    start = time.time()


    # Resize for model
    image = cv2.resize(
        frame,
        (width, height)
    )

    image = np.expand_dims(
        image,
        axis=0
    ).astype(np.uint8)


    # -----------------------------
    # TFLite inference
    # -----------------------------

    interpreter.set_tensor(
        input_details[0]["index"],
        image
    )

    interpreter.invoke()


    # Get results
    boxes = interpreter.get_tensor(
        output_details[0]["index"]
    )[0]

    classes = interpreter.get_tensor(
        output_details[1]["index"]
    )[0]

    scores = interpreter.get_tensor(
        output_details[2]["index"]
    )[0]

    count = int(
        interpreter.get_tensor(
            output_details[3]["index"]
        )[0]
    )


    # -----------------------------
    # Check for person
    # -----------------------------

    person_found = False
    confidence = 0

    for i in range(count):

        if int(classes[i]) == 0 and scores[i] > 0.5:

            person_found = True
            confidence = scores[i]
            break


    # -----------------------------
    # Timing
    # -----------------------------

    inference = (time.time() - start) * 1000


    # -----------------------------
    # Output
    # -----------------------------

    if person_found:

        print(
            f"PERSON DETECTED | "
            f"{confidence:.2f} | "
            f"{inference:.1f} ms"
        )

        # Send signal here

    else:

        print(
            f"NO PERSON | "
            f"{inference:.1f} ms"
        )


# -----------------------------
# Cleanup
# -----------------------------

process.terminate()