import cv2
import numpy as np
import tensorflow as tf
import time

interpreter = tf.lite.Interpreter(model_path="../model/detect.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
height = input_details[0]["shape"][1]
width = input_details[0]["shape"][2]

cap = cv2.VideoCapture(0)

while True:

    start = time.time()

    ret, frame = cap.read()
    if not ret:
        break

    image = cv2.resize(frame, (width, height))
    image = np.expand_dims(image, axis=0).astype(np.uint8)

    interpreter.set_tensor(input_details[0]["index"], image)
    interpreter.invoke()

    boxes = interpreter.get_tensor(output_details[0]["index"])[0]
    classes = interpreter.get_tensor(output_details[1]["index"])[0]
    scores = interpreter.get_tensor(output_details[2]["index"])[0]
    count = int(interpreter.get_tensor(output_details[3]["index"])[0])

    person_found = False
    confidence = 0

    for i in range(count):

        if int(classes[i]) == 0 and scores[i] > 0.5:
            person_found = True
            confidence = scores[i]
            break

    inference = (time.time() - start) * 1000

    if person_found:
        print(f"PERSON DETECTED | {confidence:.2f} | {inference:.1f} ms")
        # Send signal here
    else:
        print(f"NO PERSON | {inference:.1f} ms")

    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
