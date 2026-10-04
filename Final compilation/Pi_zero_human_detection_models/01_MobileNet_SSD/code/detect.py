import cv2
import numpy as np
import tensorflow as tf
import time

# Load model
interpreter = tf.lite.Interpreter(model_path="../model/detect.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

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

    H, W, _ = frame.shape

    for i in range(count):

        if classes[i] == 0 and scores[i] > 0.30:

            ymin, xmin, ymax, xmax = boxes[i]

            xmin = int(max(0, xmin * W))
            xmax = int(min(W, xmax * W))
            ymin = int(max(0, ymin * H))
            ymax = int(min(H, ymax * H))

            cv2.rectangle(frame,
                          (xmin, ymin),
                          (xmax, ymax),
                          (0, 255, 0),
                          2)

            cv2.putText(frame,
                        f"Person {scores[i]:.2f}",
                        (xmin, max(20, ymin - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0,255,0),
                        2)

    fps = 1/(time.time()-start)

    cv2.putText(frame,
                f"FPS: {fps:.1f}",
                (20,40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255,0,0),
                2)

    cv2.imshow("MobileNet SSD Human Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()