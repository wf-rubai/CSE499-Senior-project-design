from ultralytics import YOLO
import cv2
import time

model = YOLO("yolo11n.pt")

cap = cv2.VideoCapture(0)

while True:

    ret, frame = cap.read()

    if not ret:
        break

    start = time.time()

    results = model(frame, verbose=False)

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

    inference = (time.time() - start) * 1000

    if person_found:
        print(f"PERSON DETECTED | {confidence:.2f} | {inference:.1f} ms")
        # Send signal here
    else:
        print(f"NO PERSON | {inference:.1f} ms")

    if cv2.waitKey(1) == ord("q"):
        break

cap.release()