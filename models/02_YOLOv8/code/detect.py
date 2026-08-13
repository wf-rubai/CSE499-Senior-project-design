from ultralytics import YOLO
import cv2

# Load the official YOLOv8 Nano model
model = YOLO("yolov8n.pt")

# Open laptop webcam
cap = cv2.VideoCapture(0)

while True:
    success, frame = cap.read()

    if not success:
        break

    # Run detection
    results = model(frame)

    # Draw detections
    annotated_frame = results[0].plot()

    # Show output
    cv2.imshow("YOLOv8 Human Detection", annotated_frame)

    # Press Q to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()