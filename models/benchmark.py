import cv2
import time
import psutil
import os
import csv
import numpy as np
import tensorflow as tf
from ultralytics import YOLO

# =========================
# CHANGE THIS SECTION
# =========================
MODEL_NAME = "MobileNetSSD"   # YOLOv8n / YOLO11n / MobileNetSSD
DURATION = 60            # seconds
# =========================

process = psutil.Process(os.getpid())

def benchmark_yolo(model_path):
    model = YOLO(model_path)
    model_size = os.path.getsize(model_path) / (1024 * 1024)

    cap = cv2.VideoCapture(0)

    fps_list = []
    inf_list = []
    cpu_list = []
    ram_list = []
    detections = 0
    frames = 0

    start_time = time.time()

    while time.time() - start_time < DURATION:

        ret, frame = cap.read()
        if not ret:
            break

        t0 = time.time()

        results = model(frame, verbose=False)

        inference = (time.time() - t0) * 1000

        person_found = False

        for box in results[0].boxes:
            if int(box.cls[0]) == 0 and float(box.conf[0]) > 0.5:
                person_found = True
                break

        if person_found:
            detections += 1

        frames += 1

        elapsed = time.time() - t0
        fps = 1 / elapsed if elapsed > 0 else 0

        fps_list.append(fps)
        inf_list.append(inference)
        cpu_list.append(psutil.cpu_percent(interval=None))
        ram_list.append(process.memory_info().rss / (1024 * 1024))

    cap.release()

    return {
        "Model": MODEL_NAME,
        "Model Size (MB)": round(model_size, 2),
        "Avg FPS": round(np.mean(fps_list), 2),
        "Avg Inference (ms)": round(np.mean(inf_list), 2),
        "Avg CPU (%)": round(np.mean(cpu_list), 2),
        "Avg RAM (MB)": round(np.mean(ram_list), 2),
        "Frames": frames,
        "Detections": detections
    }

def benchmark_tflite(model_path):
    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    h = input_details[0]["shape"][1]
    w = input_details[0]["shape"][2]

    model_size = os.path.getsize(model_path) / (1024 * 1024)

    cap = cv2.VideoCapture(0)

    fps_list = []
    inf_list = []
    cpu_list = []
    ram_list = []
    detections = 0
    frames = 0

    start_time = time.time()

    while time.time() - start_time < DURATION:

        ret, frame = cap.read()
        if not ret:
            break

        img = cv2.resize(frame, (w, h))
        img = np.expand_dims(img, axis=0).astype(np.uint8)

        t0 = time.time()

        interpreter.set_tensor(input_details[0]["index"], img)
        interpreter.invoke()

        classes = interpreter.get_tensor(output_details[1]["index"])[0]
        scores = interpreter.get_tensor(output_details[2]["index"])[0]

        inference = (time.time() - t0) * 1000

        person_found = False

        for cls, score in zip(classes, scores):
            if int(cls) == 0 and score > 0.5:
                person_found = True
                break

        if person_found:
            detections += 1

        frames += 1

        elapsed = time.time() - t0
        fps = 1 / elapsed if elapsed > 0 else 0

        fps_list.append(fps)
        inf_list.append(inference)
        cpu_list.append(psutil.cpu_percent(interval=None))
        ram_list.append(process.memory_info().rss / (1024 * 1024))

    cap.release()

    return {
        "Model": MODEL_NAME,
        "Model Size (MB)": round(model_size, 2),
        "Avg FPS": round(np.mean(fps_list), 2),
        "Avg Inference (ms)": round(np.mean(inf_list), 2),
        "Avg CPU (%)": round(np.mean(cpu_list), 2),
        "Avg RAM (MB)": round(np.mean(ram_list), 2),
        "Frames": frames,
        "Detections": detections
    }

# =========================
# SELECT MODEL
# =========================

if MODEL_NAME == "YOLOv8n":
    result = benchmark_yolo(r"C:\Users\Hp\OneDrive\Desktop\CSE499\Pi_zero_human_detection_models\02_YOLOv8\code\yolov8n.pt")

elif MODEL_NAME == "YOLO11n":
    result = benchmark_yolo(r"C:\Users\Hp\OneDrive\Desktop\CSE499\Pi_zero_human_detection_models\03_YOLO11\code\yolo11n.pt")

elif MODEL_NAME == "MobileNetSSD":
    result = benchmark_tflite(r"C:\Users\Hp\OneDrive\Desktop\CSE499\Pi_zero_human_detection_models\01_MobileNet_SSD\model\detect.tflite")

else:
    raise ValueError("Unknown model")

print("\n===== BENCHMARK RESULT =====")
for k, v in result.items():
    print(f"{k}: {v}")

# Save to CSV
csv_file = "benchmark_results.csv"

write_header = not os.path.exists(csv_file)

with open(csv_file, "a", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=result.keys())

    if write_header:
        writer.writeheader()

    writer.writerow(result)

print(f"\nSaved to {csv_file}")