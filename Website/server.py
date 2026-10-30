from flask import Flask, jsonify
from flask_cors import CORS
import serial
import json
import threading

app = Flask(__name__)
CORS(app)

ser = serial.Serial('/dev/cu.usbmodem101', 115200, timeout=1)
latest_data = {"error": "Waiting for Pico data"}

def read_serial():
    global latest_data

    while True:
        line = ser.readline().decode(errors='ignore').strip()

        try:
            data = json.loads(line)
            latest_data = data
            print("Received:", data)
        except (json.JSONDecodeError, ValueError):
            pass

threading.Thread(target=read_serial, daemon=True).start()

@app.route('/data')
def get_data():
    return jsonify(latest_data)

app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)