from flask import Flask, jsonify
from flask_cors import CORS
import serial
import json

app = Flask(__name__)
CORS(app)

ser = serial.Serial('/dev/cu.usbmodem101', 115200, timeout=1)

@app.route('/data')
def get_data():
    line = ser.readline().decode(errors='ignore').strip()

    try:
        data = json.loads(line)   # expect JSON from Pico
        return jsonify(data)
    except:
        return jsonify({"error": "No valid data"})

app.run(host='0.0.0.0', port=5001)