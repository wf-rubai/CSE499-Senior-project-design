from flask import Flask, jsonify, request
from flask_cors import CORS
import serial
import threading
import json
import time

app = Flask(__name__)
CORS(app)

# ----------------------------
# Serial Port
# ----------------------------
ser = serial.Serial('/dev/cu.usbmodem101', 115200, timeout=1)

latest_data = {"status": "Waiting for Pico..."}

lock = threading.Lock()

# ----------------------------
# Background Serial Reader
# ----------------------------
def serial_reader():
    global latest_data

    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()

            if not line:
                continue

            print("FROM PICO: ", line)

            try:
                data = json.loads(line)

                with lock:
                    latest_data = data

            except json.JSONDecodeError:
                print("Invalid JSON from Pico")

        except Exception as e:
            print("Serial Error:", e)

        time.sleep(0.01)


threading.Thread(target=serial_reader, daemon=True).start()


# ----------------------------
# Webpage requests latest data
# ----------------------------
@app.route('/data', methods=['GET'])
def get_data():

    with lock:
        return jsonify(latest_data)


# ----------------------------
# Webpage sends command
# ----------------------------
@app.route('/send', methods=['POST'])
def send_command():

    command = request.get_json()

    print("FROM WEBPAGE: ", command)

    try:
        message = json.dumps(command) + "\n"
        ser.write(message.encode())

        return jsonify({
            "success": True
        })

    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        })


# ----------------------------
# Start Server
# ----------------------------
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001, debug=False)