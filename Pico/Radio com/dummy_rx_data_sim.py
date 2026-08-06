from machine import Pin
import time
import json
import sys
import uselect

# Built-in LED
led = Pin("LED", Pin.OUT)

# Poll object lets us check if serial data is available
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

counter = 0

while True:

    # ----------------------------
    # Simulated RF data
    # ----------------------------
    dummy_data = {
        "packet": counter,
        "temperature": 25 + (counter % 5),
        "humidity": 60 + (counter % 10),
        "signal": -70 + (counter % 6),
        "status": "OK"
    }

    # Send JSON to Flask
    print(json.dumps(dummy_data))

    counter += 1

    # ----------------------------
    # Check for incoming command
    # ----------------------------
    if poll.poll(10):          # Wait up to 10 ms

        line = sys.stdin.readline().strip()

        if line:
            command = json.loads(line)

            if command.get("led") == "toggle":
                led.toggle()

                # Optional acknowledgement
                # print(json.dumps({
                #     "ack": "LED toggled"
                # }))


    time.sleep(1)