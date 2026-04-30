from machine import SPI, Pin
from sx127x import SX127x
import time
import json

print("Starting RX...")

# SPI
spi = SPI(0,
          baudrate=1000000,
          polarity=0,
          phase=0,
          sck=Pin(18),
          mosi=Pin(19),
          miso=Pin(16))

# LoRa RX
lora = SX127x(
    spi,
    pins={
        "ss": 17,
        "reset": 20,
        "dio_0": None
    },
    parameters={
        "frequency": 433000000,
        "spreading_factor": 10,
        "signal_bandwidth": 125000,
        "coding_rate": 5,
        "preamble_length": 8,
        "sync_word": 0x12,
        "enable_CRC": True
    }
)

print("RX Ready")

# Start listening
lora.receive()

while True:
    payload = lora.listen(2000)

    if payload:
        try:
            data = payload.decode().strip()

            # CSV -> split
            parts = data.split(",")

            # Expect: DEVICE_NAME,human_detect
            device = parts[0]
            human_detect = int(parts[1])

            # Convert to JSON
            json_data = {
                "device": device,
                "human_detect": human_detect
            }

            print("JSON:", json.dumps(json_data))

        except Exception as e:
            print("Error parsing:", e)
            print("Raw:", payload)

    else:
        print("No packet")

    time.sleep(0.5)