from machine import SPI, Pin
from sx127x import SX127x
import time

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
        "dio_0": None   # can add later
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
    payload = lora.listen(2000)   # wait up to 2 sec

    if payload:
        try:
            print("Received:", payload.decode())
        except:
            print("Received (raw):", payload)

    else:
        print("No packet")

    time.sleep(0.5)