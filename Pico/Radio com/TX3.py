from machine import SPI, Pin
from sx127x import SX127x
import time

print("Starting TX...")

# CONSTANT
ROVER_NAME = "RV_0001"

# Inputs
sensor1 = Pin(15, Pin.IN)
sensor2 = Pin(14, Pin.IN)

# SPI
spi = SPI(0,
          baudrate=1000000,
          polarity=0,
          phase=0,
          sck=Pin(18),
          mosi=Pin(19),
          miso=Pin(16))

# LoRa
lora = SX127x(
    spi,
    pins={
        "ss": 17,
        "reset": 20,
        "dio_0": None
    },
    parameters={
        "frequency": 433000000,
        "tx_power_level": 20,
        "spreading_factor": 10,
        "signal_bandwidth": 125000,
        "coding_rate": 5,
        "preamble_length": 8,
        "sync_word": 0x12,
        "enable_CRC": True
    }
)

print("TX Ready")

time.sleep(2)

while True:
    print("Sending...")

    # Read sensors
    val1 = sensor1.value()
    val2 = sensor2.value()

    # Human detect logic
    human_detect = 1 if (val1 == 1 and val2 == 1) else 0

    # CSV format: device_name,human_detect
    msg = "{},{}".format(ROVER_NAME, human_detect)

    lora.println(msg)

    print("Sent:", msg)
    print("Done")

    time.sleep(2)