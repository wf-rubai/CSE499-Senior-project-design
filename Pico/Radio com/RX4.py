# =============================================================================
# LoRa RX — Raspberry Pi Pico (MicroPython)
# Receives and parses GPS + sensor payload from the combined GPS+TX3 transmitter
# Expected CSV format: ROVER_NAME,human_detect,latitude,longitude,satellites
# =============================================================================

from machine import SPI, Pin
from sx127x import SX127x
import time
import json

# -----------------------------------------------------------------------------
# SPI + LoRa (SX127x) setup  — must match TX settings exactly
# -----------------------------------------------------------------------------
spi = SPI(
    0,
    baudrate=1000000,
    polarity=0,
    phase=0,
    sck=Pin(18),
    mosi=Pin(19),
    miso=Pin(16),
)

lora = SX127x(
    spi,
    pins={
        "ss": 17,
        "reset": 20,
        "dio_0": None,
    },
    parameters={
        "frequency": 433000000,
        "spreading_factor": 10,
        "signal_bandwidth": 125000,
        "coding_rate": 5,
        "preamble_length": 8,
        "sync_word": 0x12,
        "enable_CRC": True,
    },
)

# Start listening
lora.receive()

# -----------------------------------------------------------------------------
# Main loop
# -----------------------------------------------------------------------------
while True:
    payload = lora.listen(2000)

    if payload:
        try:
            data = payload.decode().strip()
            parts = data.split(",")

            if len(parts) >= 5:
                json_data = {
                    "name": parts[0],
                    "human": int(parts[1]),
                    "lat": float(parts[2]),
                    "lon": float(parts[3]),
                    "satellites": parts[4],
                }

                print(json.dumps(json_data))  # ONLY this

        except:
            pass  # ignore errors silently

    time.sleep(0.5)