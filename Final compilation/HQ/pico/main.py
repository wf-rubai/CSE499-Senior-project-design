
# =============================================================================
# LoRa RX/TX — Raspberry Pi Pico (MicroPython)
# Two-way communication between LoRa and Flask server
#
# LoRa RX:
#   Receives GPS + sensor data
#   Validates the received data
#   Sends valid data to Flask through USB serial
#
# USB RX:
#   Receives commands/data from Flask
#   Formats the data for LoRa transmission
#   Transmits the data through LoRa
# =============================================================================

from machine import SPI, Pin
from sx127x import SX127x
import time
import json
import sys
import uselect

# =============================================================================
# SPI + LoRa SETUP
# =============================================================================

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

# Start LoRa reception
lora.receive()


# =============================================================================
# FUNCTION 1: VALIDATE RECEIVED LORA DATA
# =============================================================================

def validate_received_data(data):
    """
    Checks whether received LoRa data is valid.

    For now, all data is considered valid.
    Later, we will implement actual validation.
    """

    return True


# =============================================================================
# FUNCTION 2: PARSE RECEIVED LORA DATA
# =============================================================================

def parse_lora_data(payload):
    """
    Converts received CSV payload into a Python dictionary.

    Expected format:
    name,human,latitude,longitude,satellites
    """

    try:
        data = payload.decode().strip()
        parts = data.split(",")

        if len(parts) < 5:
            return None

        parsed_data = {
            "name": parts[0],
            "human": int(parts[1]),
            "lat": float(parts[2]),
            "lon": float(parts[3]),
            "satellites": parts[4]
        }

        return parsed_data

    except (ValueError, UnicodeError):
        return None


# =============================================================================
# FUNCTION 3: FORMAT SERVER DATA FOR LORA TRANSMISSION
# =============================================================================

def format_server_data(data):
    """
    Converts data received from Flask into a LoRa-transmittable string.

    Currently uses JSON formatting.
    Both LoRa modules must agree on this format.
    """

    try:
        message = json.dumps(data)
        return message

    except Exception:
        return None


# =============================================================================
# FUNCTION 4: HANDLE RECEIVED LORA DATA
# =============================================================================

def handle_lora_receive():
    """
    Receives LoRa payload, parses it, validates it,
    and forwards valid data to Flask through USB serial.
    """

    payload = lora.listen(100)

    if payload:
        parsed_data = parse_lora_data(payload)

        if parsed_data is not None:

            # Validate received data
            if validate_received_data(parsed_data):

                # Send JSON to Flask
                print(json.dumps(parsed_data))


# =============================================================================
# FUNCTION 5: HANDLE DATA RECEIVED FROM FLASK
# =============================================================================

def handle_server_receive():
    """
    Reads incoming JSON commands from Flask through USB serial.
    Formats the data and transmits it through LoRa.
    """

    if poll.poll(0):

        line = sys.stdin.readline().strip()

        if not line:
            return

        try:
            server_data = json.loads(line)

            # Format server data for LoRa
            tx_message = format_server_data(server_data)

            if tx_message is not None:

                # Transmit through LoRa
                lora.println(tx_message)

                # Return to receiving mode
                lora.receive()

        except ValueError:
            pass


# =============================================================================
# MAIN LOOP
# =============================================================================

poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

while True:

    # Receive and forward LoRa data to Flask
    handle_lora_receive()

    # Receive commands from Flask and transmit through LoRa
    handle_server_receive()

    time.sleep_ms(10)
