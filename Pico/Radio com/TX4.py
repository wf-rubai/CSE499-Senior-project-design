# =============================================================================
# GPS + TX3 Combined — Raspberry Pi Pico (MicroPython)
# Reads GPS coordinates (UART) + sensor inputs, transmits over LoRa (SX127x)
# Payload format (CSV): ROVER_NAME,human_detect,lat,lon,satellites
# =============================================================================
 
from machine import UART, SPI, Pin
from sx127x import SX127x
import time
 
# -----------------------------------------------------------------------------
# Constants
# -----------------------------------------------------------------------------
ROVER_NAME = "RV_0001"
TRANSMIT_INTERVAL_MS = 1000   # Transmit at most once every 2 seconds
 
# -----------------------------------------------------------------------------
# Sensor inputs
# -----------------------------------------------------------------------------
sensor1 = Pin(15, Pin.IN)
sensor2 = Pin(14, Pin.IN)
led = Pin(25, Pin.OUT)
led.value(1)
 
# -----------------------------------------------------------------------------
# UART — GPS module (GP4 = TX, GP5 = RX)
# -----------------------------------------------------------------------------
uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
 
# -----------------------------------------------------------------------------
# SPI + LoRa (SX127x) setup
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
        "tx_power_level": 20,
        "spreading_factor": 10,
        "signal_bandwidth": 125000,
        "coding_rate": 5,
        "preamble_length": 8,
        "sync_word": 0x12,
        "enable_CRC": True,
    },
)
 
print("TX Ready")
print("Waiting for GPS fix...")
 
# -----------------------------------------------------------------------------
# GPS parsing helpers
# -----------------------------------------------------------------------------
 
def nmea_to_decimal(raw_value, direction):
    """Convert NMEA coordinate format (ddmm.mmmm) to decimal degrees."""
    if not raw_value:
        return None
    try:
        value = float(raw_value)
    except ValueError:
        return None
 
    degrees = int(value / 100)
    minutes = value - (degrees * 100)
    decimal = degrees + (minutes / 60)
 
    if direction in ("S", "W"):
        decimal = -decimal
 
    return decimal


def parse_gga(sentence):
    """Parse GGA sentence → (lat, lon, satellites) or None."""
    parts = sentence.split(",")
    if len(parts) < 7:
        return None
 
    fix_quality = parts[6]
    satellites_in_view = parts[7] if len(parts) > 7 else "N/A"
 
    if fix_quality == "0":      # No fix
        return None
 
    lat = nmea_to_decimal(parts[2], parts[3])
    lon = nmea_to_decimal(parts[4], parts[5])
 
    return lat, lon, satellites_in_view
 
 
def parse_rmc(sentence):
    """Parse RMC sentence → (lat, lon) or None."""
    parts = sentence.split(",")
    if len(parts) < 7:
        return None
 
    if parts[2] != "A":         # Invalid status
        return None
 
    lat = nmea_to_decimal(parts[3], parts[4])
    lon = nmea_to_decimal(parts[5], parts[6])
 
    return lat, lon
 
 
# -----------------------------------------------------------------------------
# Transmission helper
# -----------------------------------------------------------------------------
 
def transmit(human_detect, lat, lon, satellites="N/A"):
    # CSV: ROVER_NAME, human_detect, latitude, longitude, satellites
    msg = "{},{},{:.6f},{:.6f},{}".format(
        ROVER_NAME, human_detect, lat, lon, satellites
    )
 
    led.value(1)
    print("Sending:", msg)
    lora.println(msg)
    print("Sent.")
    #time.sleep(0.1)
    led.value(0)
 
 
# -----------------------------------------------------------------------------
# Main loop
# -----------------------------------------------------------------------------
 
last_tx_time = time.ticks_ms()
 
while True:
    if uart.any():
        line = uart.readline()
        if line:
            try:
                sentence = line.decode("utf-8").strip()
 
                # --- GGA (preferred: includes satellite count) ---
                if sentence.startswith("$GPGGA") or sentence.startswith("$GNGGA"):
                    result = parse_gga(sentence)
                    if result:
                        lat, lon, satellites = result
                        val1 = sensor1.value()
                        val2 = sensor2.value()
                        human_detect = 1 if ((val1 == 1 and val2 == 1) or human_detect == 1) else 0
                        print(f"GPS Fix — Lat: {lat:.6f}, Lon: {lon:.6f}, Sats: {satellites}")
 
                        now = time.ticks_ms()
                        if time.ticks_diff(now, last_tx_time) >= TRANSMIT_INTERVAL_MS:
                            transmit(human_detect, lat, lon, satellites)
                            last_tx_time = now
                    else:
                        print("No GPS fix yet...")
 
                # --- RMC (fallback: no satellite count) ---
                elif sentence.startswith("$GPRMC") or sentence.startswith("$GNRMC"):
                    result = parse_rmc(sentence)
                    if result:
                        lat, lon = result
                        print(f"GPS Fix — Lat: {lat:.6f}, Lon: {lon:.6f}")
 
                        now = time.ticks_ms()
                        if time.ticks_diff(now, last_tx_time) >= TRANSMIT_INTERVAL_MS:
                            transmit(0, lat, lon)
                            last_tx_time = now
                    else:
                        print("No valid RMC data.")
 
                else:
                    pass  # Ignore GNVTG, GSA, and other sentences
 
            except Exception as e:
                pass  # Ignore parsing errors and continue
 
    time.sleep_ms(200)