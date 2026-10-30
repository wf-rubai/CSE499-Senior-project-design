# Hardware: [NEO-M8N GPS MODULE]
# Microcontroller: [RASPBERRY PI PICO]
# LED: [ONBOARD LED]

from machine import UART, Pin
import time

# ---------- Hardware Setup ----------
uart = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9))
led = Pin("LED", Pin.OUT)

DEVICE_NAME = "[HARDWARE_NAME]"
HUMAN_COUNT = 0
CSV_FILE = "gps_data_v2.csv"

# Create CSV file with headers if it doesn't exist
try:
    with open(CSV_FILE, "r") as file:
        pass
except OSError:
    with open(CSV_FILE, "w") as file:
        file.write("name,human,lat,lon,satellites\n")


# ---------- Convert NMEA Coordinates ----------
def nmea_to_decimal(raw_value, direction):
    if not raw_value:
        return None

    try:
        value = float(raw_value)
        degrees = int(value / 100)
        minutes = value - degrees * 100
        decimal = degrees + minutes / 60

        if direction in ("S", "W"):
            decimal = -decimal

        return decimal
    except (ValueError, TypeError):
        return None


# ---------- Parse GPS GGA Sentence ----------
def parse_gga(sentence):
    parts = sentence.split(",")

    if len(parts) < 8 or parts[6] == "0":
        return None

    lat = nmea_to_decimal(parts[2], parts[3])
    lon = nmea_to_decimal(parts[4], parts[5])
    satellites = parts[7]

    if lat is None or lon is None:
        return None

    return lat, lon, satellites


# ---------- Main Program ----------
while True:
    if uart.any():
        line = uart.readline()

        if line:
            try:
                sentence = line.decode("utf-8").strip()

                if sentence.startswith(("$GPGGA", "$GNGGA")):
                    result = parse_gga(sentence)

                    if result:
                        lat, lon, satellites = result

                        # Print JSON in fixed key order
                        output = (
                            '{"name": "%s", "human": %d, '
                            '"lat": %.6f, "lon": %.6f, '
                            '"satellites": "%s"}'
                            % (DEVICE_NAME, HUMAN_COUNT, lat, lon, satellites)
                        )
                        print(output)

                        # Append reading to CSV
                        with open(CSV_FILE, "a") as file:
                            file.write(
                                "{},{},{:.6f},{:.6f},{}\n".format(
                                    DEVICE_NAME, HUMAN_COUNT,
                                    lat, lon, satellites
                                )
                            )

                        # Blink LED for each valid GPS fix
                        led.on()
                        time.sleep_ms(100)
                        led.off()

            except Exception:
                pass

    time.sleep_ms(50)
