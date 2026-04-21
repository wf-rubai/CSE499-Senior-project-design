from machine import UART, Pin 
import time

# Initialize UART for GPS communication (GP4 is TX, GP5 is RX)
uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))  # GP4 (TX) and GP5 (RX)

def nmea_to_decimal(raw_value, direction):
    """
    Converts NMEA coordinate format (ddmm.mmmm) to decimal degrees.
    """
    if not raw_value:
        return None
    try:
        value = float(raw_value)
    except ValueError:
        return None  # Handle invalid data gracefully
    
    degrees = int(value / 100)
    minutes = value - (degrees * 100)
    decimal = degrees + (minutes / 60)

    if direction in ("S", "W"):
        decimal = -decimal

    return decimal

def parse_gga(sentence):
    """
    Parses the GGA sentence to extract latitude, longitude, and fix status.
    """
    parts = sentence.split(",")
    if len(parts) < 7:
        return None

    fix_quality = parts[6]
    satellites_in_view = parts[7] if len(parts) > 7 else "N/A"
    if fix_quality == "0":  # No fix
        return None

    lat = nmea_to_decimal(parts[2], parts[3])
    lon = nmea_to_decimal(parts[4], parts[5])
    
    return lat, lon, satellites_in_view

def parse_rmc(sentence):
    """
    Parses the RMC sentence to extract latitude and longitude.
    """
    parts = sentence.split(",")
    if len(parts) < 7:
        return None
    
    status = parts[2]
    if status != "A":  # Invalid status
        return None

    lat = nmea_to_decimal(parts[3], parts[4])
    lon = nmea_to_decimal(parts[5], parts[6])
    
    return lat, lon

print("Waiting for GPS fix...")

while True:
    if uart.any():
        line = uart.readline()
        if line:
            try:
                sentence = line.decode("utf-8").strip()

                # Parse GGA sentences (GPS position information)
                if sentence.startswith("$GPGGA") or sentence.startswith("$GNGGA"):
                    result = parse_gga(sentence)
                    if result:
                        lat, lon, satellites = result
                        print(f"Latitude: {lat:.6f}, Longitude: {lon:.6f}")
                        print(f"Satellites in view: {satellites}")
                    else:
                        print("No GPS fix yet...")

                # Parse RMC sentences (Navigation information)
                elif sentence.startswith("$GPRMC") or sentence.startswith("$GNRMC"):
                    result = parse_rmc(sentence)
                    if result:
                        lat, lon = result
                        print(f"Latitude: {lat:.6f}, Longitude: {lon:.6f}")
                    else:
                        print("No valid RMC data.")

                # Ignore other non-GPS sentences like GNVTG, GSA
                else:
                    pass  # Ignore other sentences

            except Exception as e:
                pass  # Ignore parsing errors and continue

    time.sleep_ms(200)  # Delay for a smoother reading