from machine import Pin, PWM, UART
import time

# =========================================================
# MOTOR
# =========================================================

IN1 = Pin(10, Pin.OUT)
IN2 = Pin(15, Pin.OUT)

EN = PWM(Pin(11))
EN.freq(1000)

# Clockwise
IN1.value(1)
IN2.value(0)

# 100% PWM
EN.duty_u16(65535)

print("Motor ON")


# =========================================================
# LIDAR UART
# =========================================================

uart = UART(
    0,
    baudrate=115200,
    bits=8,
    parity=None,
    stop=1,
    rx=Pin(1)
)

print("LiDAR starting...")
print("Listening for AD measurement frames...")


# =========================================================
# PARSE FRAME
# =========================================================

def parse_frame(frame):

    if len(frame) < 13:
        return

    if frame[0] != 0xAA:
        return

    if frame[5] != 0xAD:
        return

    # Payload length
    payload_length = (frame[6] << 8) | frame[7]

    payload = frame[8:8 + payload_length]

    if len(payload) < 5:
        return

    # -----------------------------------------------------
    # RPM
    # -----------------------------------------------------

    rpm_byte = payload[0]
    rpm = rpm_byte * 3

    # -----------------------------------------------------
    # Offset angle
    # -----------------------------------------------------

    offset_raw = (payload[1] << 8) | payload[2]

    if offset_raw >= 32768:
        offset_raw -= 65536

    offset_angle = offset_raw / 100.0

    # -----------------------------------------------------
    # Start angle
    # -----------------------------------------------------

    start_raw = (payload[3] << 8) | payload[4]
    start_angle = start_raw / 100.0

    # -----------------------------------------------------
    # Measurements
    # -----------------------------------------------------

    measurement_data = payload[5:]

    sample_count = len(measurement_data) // 3

    print()
    print("-----------------------------")
    print("RPM:", rpm)
    print("Offset:", offset_angle)
    print("Start angle:", start_angle)
    print("Samples:", sample_count)
    print("-----------------------------")

    for i in range(sample_count):

        index = i * 3

        quality = measurement_data[index]

        distance_raw = (
            (measurement_data[index + 1] << 8)
            | measurement_data[index + 2]
        )

        # 0.25 mm per unit
        distance_m = distance_raw * 0.00025

        # TEMPORARY angle calculation
        # We will verify this from your packets.
        angle = start_angle + (i * 22.5 / sample_count)

        angle %= 360.0

        if distance_raw == 0:
            continue

        print(
            "{:7.2f} deg   {:6.3f} m   Q={:3d}".format(
                angle,
                distance_m,
                quality
            )
        )


# =========================================================
# MAIN LOOP
# =========================================================

buffer = bytearray()

while True:

    if uart.any():

        incoming = uart.read()

        if incoming:
            buffer.extend(incoming)

    # Look for a complete frame
    while len(buffer) >= 8:

        # Find AA
        if buffer[0] != 0xAA:

            buffer = buffer[1:]
            continue

        # Frame length
        frame_length = (
            (buffer[1] << 8)
            | buffer[2]
        )

        total_length = frame_length + 3

        # Not enough data yet
        if len(buffer) < total_length:
            break

        # Extract frame
        frame = buffer[:total_length]

        # Keep remaining data
        buffer = buffer[total_length:]

        # Only AD measurement packets
        if len(frame) > 5 and frame[5] == 0xAD:
            parse_frame(frame)

    time.sleep_ms(2)
