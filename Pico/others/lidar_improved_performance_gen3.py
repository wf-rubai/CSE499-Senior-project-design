from machine import Pin, PWM, UART
import time

# ============================================================
# MOTOR
# ============================================================

motor = PWM(Pin(2))
motor.freq(1000)

PWM_VALUE = 50000
motor.duty_u16(PWM_VALUE)

# ============================================================
# UART
# ============================================================

uart = UART(
    0,
    baudrate=115200,
    bits=8,
    parity=None,
    stop=1,
    rx=Pin(1)
)

buffer = bytearray()

# ============================================================
# STATISTICS
# ============================================================

sample_count = 0
zero_count = 0
invalid_count = 0
nonzero_count = 0

distance_sum = 0
min_distance = None
max_distance = None

ad_count = 0
current_rpm = 0

last_report = time.ticks_ms()


# ============================================================
# PROCESS AD PACKET
# ============================================================

def process_ad_frame(frame):

    global sample_count
    global zero_count
    global invalid_count
    global nonzero_count
    global distance_sum
    global min_distance
    global max_distance
    global ad_count
    global current_rpm

    if len(frame) < 16:
        return

    if frame[5] != 0xAD:
        return

    ad_count += 1

    # RPM
    current_rpm = frame[8] * 3

    # --------------------------------------------------------
    # Samples
    #
    # Each sample:
    #
    # byte 0 = quality
    # byte 1 = distance high
    # byte 2 = distance low
    # --------------------------------------------------------

    sample_start = 13
    sample_bytes = len(frame) - sample_start
    samples = sample_bytes // 3

    if samples <= 0:
        return

    for i in range(samples):

        p = sample_start + i * 3

        if p + 2 >= len(frame):
            break

        quality = frame[p]

        distance_raw = (
            (frame[p + 1] << 8)
            | frame[p + 2]
        )

        distance_mm = distance_raw * 0.25

        sample_count += 1

        # ----------------------------------------------------
        # Invalid measurement
        # ----------------------------------------------------

        if quality == 0 or distance_raw == 0:

            zero_count += 1

        else:

            nonzero_count += 1

            distance_sum += distance_mm

            if min_distance is None or distance_mm < min_distance:
                min_distance = distance_mm

            if max_distance is None or distance_mm > max_distance:
                max_distance = distance_mm


# ============================================================
# MAIN LOOP
# ============================================================

print("LiDAR summary test started")

while True:

    # --------------------------------------------------------
    # Read UART
    # --------------------------------------------------------

    if uart.any():

        incoming = uart.read()

        if incoming:
            buffer.extend(incoming)

    # --------------------------------------------------------
    # Parse packets
    # --------------------------------------------------------

    while len(buffer) >= 8:

        # Synchronize to AA
        if buffer[0] != 0xAA:

            found = -1

            for i in range(1, len(buffer)):

                if buffer[i] == 0xAA:
                    found = i
                    break

            if found >= 0:
                buffer = buffer[found:]
            else:
                buffer = buffer[-1:]

            continue

        # Protocol check
        if buffer[3] != 0x01:

            buffer = buffer[1:]
            continue

        packet_type = buffer[5]

        # Only AD / AE
        if packet_type not in (0xAD, 0xAE):

            buffer = buffer[1:]
            continue

        # Frame length
        frame_length = (buffer[1] << 8) | buffer[2]

        # Sanity check
        if frame_length < 5 or frame_length > 200:

            buffer = buffer[1:]
            continue

        total_length = frame_length + 3

        # Wait for complete packet
        if len(buffer) < total_length:
            break

        frame = buffer[:total_length]

        buffer = buffer[total_length:]

        # Process AD
        if packet_type == 0xAD:

            process_ad_frame(frame)

    # --------------------------------------------------------
    # Print summary every second
    # --------------------------------------------------------

    now = time.ticks_ms()

    if time.ticks_diff(now, last_report) >= 1000:

        if nonzero_count > 0:
            average_distance = distance_sum / nonzero_count
        else:
            average_distance = 0

        if min_distance is None:
            minimum = 0
        else:
            minimum = min_distance

        if max_distance is None:
            maximum = 0
        else:
            maximum = max_distance

        print(
            "RPM: {} | AD/s: {} | "
            "Samples: {} | Zero: {} | "
            "Non-zero: {} | "
            "Min: {:.0f} mm | "
            "Max: {:.0f} mm | "
            "Avg: {:.0f} mm".format(
                current_rpm,
                ad_count,
                sample_count,
                zero_count,
                nonzero_count,
                minimum,
                maximum,
                average_distance
            )
        )

        # Reset statistics
        sample_count = 0
        zero_count = 0
        invalid_count = 0
        nonzero_count = 0

        distance_sum = 0
        min_distance = None
        max_distance = None

        ad_count = 0

        last_report = now

    time.sleep_ms(1)