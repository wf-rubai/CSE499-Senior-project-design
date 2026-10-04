from machine import UART, Pin
import time

# ============================================================
# LIDAR UART
# ============================================================

uart = UART(
    0,
    baudrate=115200,
    bits=8,
    parity=None,
    stop=1,
    rx=Pin(1)
)

print("LiDAR robust parser")
print("Listening...")


# ============================================================
# PARSER
# ============================================================

buffer = bytearray()

packet_count = 0
sample_count = 0
invalid_count = 0

last_report = time.ticks_ms()


def parse_ad_frame(frame):

    global packet_count
    global sample_count
    global invalid_count

    # --------------------------------------------------------
    # Basic frame validation
    # --------------------------------------------------------

    if len(frame) < 13:
        return

    if frame[0] != 0xAA:
        return

    # AD measurement packet
    if frame[5] != 0xAD:
        return

    # --------------------------------------------------------
    # Payload length
    # --------------------------------------------------------

    payload_length = (
        (frame[6] << 8)
        | frame[7]
    )

    if payload_length < 5:
        return

    payload_end = 8 + payload_length

    if payload_end > len(frame):
        return

    payload = frame[8:payload_end]

    # --------------------------------------------------------
    # Packet information
    # --------------------------------------------------------

    rpm_value = payload[0]
    rpm = rpm_value * 3

    offset_raw = (
        (payload[1] << 8)
        | payload[2]
    )

    if offset_raw >= 32768:
        offset_raw -= 65536

    offset_angle = offset_raw / 100.0

    start_raw = (
        (payload[3] << 8)
        | payload[4]
    )

    start_angle = start_raw / 100.0

    # --------------------------------------------------------
    # Measurement samples
    # --------------------------------------------------------

    measurement_data = payload[5:]

    sample_count_packet = len(measurement_data) // 3

    if sample_count_packet == 0:
        return

    packet_count += 1
    sample_count += sample_count_packet

    # Your current interpretation:
    # each packet covers 22.5 degrees

    sector_angle = 22.5

    for i in range(sample_count_packet):

        index = i * 3

        quality = measurement_data[index]

        distance_raw = (
            (measurement_data[index + 1] << 8)
            | measurement_data[index + 2]
        )

        # Zero distance = invalid/no measurement
        if distance_raw == 0:
            invalid_count += 1
            continue

        distance_m = distance_raw * 0.00025

        angle = (
            start_angle
            + (i * sector_angle / sample_count_packet)
        )

        angle %= 360.0

        # Send clean machine-readable data
        print(
            "POINT,{:.2f},{:.3f}".format(
                angle,
                distance_m
            )
        )


# ============================================================
# MAIN STREAM PARSER
# ============================================================

while True:

    # --------------------------------------------------------
    # Read everything currently available
    # --------------------------------------------------------

    if uart.any():

        incoming = uart.read()

        if incoming:
            buffer.extend(incoming)

    # --------------------------------------------------------
    # Find complete frames
    # --------------------------------------------------------

    while True:

        # Need at least header + length
        if len(buffer) < 3:
            break

        # ----------------------------------------------------
        # Synchronize to AA
        # ----------------------------------------------------

        if buffer[0] != 0xAA:
            
            """

            # Find next AA instead of deleting only one byte
            try:
                next_header = buffer.index(0xAA, 1)

                buffer = buffer[next_header:]

            except ValueError:

                # No AA found.
                # Keep last byte in case it becomes
                # the beginning of a new frame.
                buffer = buffer[-1:]
            """
            
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

        # ----------------------------------------------------
        # Read frame length
        # ----------------------------------------------------

        frame_length = (
            (buffer[1] << 8)
            | buffer[2]
        )

        # The length field represents everything
        # after the first 3 bytes.
        total_length = frame_length + 3

        # Sanity check
        if total_length < 8 or total_length > 512:

            # Bad length.
            # Throw away this AA and resynchronize.
            buffer = buffer[1:]

            continue

        # ----------------------------------------------------
        # Wait for complete frame
        # ----------------------------------------------------

        if len(buffer) < total_length:
            break

        # ----------------------------------------------------
        # Extract frame
        # ----------------------------------------------------

        frame = buffer[:total_length]

        # Remove it from buffer
        buffer = buffer[total_length:]

        # ----------------------------------------------------
        # Process AD packets
        # ----------------------------------------------------

        if len(frame) > 5:

            if frame[5] == 0xAD:

                parse_ad_frame(frame)

    # --------------------------------------------------------
    # Status report
    # --------------------------------------------------------

    now = time.ticks_ms()

    if time.ticks_diff(now, last_report) >= 1000:

        print()
        print("==============================")
        print("AD packets/sec:", packet_count)
        print("Samples/sec:", sample_count)
        print("Invalid:", invalid_count)
        print("==============================")

        packet_count = 0
        sample_count = 0
        invalid_count = 0

        last_report = now

    time.sleep_ms(1)
    
    
#  1  start= 225.00  samples=17  rpm=456  offset= -0.53
#  2  start= 270.00  samples=17  rpm=456  offset= -0.53
#  3  start= 315.00  samples=17  rpm=459  offset= -0.53
#  4  start=   0.00  samples=17  rpm=459  offset= -0.53
#  5  start=  45.00  samples=17  rpm=459  offset= -0.53
#  6  start=  90.00  samples=17  rpm=456  offset= -0.53
#  7  start= 135.00  samples=17  rpm=459  offset= -0.53
#  8  start= 180.00  samples=17  rpm=456  offset= -0.53
#  9  start= 225.00  samples=17  rpm=453  offset= -0.53
# 10  start= 270.00  samples=17  rpm=456  offset= -0.53
# 11  start= 315.00  samples=17  rpm=456  offset= -0.53
# 12  start=   0.00  samples=17  rpm=459  offset= -0.53
# 13  start=  45.00  samples=17  rpm=459  offset= -0.53
# 14  start=  90.00  samples=17  rpm=456  offset= -0.53
# 15  start= 135.00  samples=17  rpm=459  offset= -0.53
# 16  start= 180.00  samples=17  rpm=456  offset= -0.53
# 17  start= 225.00  samples=17  rpm=456  offset= -0.53
# 18  start= 270.00  samples=17  rpm=456  offset= -0.53
# 19  start= 315.00  samples=17  rpm=456  offset= -0.53
# 20  start=   0.00  samples=17  rpm=459  offset= -0.53
# 21  start=  45.00  samples=17  rpm=459  offset= -0.53
# 22  start=  90.00  samples=17  rpm=456  offset= -0.53
# 23  start= 135.00  samples=17  rpm=459  offset= -0.53
# 24  start= 180.00  samples=17  rpm=456  offset= -0.53
# 25  start= 225.00  samples=17  rpm=456  offset= -0.53
# 26  start= 270.00  samples=17  rpm=456  offset= -0.53
# 27  start= 315.00  samples=17  rpm=456  offset= -0.53
# 28  start=   0.00  samples=17  rpm=459  offset= -0.53
# 29  start=  45.00  samples=17  rpm=459  offset= -0.53
# 30  start=  90.00  samples=17  rpm=456  offset= -0.53
# 31  start= 135.00  samples=17  rpm=459  offset= -0.53
# 32  start= 180.00  samples=17  rpm=456  offset= -0.53
# 33  start= 225.00  samples=17  rpm=456  offset= -0.53
# 34  start= 270.00  samples=17  rpm=456  offset= -0.53
# 35  start= 315.00  samples=17  rpm=459  offset= -0.53
# 36  start=   0.00  samples=17  rpm=456  offset= -0.53
# 37  start=  45.00  samples=17  rpm=459  offset= -0.53
# 38  start=  90.00  samples=17  rpm=456  offset= -0.53
# 39  start= 135.00  samples=17  rpm=459  offset= -0.53
# 40  start= 180.00  samples=17  rpm=456  offset= -0.53
# 41  start= 225.00  samples=17  rpm=456  offset= -0.53
# 42  start= 270.00  samples=17  rpm=459  offset= -0.53
# 43  start= 315.00  samples=17  rpm=459  offset= -0.53
# 44  start=   0.00  samples=17  rpm=459  offset= -0.53
# 45  start=  45.00  samples=17  rpm=456  offset= -0.53
# 46  start=  90.00  samples=17  rpm=456  offset= -0.53
# 47  start= 135.00  samples=17  rpm=456  offset= -0.53
# 48  start= 180.00  samples=17  rpm=456  offset= -0.53
# 49  start= 225.00  samples=17  rpm=456  offset= -0.53
# 50  start= 270.00  samples=17  rpm=456  offset= -0.53
# 51  start= 315.00  samples=17  rpm=459  offset= -0.53
# 52  start=   0.00  samples=17  rpm=459  offset= -0.53
# 53  start=  45.00  samples=17  rpm=459  offset= -0.53
# 54  start=  90.00  samples=17  rpm=456  offset= -0.53
# 55  start= 135.00  samples=17  rpm=456  offset= -0.53
# 56  start= 180.00  samples=17  rpm=456  offset= -0.53
# 57  start= 225.00  samples=17  rpm=456  offset= -0.53
# 58  start= 270.00  samples=17  rpm=456  offset= -0.53
# 59  start= 315.00  samples=17  rpm=459  offset= -0.53
# 60  start=   0.00  samples=17  rpm=459  offset= -0.53
# 61  start=  45.00  samples=17  rpm=459  offset= -0.53
# 62  start=  90.00  samples=17  rpm=456  offset= -0.53
# 63  start= 135.00  samples=17  rpm=456  offset= -0.53
# 64  start= 180.00  samples=17  rpm=453  offset= -0.53
# 65  start= 225.00  samples=17  rpm=456  offset= -0.53
# 66  start= 270.00  samples=17  rpm=456  offset= -0.53
# 67  start= 315.00  samples=17  rpm=459  offset= -0.53
# 68  start=   0.00  samples=17  rpm=459  offset= -0.53
# 69  start=  45.00  samples=17  rpm=459  offset= -0.53
# 70  start=  90.00  samples=17  rpm=456  offset= -0.53
# 71  start= 135.00  samples=17  rpm=456  offset= -0.53
# 72  start= 180.00  samples=17  rpm=456  offset= -0.53
# 73  start= 225.00  samples=17  rpm=453  offset= -0.53
# 74  start= 270.00  samples=17  rpm=456  offset= -0.53
# 75  start= 315.00  samples=17  rpm=456  offset= -0.53
# 76  start=   0.00  samples=17  rpm=459  offset= -0.53
# 77  start=  45.00  samples=17  rpm=459  offset= -0.53
# 78  start=  90.00  samples=17  rpm=456  offset= -0.53
# 79  start= 135.00  samples=17  rpm=459  offset= -0.53
# 80  start= 180.00  samples=17  rpm=456  offset= -0.53
# 81  start= 225.00  samples=17  rpm=456  offset= -0.53
# 82  start= 270.00  samples=17  rpm=453  offset= -0.53
# 83  start= 315.00  samples=17  rpm=456  offset= -0.53
# 84  start=   0.00  samples=17  rpm=459  offset= -0.53
# 85  start=  45.00  samples=17  rpm=459  offset= -0.53
# 86  start=  90.00  samples=17  rpm=456  offset= -0.53
# 87  start= 135.00  samples=17  rpm=459  offset= -0.53
# 88  start= 180.00  samples=17  rpm=456  offset= -0.53
# 89  start= 225.00  samples=17  rpm=456  offset= -0.53
# 90  start= 270.00  samples=17  rpm=456  offset= -0.53
# 91  start= 315.00  samples=17  rpm=456  offset= -0.53
# 92  start=   0.00  samples=17  rpm=456  offset= -0.53
# 93  start=  45.00  samples=17  rpm=459  offset= -0.53
# 94  start=  90.00  samples=17  rpm=456  offset= -0.53
# 95  start= 135.00  samples=17  rpm=459  offset= -0.53
# 96  start= 180.00  samples=17  rpm=456  offset= -0.53
# 97  start= 225.00  samples=17  rpm=456  offset= -0.53
# 98  start= 270.00  samples=17  rpm=456  offset= -0.53
# 99  start= 315.00  samples=17  rpm=456  offset= -0.53
#100  start=   0.00  samples=17  rpm=456  offset= -0.53
#101  start=  45.00  samples=17  rpm=456  offset= -0.53
#102  start=  90.00  samples=17  rpm=456  offset= -0.53
#103  start= 135.00  samples=17  rpm=456  offset= -0.53
#104  start= 180.00  samples=17  rpm=456  offset= -0.53
#105  start= 225.00  samples=17  rpm=456  offset= -0.53
#106  start= 270.00  samples=17  rpm=456  offset= -0.53
#107  start= 315.00  samples=17  rpm=459  offset= -0.53
#108  start=   0.00  samples=17  rpm=459  offset= -0.53
#109  start=  45.00  samples=17  rpm=456  offset= -0.53
#110  start=  90.00  samples=17  rpm=453  offset= -0.53
#111  start= 135.00  samples=17  rpm=456  offset= -0.53
#112  start= 180.00  samples=17  rpm=456  offset= -0.53
#113  start= 225.00  samples=17  rpm=456  offset= -0.53
#114  start= 270.00  samples=17  rpm=456  offset= -0.53
#115  start= 315.00  samples=17  rpm=459  offset= -0.53
#116  start=   0.00  samples=17  rpm=459  offset= -0.53
#117  start=  45.00  samples=17  rpm=459  offset= -0.53
#118  start=  90.00  samples=17  rpm=456  offset= -0.53
#119  start= 135.00  samples=17  rpm=456  offset= -0.53
#120  start= 180.00  samples=17  rpm=456  offset= -0.53
#121  start= 225.00  samples=17  rpm=456  offset= -0.53
#122  start= 270.00  samples=17  rpm=456  offset= -0.53
#123  start= 315.00  samples=17  rpm=456  offset= -0.53
#124  start=   0.00  samples=17  rpm=459  offset= -0.53
#125  start=  45.00  samples=17  rpm=459  offset= -0.53
#126  start=  90.00  samples=17  rpm=456  offset= -0.53
#127  start= 135.00  samples=17  rpm=456  offset= -0.53
#128  start= 180.00  samples=17  rpm=453  offset= -0.53
#129  start= 225.00  samples=17  rpm=453  offset= -0.53
#130  start= 270.00  samples=17  rpm=456  offset= -0.53
#131  start= 315.00  samples=17  rpm=456  offset= -0.53
#132  start=   0.00  samples=17  rpm=459  offset= -0.53
#133  start=  45.00  samples=17  rpm=459  offset= -0.53
#134  start=  90.00  samples=17  rpm=456  offset= -0.53
#135  start= 135.00  samples=17  rpm=456  offset= -0.53
#136  start= 180.00  samples=17  rpm=456  offset= -0.53
#137  start= 225.00  samples=17  rpm=453  offset= -0.53
#138  start= 270.00  samples=17  rpm=456  offset= -0.53
#139  start= 315.00  samples=17  rpm=456  offset= -0.53
#140  start=   0.00  samples=17  rpm=459  offset= -0.53
#141  start=  45.00  samples=17  rpm=459  offset= -0.53
#142  start=  90.00  samples=17  rpm=456  offset= -0.53
#143  start= 135.00  samples=17  rpm=459  offset= -0.53
#144  start= 180.00  samples=17  rpm=456  offset= -0.53
#145  start= 225.00  samples=17  rpm=456  offset= -0.53
#146  start= 270.00  samples=17  rpm=453  offset= -0.53
#147  start= 315.00  samples=17  rpm=456  offset= -0.53
#148  start=   0.00  samples=17  rpm=459  offset= -0.53
#149  start=  45.00  samples=17  rpm=459  offset= -0.53
#150  start=  90.00  samples=17  rpm=456  offset= -0.53
#151  start= 135.00  samples=17  rpm=459  offset= -0.53
#152  start= 180.00  samples=17  rpm=456  offset= -0.53
#153  start= 225.00  samples=17  rpm=456  offset= -0.53
#154  start= 270.00  samples=17  rpm=456  offset= -0.53
#155  start= 315.00  samples=17  rpm=456  offset= -0.53
#156  start=   0.00  samples=17  rpm=456  offset= -0.53
#157  start=  45.00  samples=17  rpm=459  offset= -0.53
#158  start=  90.00  samples=17  rpm=456  offset= -0.53
#159  start= 135.00  samples=17  rpm=456  offset= -0.53
#160  start= 180.00  samples=17  rpm=456  offset= -0.53
#161  start= 225.00  samples=17  rpm=456  offset= -0.53
#162  start= 270.00  samples=17  rpm=456  offset= -0.53
#163  start= 315.00  samples=17  rpm=456  offset= -0.53
#164  start=   0.00  samples=17  rpm=456  offset= -0.53
#165  start=  45.00  samples=17  rpm=456  offset= -0.53
#166  start=  90.00  samples=17  rpm=456  offset= -0.53
#167  start= 135.00  samples=17  rpm=456  offset= -0.53
#168  start= 180.00  samples=17  rpm=456  offset= -0.53
#169  start= 225.00  samples=17  rpm=456  offset= -0.53
#170  start= 270.00  samples=17  rpm=459  offset= -0.53
#171  start= 315.00  samples=17  rpm=459  offset= -0.53
#172  start=   0.00  samples=17  rpm=459  offset= -0.53
#173  start=  45.00  samples=17  rpm=456  offset= -0.53
#174  start=  90.00  samples=17  rpm=456  offset= -0.53
#175  start= 135.00  samples=17  rpm=456  offset= -0.53
#176  start= 180.00  samples=17  rpm=456  offset= -0.53
#177  start= 225.00  samples=17  rpm=456  offset= -0.53
#178  start= 270.00  samples=17  rpm=456  offset= -0.53
#179  start= 315.00  samples=17  rpm=459  offset= -0.53
#180  start=   0.00  samples=17  rpm=459  offset= -0.53
#181  start=  45.00  samples=17  rpm=459  offset= -0.53
#182  start=  90.00  samples=17  rpm=456  offset= -0.53
#183  start= 135.00  samples=17  rpm=456  offset= -0.53
#184  start= 180.00  samples=17  rpm=456  offset= -0.53
#185  start= 225.00  samples=17  rpm=456  offset= -0.53
#186  start= 270.00  samples=17  rpm=456  offset= -0.53
#187  start= 315.00  samples=17  rpm=456  offset= -0.53
#188  start=   0.00  samples=17  rpm=459  offset= -0.53
#189  start=  45.00  samples=17  rpm=459  offset= -0.53
#190  start=  90.00  samples=17  rpm=456  offset= -0.53
#191  start= 135.00  samples=17  rpm=456  offset= -0.53
#192  start= 180.00  samples=17  rpm=453  offset= -0.53
#193  start= 225.00  samples=17  rpm=456  offset= -0.53
#194  start= 270.00  samples=17  rpm=456  offset= -0.53
#195  start= 315.00  samples=17  rpm=456  offset= -0.53
#196  start=   0.00  samples=17  rpm=459  offset= -0.53
#197  start=  45.00  samples=17  rpm=459  offset= -0.53
#198  start=  90.00  samples=17  rpm=456  offset= -0.53
#199  start= 135.00  samples=17  rpm=456  offset= -0.53
#200  start= 180.00  samples=17  rpm=456  offset= -0.53
#201  start= 225.00  samples=17  rpm=453  offset= -0.53
#202  start= 270.00  samples=17  rpm=456  offset= -0.53
#203  start= 315.00  samples=17  rpm=456  offset= -0.53
#204  start=   0.00  samples=17  rpm=459  offset= -0.53
#205  start=  45.00  samples=17  rpm=459  offset= -0.53
#206  start=  90.00  samples=17  rpm=456  offset= -0.53
#207  start= 135.00  samples=17  rpm=459  offset= -0.53
#208  start= 180.00  samples=17  rpm=456  offset= -0.53
#209  start= 225.00  samples=17  rpm=456  offset= -0.53
#210  start= 270.00  samples=17  rpm=453  offset= -0.53
#211  start= 315.00  samples=17  rpm=456  offset= -0.53
#212  start=   0.00  samples=17  rpm=456  offset= -0.53
#213  start=  45.00  samples=17  rpm=459  offset= -0.53
#214  start=  90.00  samples=17  rpm=456  offset= -0.53
#215  start= 135.00  samples=17  rpm=456  offset= -0.53
#216  start= 180.00  samples=17  rpm=456  offset= -0.53
#217  start= 225.00  samples=17  rpm=456  offset= -0.53
#218  start= 270.00  samples=17  rpm=456  offset= -0.53
#219  start= 315.00  samples=17  rpm=453  offset= -0.53
#220  start=   0.00  samples=17  rpm=456  offset= -0.53
#221  start=  45.00  samples=17  rpm=456  offset= -0.53
#222  start=  90.00  samples=17  rpm=456  offset= -0.53
#223  start= 135.00  samples=17  rpm=456  offset= -0.53
#224  start= 180.00  samples=17  rpm=456  offset= -0.53
#225  start= 225.00  samples=17  rpm=456  offset= -0.53
#226  start= 270.00  samples=17  rpm=456  offset= -0.53
#227  start= 315.00  samples=17  rpm=456  offset= -0.53
#228  start=   0.00  samples=17  rpm=456  offset= -0.53
#229  start=  45.00  samples=17  rpm=456  offset= -0.53
#230  start=  90.00  samples=17  rpm=453  offset= -0.53
#231  start= 135.00  samples=17  rpm=456  offset= -0.53
#232  start= 180.00  samples=17  rpm=456  offset= -0.53
#233  start= 225.00  samples=17  rpm=456  offset= -0.53
#234  start= 270.00  samples=17  rpm=456  offset= -0.53
#235  start= 315.00  samples=17  rpm=459  offset= -0.53
#236  start=   0.00  samples=17  rpm=459  offset= -0.53
#237  start=  45.00  samples=17  rpm=456  offset= -0.53
#238  start=  90.00  samples=17  rpm=453  offset= -0.53
#239  start= 135.00  samples=17  rpm=456  offset= -0.53
#240  start= 180.00  samples=17  rpm=456  offset= -0.53
#241  start= 225.00  samples=17  rpm=456  offset= -0.53
#242  start= 270.00  samples=17  rpm=456  offset= -0.53
#243  start= 315.00  samples=17  rpm=459  offset= -0.53
#244  start=   0.00  samples=17  rpm=459  offset= -0.53
#245  start=  45.00  samples=17  rpm=459  offset= -0.53
#246  start=  90.00  samples=17  rpm=456  offset= -0.53
#247  start= 135.00  samples=17  rpm=456  offset= -0.53
#248  start= 180.00  samples=17  rpm=456  offset= -0.53
#249  start= 225.00  samples=17  rpm=456  offset= -0.53
#250  start= 270.00  samples=17  rpm=456  offset= -0.53
#251  start= 315.00  samples=17  rpm=456  offset= -0.53
#252  start=   0.00  samples=17  rpm=459  offset= -0.53
#253  start=  45.00  samples=17  rpm=459  offset= -0.53
#254  start=  90.00  samples=17  rpm=456  offset= -0.53
#255  start= 135.00  samples=17  rpm=456  offset= -0.53
#256  start= 180.00  samples=17  rpm=453  offset= -0.53
#257  start= 225.00  samples=17  rpm=453  offset= -0.53
#258  start= 270.00  samples=17  rpm=456  offset= -0.53
#259  start= 315.00  samples=17  rpm=456  offset= -0.53
#260  start=   0.00  samples=17  rpm=459  offset= -0.53
#261  start=  45.00  samples=17  rpm=459  offset= -0.53
#262  start=  90.00  samples=17  rpm=456  offset= -0.53
#263  start= 135.00  samples=17  rpm=456  offset= -0.53
#264  start= 180.00  samples=17  rpm=456  offset= -0.53
#265  start= 225.00  samples=17  rpm=453  offset= -0.53
#266  start= 270.00  samples=17  rpm=456  offset= -0.53
#267  start= 315.00  samples=17  rpm=456  offset= -0.53
#268  start=   0.00  samples=17  rpm=459  offset= -0.53
#269  start=  45.00  samples=17  rpm=459  offset= -0.53
#270  start=  90.00  samples=17  rpm=456  offset= -0.53
#271  start= 135.00  samples=17  rpm=459  offset= -0.53
#272  start= 180.00  samples=17  rpm=456  offset= -0.53
#273  start= 225.00  samples=17  rpm=456  offset= -0.53
#274  start= 270.00  samples=17  rpm=453  offset= -0.53
#275  start= 315.00  samples=17  rpm=456  offset= -0.53
#276  start=   0.00  samples=17  rpm=456  offset= -0.53
#277  start=  45.00  samples=17  rpm=459  offset= -0.53
#278  start=  90.00  samples=17  rpm=456  offset= -0.53
#279  start= 135.00  samples=17  rpm=456  offset= -0.53
#280  start= 180.00  samples=17  rpm=456  offset= -0.53
#281  start= 225.00  samples=17  rpm=456  offset= -0.53
#282  start= 270.00  samples=17  rpm=456  offset= -0.53
#283  start= 315.00  samples=17  rpm=453  offset= -0.53
#284  start=   0.00  samples=17  rpm=456  offset= -0.53
#285  start=  45.00  samples=17  rpm=456  offset= -0.53
#286  start=  90.00  samples=17  rpm=456  offset= -0.53
#287  start= 135.00  samples=17  rpm=456  offset= -0.53
#288  start= 180.00  samples=17  rpm=456  offset= -0.53
#289  start= 225.00  samples=17  rpm=456  offset= -0.53
#290  start= 270.00  samples=17  rpm=456  offset= -0.53
#291  start= 315.00  samples=17  rpm=456  offset= -0.53
#292  start=   0.00  samples=17  rpm=456  offset= -0.53
#293  start=  45.00  samples=17  rpm=456  offset= -0.53
#294  start=  90.00  samples=17  rpm=453  offset= -0.53
#295  start= 135.00  samples=17  rpm=456  offset= -0.53
#296  start= 180.00  samples=17  rpm=456  offset= -0.53
#297  start= 225.00  samples=17  rpm=456  offset= -0.53
#298  start= 270.00  samples=17  rpm=456  offset= -0.53
#299  start= 315.00  samples=17  rpm=456  offset= -0.53
#300  start=   0.00  samples=17  rpm=456  offset= -0.53