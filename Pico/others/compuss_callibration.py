from machine import I2C, Pin
import time
import json


# =========================================================
# I2C
# =========================================================

i2c = I2C(
    0,
    scl=Pin(5),
    sda=Pin(4),
    freq=100000
)

MAG_ADDR = 0x1E


# =========================================================
# HMC5883L REGISTERS
# =========================================================

CONFIG_A = 0x00
CONFIG_B = 0x01
MODE = 0x02
DATA = 0x03


# =========================================================
# HELPERS
# =========================================================

def write_reg(reg, value):
    i2c.writeto_mem(
        MAG_ADDR,
        reg,
        bytes([value])
    )


def read_reg(reg, length):
    return i2c.readfrom_mem(
        MAG_ADDR,
        reg,
        length
    )


def signed16(high, low):

    value = (high << 8) | low

    if value & 0x8000:
        value -= 65536

    return value


# =========================================================
# INITIALIZE HMC5883L
# =========================================================

# 8 samples averaged
# 15 Hz output
# Normal measurement

write_reg(CONFIG_A, 0x70)

# Gain = ±1.3 Gauss

write_reg(CONFIG_B, 0x20)

# Continuous measurement

write_reg(MODE, 0x00)

time.sleep_ms(100)


# =========================================================
# READ MAGNETOMETER
# =========================================================

def read_mag():

    data = read_reg(DATA, 6)

    # HMC5883L order:
    #
    # X
    # Z
    # Y

    mx = signed16(data[0], data[1])
    mz = signed16(data[2], data[3])
    my = signed16(data[4], data[5])

    return mx, my, mz


# =========================================================
# START
# =========================================================

print()
print("==============================")
print(" HMC5883L CALIBRATION")
print("==============================")
print()

print("Move the sensor through")
print("MANY different orientations.")
print()
print("Rotate around X, Y and Z.")
print()
print("Do NOT just rotate it flat.")
print()

for i in range(5, 0, -1):

    print("Starting in", i)

    time.sleep(1)


# =========================================================
# COLLECT DATA
# =========================================================

print()
print("CALIBRATING...")
print()

min_x = 999999
min_y = 999999
min_z = 999999

max_x = -999999
max_y = -999999
max_z = -999999


# 60 seconds

start = time.ticks_ms()

while time.ticks_diff(time.ticks_ms(), start) < 60000:

    mx, my, mz = read_mag()

    min_x = min(min_x, mx)
    min_y = min(min_y, my)
    min_z = min(min_z, mz)

    max_x = max(max_x, mx)
    max_y = max(max_y, my)
    max_z = max(max_z, mz)

    time.sleep_ms(20)


# =========================================================
# CALCULATE OFFSETS
# =========================================================

offset_x = (max_x + min_x) / 2
offset_y = (max_y + min_y) / 2
offset_z = (max_z + min_z) / 2


# =========================================================
# CALCULATE SCALE
# =========================================================

range_x = (max_x - min_x) / 2
range_y = (max_y - min_y) / 2
range_z = (max_z - min_z) / 2

average_range = (
    range_x +
    range_y +
    range_z
) / 3


scale_x = average_range / range_x
scale_y = average_range / range_y
scale_z = average_range / range_z


# =========================================================
# PRINT RESULTS
# =========================================================

print()
print("==============================")
print("MAGNETOMETER RESULTS")
print("==============================")

print()
print("Minimum:")
print("X =", min_x)
print("Y =", min_y)
print("Z =", min_z)

print()
print("Maximum:")
print("X =", max_x)
print("Y =", max_y)
print("Z =", max_z)

print()
print("Offsets:")
print("X =", offset_x)
print("Y =", offset_y)
print("Z =", offset_z)

print()
print("Scale:")
print("X =", scale_x)
print("Y =", scale_y)
print("Z =", scale_z)


# =========================================================
# LOAD EXISTING CALIBRATION
# =========================================================

try:

    with open("calibration.json", "r") as f:

        calibration = json.load(f)

    print()
    print("Existing calibration.json found.")

except:

    calibration = {}

    print()
    print("No existing calibration found.")


# =========================================================
# ADD MAGNETOMETER CALIBRATION
# =========================================================

calibration["mag"] = {
    "offset": [
        offset_x,
        offset_y,
        offset_z
    ],

    "scale": [
        scale_x,
        scale_y,
        scale_z
    ]
}


# =========================================================
# SAVE
# =========================================================

with open("calibration.json", "w") as f:

    json.dump(
        calibration,
        f
    )


print()
print("==============================")
print("CALIBRATION SAVED")
print("==============================")
print()
print("File: calibration.json")
print()
print("Your ACC + GYRO + MAG")
print("calibration is now stored.")
