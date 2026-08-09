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
    freq=400000
)

MPU_ADDR = 0x68


# =========================================================
# MPU6500
# =========================================================

def write_reg(addr, reg, value):
    i2c.writeto_mem(addr, reg, bytes([value]))


def read_reg(addr, reg, length=1):
    return i2c.readfrom_mem(addr, reg, length)


def signed16(high, low):

    value = (high << 8) | low

    if value & 0x8000:
        value -= 65536

    return value


# Wake MPU6500
write_reg(MPU_ADDR, 0x6B, 0x00)

time.sleep_ms(100)

# Gyro ±250 deg/s
write_reg(MPU_ADDR, 0x1B, 0x00)

# Accelerometer ±2g
write_reg(MPU_ADDR, 0x1C, 0x00)

time.sleep_ms(100)


# =========================================================
# READ MPU
# =========================================================

def read_accel_gyro():

    # Accelerometer
    data = read_reg(MPU_ADDR, 0x3B, 6)

    ax = signed16(data[0], data[1])
    ay = signed16(data[2], data[3])
    az = signed16(data[4], data[5])

    # Gyroscope
    data = read_reg(MPU_ADDR, 0x43, 6)

    gx = signed16(data[0], data[1])
    gy = signed16(data[2], data[3])
    gz = signed16(data[4], data[5])

    return ax, ay, az, gx, gy, gz


# =========================================================
# CALIBRATION
# =========================================================

print()
print("==============================")
print(" MPU6500 CALIBRATION")
print("==============================")
print()

print("Place the sensor LEVEL.")
print("Keep it COMPLETELY STILL.")
print()

for i in range(5, 0, -1):
    print("Starting in", i)
    time.sleep(1)

print()
print("CALIBRATING...")
print()


samples = 1000

sum_ax = 0
sum_ay = 0
sum_az = 0

sum_gx = 0
sum_gy = 0
sum_gz = 0


for i in range(samples):

    ax, ay, az, gx, gy, gz = read_accel_gyro()

    sum_ax += ax
    sum_ay += ay
    sum_az += az

    sum_gx += gx
    sum_gy += gy
    sum_gz += gz

    if i % 100 == 0:
        print("Samples:", i)

    time.sleep_ms(5)


# =========================================================
# AVERAGES
# =========================================================

avg_ax = sum_ax / samples
avg_ay = sum_ay / samples
avg_az = sum_az / samples

avg_gx = sum_gx / samples
avg_gy = sum_gy / samples
avg_gz = sum_gz / samples


# =========================================================
# CONVERT TO SENSOR UNITS
# =========================================================

# Accelerometer:
# 16384 counts = 1g

acc_x_offset = avg_ax / 16384.0
acc_y_offset = avg_ay / 16384.0

# Z should read +1g when level
acc_z_offset = (avg_az / 16384.0) - 1.0


# Gyroscope:
# 131 counts = 1 deg/s

gyro_x_offset = avg_gx / 131.0
gyro_y_offset = avg_gy / 131.0
gyro_z_offset = avg_gz / 131.0


# =========================================================
# PRINT RESULTS
# =========================================================

print()
print("==============================")
print("CALIBRATION RESULTS")
print("==============================")

print()
print("Accelerometer offsets:")
print("X =", acc_x_offset)
print("Y =", acc_y_offset)
print("Z =", acc_z_offset)

print()
print("Gyroscope offsets:")
print("X =", gyro_x_offset)
print("Y =", gyro_y_offset)
print("Z =", gyro_z_offset)


# =========================================================
# SAVE TO PICO FLASH
# =========================================================

calibration = {
    "acc": [
        acc_x_offset,
        acc_y_offset,
        acc_z_offset
    ],

    "gyro": [
        gyro_x_offset,
        gyro_y_offset,
        gyro_z_offset
    ]
}


with open("calibration.json", "w") as f:
    json.dump(calibration, f)


print()
print("==============================")
print("CALIBRATION SAVED")
print("==============================")
print()
print("File: calibration.json")
print()
print("You can now power off the Pico.")
print("The calibration will remain saved.")
