from machine import I2C, Pin
import time
import math
import json

# ---------- I2C ----------
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

MPU = 0x68
MAG = 0x1E

# ---------- Calibration ----------
with open("calibration.json") as f:
    cal = json.load(f)

AO = cal["acc"]
GO = cal["gyro"]
MO = cal["mag"]["offset"]
MS = cal["mag"]["scale"]

# ---------- Initialize MPU6500 ----------
i2c.writeto_mem(MPU, 0x6B, b'\x00')
time.sleep_ms(100)

# Gyro ±250 °/s
i2c.writeto_mem(MPU, 0x1B, b'\x00')

# Accel ±2g
i2c.writeto_mem(MPU, 0x1C, b'\x00')

# ---------- Initialize HMC5883L ----------
i2c.writeto_mem(MAG, 0x00, b'\x70')  # 8 samples, 15 Hz
i2c.writeto_mem(MAG, 0x01, b'\x20')  # ±1.3 Gauss
i2c.writeto_mem(MAG, 0x02, b'\x00')  # continuous

time.sleep_ms(100)


def s16(h, l):
    v = (h << 8) | l
    return v - 65536 if v & 0x8000 else v


# ---------- State for tilt estimation ----------
pitch = 0.0
roll = 0.0
last_time = time.ticks_ms()


def get_data():

    global pitch, roll, last_time

    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 1000
    last_time = now

    if dt <= 0 or dt > 0.1:
        dt = 0.01

    # =========================
    # ACCELEROMETER
    # =========================

    d = i2c.readfrom_mem(MPU, 0x3B, 6)

    ax = s16(d[0], d[1]) / 16384
    ay = s16(d[2], d[3]) / 16384
    az = s16(d[4], d[5]) / 16384

    # Calibration
    ax -= AO[0]
    ay -= AO[1]
    az -= AO[2]

    # =========================
    # GYROSCOPE
    # =========================

    d = i2c.readfrom_mem(MPU, 0x43, 6)

    gx = s16(d[0], d[1]) / 131 - GO[0]
    gy = s16(d[2], d[3]) / 131 - GO[1]
    gz = s16(d[4], d[5]) / 131 - GO[2]

    # =========================
    # TILT ESTIMATION
    # =========================

    # Accelerometer angles
    acc_roll = math.atan2(ay, az)
    acc_pitch = math.atan2(
        -ax,
        math.sqrt(ay * ay + az * az)
    )

    acc_roll = math.degrees(acc_roll)
    acc_pitch = math.degrees(acc_pitch)

    # Gyro integration
    roll += gx * dt
    pitch += gy * dt

    # Complementary filter
    alpha = 0.98

    roll = alpha * roll + (1 - alpha) * acc_roll
    pitch = alpha * pitch + (1 - alpha) * acc_pitch

    # =========================
    # MAGNETOMETER
    # =========================

    d = i2c.readfrom_mem(MAG, 0x03, 6)

    # HMC5883L order = X, Z, Y
    mx = s16(d[0], d[1])
    mz = s16(d[2], d[3])
    my = s16(d[4], d[5])

    # Calibration
    mx = (mx - MO[0]) * MS[0]
    my = (my - MO[1]) * MS[1]
    mz = (mz - MO[2]) * MS[2]

    # Convert to approximately µT
    mx *= 0.092
    my *= 0.092
    mz *= 0.092

    # =========================
    # HEADING
    # =========================

    heading = math.degrees(math.atan2(my, mx))

    if heading < 0:
        heading += 360

    # =========================
    # RETURN DATA
    # =========================

    return {
        "ax": ax,
        "ay": ay,
        "az": az,

        "gx": gx,
        "gy": gy,
        "gz": gz,

        "mx": mx,
        "my": my,
        "mz": mz,

        "heading": heading,

        "pitch": pitch,
        "roll": roll
    }
