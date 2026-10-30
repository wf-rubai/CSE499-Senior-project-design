from machine import Pin, PWM, time_pulse_us
import time

# Receiver inputs — unchanged
CH2 = Pin(14, Pin.IN, Pin.PULL_DOWN)  # Steering
CH3 = Pin(15, Pin.IN, Pin.PULL_DOWN)  # Forward

# ESC outputs — unchanged
left_esc = PWM(Pin(17))
right_esc = PWM(Pin(16))

left_esc.freq(50)
right_esc.freq(50)

STOP_US = 1000
RUN_US = 1300
TURN_US = 1300

STEERING_CENTER = 1500
STEERING_DEADBAND = 100

led = Pin("LED", Pin.OUT)

def set_motor(esc, pulse_us):
    esc.duty_ns(pulse_us * 1000)

def read_channel(pin):
    time_pulse_us(pin, 1, 25000)
    pulse = time_pulse_us(pin, 1, 25000)

    if 900 <= pulse <= 2100:
        return pulse
    return None

set_motor(left_esc, STOP_US)
set_motor(right_esc, STOP_US)
time.sleep(2)

last_led = time.ticks_ms()
led_on = False

try:
    while True:
        now = time.ticks_ms()
        phase = time.ticks_diff(now, last_led)

        if phase >= 1000:
            last_led = now
            phase = 0

        new_led_state = phase < 100
        if new_led_state != led_on:
            led.value(new_led_state)
            led_on = new_led_state

        ch2 = read_channel(CH2)
        ch3 = read_channel(CH3)

        # Stop if receiver signal is invalid
        if ch2 is None or ch3 is None:
            set_motor(left_esc, STOP_US)
            set_motor(right_esc, STOP_US)
            time.sleep_ms(20)
            continue

        # Steering has priority over forward
        if ch2 > STEERING_CENTER + STEERING_DEADBAND:
            left = RUN_US
            right = STOP_US       # Turn right

        elif ch2 < STEERING_CENTER - STEERING_DEADBAND:
            left = STOP_US
            right = RUN_US        # Turn left

        elif ch3 > 1200:
            left = RUN_US
            right = RUN_US        # Forward

        else:
            left = STOP_US
            right = STOP_US       # Stop

        set_motor(left_esc, left)
        set_motor(right_esc, right)

        time.sleep_ms(20)

except KeyboardInterrupt:
    pass

finally:
    set_motor(left_esc, STOP_US)
    set_motor(right_esc, STOP_US)
    time.sleep(1)
    left_esc.deinit()
    right_esc.deinit()
    led.off()
