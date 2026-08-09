from machine import Pin, time_pulse_us
import time

TRIG = Pin(2, Pin.OUT)
ECHO = Pin(3, Pin.IN)

TRIG.low()

def get_distance():

    # Trigger
    TRIG.low()
    time.sleep_us(5)

    TRIG.high()
    time.sleep_us(20)
    TRIG.low()

    # Measure echo pulse
    pulse = time_pulse_us(ECHO, 1, 100000)

    if pulse < 0:
        return None

    return pulse / 57.5


while True:

    distance = get_distance()

    if distance is None:
        print("NO ECHO")
    else:
        print("Distance: {:.1f} cm".format(distance))

    time.sleep_ms(100)