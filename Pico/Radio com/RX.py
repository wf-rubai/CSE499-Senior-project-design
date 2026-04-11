from machine import SPI, Pin
from sx127x import SX127x
import time

print("Starting RX...")

spi = SPI(0, baudrate=1000000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))

lora = SX127x(spi, pins={"ss": 17, "reset": None, "dio_0": None},
              parameters={"frequency": 433000000, "spreading_factor": 7, "signal_bandwidth": 125000})

print("RX Ready, SX version:", lora.readRegister(0x42))

lora.receive()  # continuous receive mode

while True:
    # check if RX FIFO has data
    nb_bytes = lora.readRegister(0x13)  # REG_RX_NB_BYTES
    if nb_bytes > 0:
        payload = lora.readPayload()
        try:
            msg = payload.decode()
        except:
            msg = str(payload)
        rssi = lora.packetRssi()
        print("Received:", msg, "| RSSI:", rssi, "dBm")
    else:
        print("No packet...")
    time.sleep(0.2)