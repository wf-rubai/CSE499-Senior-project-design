from machine import SPI, Pin
from sx127x import SX127x
import time

print("Starting TX...")

# Initialize SPI
spi = SPI(0, baudrate=1000000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))

# Initialize LoRa
lora = SX127x(spi, pins={"ss": 17, "reset": 20, "dio_0": None},
              parameters={
                  "frequency": 433000000,  # 433 MHz
                  "tx_power_level": 20,
                  "spreading_factor": 10,
                  "signal_bandwidth": 125000,
                  "coding_rate": 5,
                  "preamble_length": 8,
                  "sync_word": 0x12,
                  "enable_CRC": True
              })

print("TX Ready, SX version:", lora.readRegister(0x42))
time.sleep(2)  # give RX time to start

# Send loop
while True:
    print("Sending HELLO...")
    lora.println("Hello")
    print("Sent!")
    time.sleep(2)