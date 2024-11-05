from machine import Pin
from time import sleep_ms
from ir_tx.nec import NEC

tx_pin = Pin(17, Pin.OUT, value=0)
device_addr = 0x01
transmitter = NEC(tx_pin)
commands = [0x01, 0x02, 0x03, 0x04]

if __name__ == "__main__":
    while True:
        for command in commands:
            transmitter.transmit(device_addr, command)
            print("commands", hex(command), "transmitted")
            sleep_ms(3000)
