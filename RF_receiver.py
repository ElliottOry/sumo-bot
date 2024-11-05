from machine import Pin
from machine import PWM
from time import sleep_ms
from ir_rx.nec import NEC_8
from ir_rx import print_error

#---------------------------variables---------------------------
#RF pins
RF = [
    Pin(10, Pin.IN, Pin.PULL_UP),
    Pin(11, Pin.IN, Pin.PULL_UP),
    Pin(12, Pin.IN, Pin.PULL_UP),
    Pin(13, Pin.IN, Pin.PULL_UP)
]

#---------------------------main while--------------------------
while True:
    for pin in range(len(RF)):
        if RF[pin]() != 0:
            print("RF signal received on pin #", pin+1, ". Signal is ", RF[pin]())
    pass
