from machine import Pin
from machine import PWM
from time import sleep_ms
from ir_rx.nec import NEC_8
from ir_rx import print_error

#----------------------auxillary functions----------------------
#inturrupt based function for IR data
def ir_callback(data, addr, _):
        
    if (data==1):
        LED[data-1].high()
    elif (data==2):
        LED[data-1].high()
    elif (data==3):
        LED[data-1].high()

    for pin in LED:
        pin.low()

#---------------------------variables---------------------------
#LED pins
LED = [
    Pin(22, Pin.OUT), #R
    Pin(26, Pin.OUT), #G
    Pin(27, Pin.OUT)  #B
]

#IR receiver variables
ir_pin = Pin(18, Pin.IN, Pin.PULL_UP)
ir_receiver = NEC_8(ir_pin, callback=ir_callback)
ir_receiver.error_function(print_error)

#---------------------------main while--------------------------
while True:
    pass
