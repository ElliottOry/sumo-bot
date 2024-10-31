from machine import Pin
import time
import random

pin6 = Pin(11, Pin.OUT)
pin7 = Pin(12, Pin.OUT)
pin8 = Pin(13, Pin.OUT)
pin6.on()
pin7.on()
pin8.on()
while True:
    '''
    pin6.off()
    pin7.off()
    pin8.off()
    time.sleep(1)
    
    pin6.on()
    pin7.on()
    pin8.on()
    time.sleep(1)
    '''
