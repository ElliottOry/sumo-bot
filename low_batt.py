from machine import Pin, ADC
from time import sleep
analogue_input = ADC(28)

low_bat_pin = Pin(17, Pin.OUT)
low_bat_pin.off()

buzzer_pin = Pin(15, Pin.OUT)
buzzer_pin.off()

while True:
    sensor_value = analogue_input.read_u16()
    voltage = sensor_value * (3.3 / 65535)
    sleep(1)
    voltage = 0
    if(voltage <= 3.1):
        low_bat_pin.on()
        buzzer_pin.on()
    print(voltage)
    sleep(1)
