#---------------------------------------------------------------
#MAIN FILE FOR GROUP DO NOT EDIT UNLESS YOUR CODE CONFIRMED WORKS
#---------------------------------------------------------------

from machine import Pin
from machine import PWM
from time import sleep_ms
from ir_rx.nec import NEC_8
from ir_rx import print_error

#----------------------auxillary functions----------------------
#turns on motor for given direction, duty, and time
def motor_on(motor, duty, dir, dur=-1): 
    #set direction
    if dir == "forward":
        motor_PIN[motor].low()
    elif dir == "backward":
        motor_PIN[motor].high()

    #set power
    motor_PWM[motor].duty_u16(duty)
    if dur!=-1:
        sleep_ms(dur)
        motor_off(motor)

#turns off motor and resets GPIO to low
def motor_off(motor):
    motor_PWM[motor].duty_u16(0)
    motor_PIN[motor].low()

#inturrupt based function for IR data
def ir_callback(data, addr, _):    
    print(data)
    if (data==1):
        motor_on(0, max, "forward")
    elif (data==2):
        motor_on(0, max, "backward")
    elif (data==3):
        motor_on(1, max, "forward")
    elif (data==4):
        motor_on(1, max, "backward")
    else:
        motor_on(0, 0, "forward")
        motor_on(1, 0, "forward")
    
    t = 0
    while True:
        if(t >= 5):
            break
        if(True):
            t += 1
        else:
            t = 0
        
        sleep_ms(1)
        pass
    motor_on(0, 0, "forward")
    motor_on(1, 0, "forward")

#RF interrupts rising
def rf_irqr0(p):
    motor_on(0, max, "backward")
    motor_on(1, max, "backward")
    t = 0
    while True:
        if(t >= 25):
            break
        if(p() == 0):
            t += 1
        else:
            t = 0
        
        sleep_ms(1)
        pass
    motor_off(0)
    motor_off(1)

def rf_irqr1(p):
    motor_on(0, max, "forward")
    motor_on(1, max, "forward")
    t = 0
    while True:
        if(t >= 25):
            break
        if(p() == 0):
            t += 1
        else:
            t = 0
        
        sleep_ms(1)
        pass
    motor_off(0)
    motor_off(1)

def rf_irqr2(p):
    motor_on(0, max, "backward")
    motor_on(1, max, "forward")
    t = 0
    while True:
        if(t >= 25):
            break
        if(p() == 0):
            t += 1
        else:
            t = 0
        
        sleep_ms(1)
        pass
    motor_off(0)
    motor_off(1)

def rf_irqr3(p):
    motor_on(0, max, "forward")
    motor_on(1, max, "backward")
    t = 0
    while True:
        if(t >= 25):
            break
        if(p() == 0):
            t += 1
        else:
            t = 0
        
        sleep_ms(1)
        pass
    motor_off(0)
    motor_off(1)

#---------------------------variables---------------------------
#logic
motor_num = 0

#PWM
pwm_rate = 2000
#PWM duty_u16 min = 0 max = 65535
min = 0             
max = 65535

#motor pins
motor_PWM = [ #PWM pins - duty cycle 
    PWM(15, freq=pwm_rate, duty_u16=0),
    PWM(13, freq=pwm_rate, duty_u16=0)
]

motor_PIN = [ #logic pins - direction
    Pin(14, Pin.OUT),
    Pin(12, Pin.OUT)
]

#RF pins
RF = [
    Pin(4, Pin.IN, Pin.PULL_UP),
    Pin(5, Pin.IN, Pin.PULL_UP),
    Pin(6, Pin.IN, Pin.PULL_UP),
    Pin(7, Pin.IN, Pin.PULL_UP)
]

#IR receiver variables
ir_pin = Pin(18, Pin.IN, Pin.PULL_UP)
ir_receiver = NEC_8(ir_pin, callback=ir_callback)
ir_receiver.error_function(print_error)

#RF receiver variables
RF[0].irq(trigger=Pin.IRQ_RISING, handler=rf_irqr0)
RF[1].irq(trigger=Pin.IRQ_RISING, handler=rf_irqr1)
RF[2].irq(trigger=Pin.IRQ_RISING, handler=rf_irqr2)
RF[3].irq(trigger=Pin.IRQ_RISING, handler=rf_irqr3)

#---------------------------main while--------------------------

while True:
    motor_off(0)
    motor_off(1)
    pass
