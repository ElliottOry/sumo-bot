from machine import Pin

from machine import PWM

from time import sleep_ms

from ir_rx.nec import NEC_8

from ir_rx import print_error


#----------------------auxillary functions----------------------

#turns on motor for given direction and duty

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

    """print(f"Received NEC command. Data:{data:02X}, Addr: {addr:02X}")


    if (data==1 or data==2):

        motor = 0

    elif (data==3 or data==4):

        motor = 1


    if (data == 0 and motor_PWM[motor].duty_u16() == min):

        motor_on"""


    

    if (data==1):

        LED[data-1].high()

        motor_on(0, max, "forward")

    elif (data==2):

        LED[data-1].high()

        motor_on(0, max, "backward")

    elif (data==3):

        LED[data-1].high()

        motor_on(1, max, "forward")

    elif (data==4):

        motor_on(1, max, "backward")

    else:

        motor_on(0, 0, "forward")

        motor_on(1, 0, "forward")

    

    sleep_ms(2000)

    motor_on(0, 0, "forward")

    motor_on(1, 0, "forward")

    for pin in LED:

        pin.low()


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

    PWM(14, freq=pwm_rate, duty_u16=0),

    PWM(16, freq=pwm_rate, duty_u16=0)

]


motor_PIN = [ #logic pins - direction

    Pin(15, Pin.OUT),

    Pin(17, Pin.OUT)

]


#LED pins

LED = [

    Pin(22, Pin.OUT), #R

    Pin(26, Pin.OUT), #G

    Pin(27, Pin.OUT)  #B

]


#RF pins

RF = [

    Pin(10, Pin.IN, Pin.PULL_UP),

    Pin(11, Pin.IN, Pin.PULL_UP),

    Pin(12, Pin.IN, Pin.PULL_UP),

    Pin(13, Pin.IN, Pin.PULL_UP)

]


#IR receiver variables

ir_pin = Pin(18, Pin.IN, Pin.PULL_UP)

ir_receiver = NEC_8(ir_pin, callback=ir_callback)

ir_receiver.error_function(print_error)


#---------------------------main while--------------------------

while True:

    for pin in range(len(RF)):

        if RF[pin]() != 0:

            print("RF signal received on pin #", pin+1, ". Signal is ", RF[pin]())

    pass