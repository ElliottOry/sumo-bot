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

#---------------------------main while--------------------------
while True:
    print("motor", motor_num+1, "forwards 2s")
    motor_on(motor_num, max, "forward", 2000)
    motor_off(motor_num)

    print("motor", motor_num+1, "backwards 2s")
    motor_on(motor_num, max, "backward", 2000)
    motor_off(motor_num)

    print("wait 1s")
    if motor_num==0:
        motor_num=1
    else:
        motor_num=0
    sleep_ms(1000)
    pass
