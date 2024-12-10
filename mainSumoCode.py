from machine import Pin, ADC
from machine import PWM
from time import sleep_ms, ticks_ms
from ir_rx.nec import NEC_8
from ir_rx import print_error

debounce = 0

'''
def callback(pin):
    global interrupt, debounce
    if(ticks_ms() - debounce) > 500:
        notFound = True
        for i in range(4):
            if(pin == order[i] and interrupt == i):
                interrupt += 1
                notFound = False
                break

        if(notFound):
            print("WRONG")
            interrupt = 0

                
        debounce = ticks_ms()

'''

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

irDebounce = 0
irInterrupt = False
#inturrupt based function for IR data
last = None
def ir_callback(data, addr, _):
    global irDebounce, irInterrupt, last
    irInterrupt = False
    
    '''
    if(not irInterrupt):
        irInterrupt = True
        print("on")
        if(str(p)[8] == '4'):
            motor_on(0, max, "forward")
            motor_on(1, max, "forward")
        elif(str(p)[8] == '5'):
            motor_on(0, max, "backward")
            motor_on(1, max, "backward")
        elif(str(p)[8] == '6'):
            motor_on(0, max, "forward")
            motor_on(1, max, "backward")
        elif(str(p)[8] == '7'):
            motor_on(0, max, "forward")
            motor_on(1, max, "backward")
    
    '''
    if(last != None):
        motor_on(0, 0, "forward")
        motor_on(1, 0, "forward")
        last = None
    else:

    
        if (data==1):

            motor_on(0, max, "forward")
            motor_on(1, max, "forward")
        elif (data==2):
            motor_on(0, max, "backward")
            motor_on(1, max, "backward")
        elif (data==3):
            motor_on(0, max, "forward")
            motor_on(1, max, "backward")
        elif (data==4):
            motor_on(0, max, "forward")
            motor_on(1, max, "backward")
        else:
            motor_on(0, 0, "forward")
            motor_on(1, 0, "forward")
        last = data
    irDebounce = ticks_ms()
    '''    
    sleep_ms(2000)
    motor_on(0, 0, "forward")
    motor_on(1, 0, "forward")
    '''
#RF interrupts rising
interrupt = False
def rf_irqf0(p):
    global interrupt, debounce
    debounce = 0
    if(not interrupt):
        interrupt = True
        #print("on")
        if(str(p)[8] == '4'):
            motor_on(0, max, "forward")
            motor_on(1, max, "forward")
        elif(str(p)[8] == '5'):
            motor_on(0, max, "backward")
            motor_on(1, max, "backward")
        elif(str(p)[8] == '6'):
            motor_on(0, max, "forward")
            motor_on(1, max, "backward")
        elif(str(p)[8] == '7'):
            motor_on(0, max, "forward")
            motor_on(1, max, "backward")
    debounce = ticks_ms()
    
    while(ticks_ms()-debounce < 200):
        if(p() == 1):
            debounce = ticks_ms()
    
    '''
        
        if(ticks_ms() - debounce) > 200:
            print("moving")
            if(not interrupt):
                print("on")
                if(str(p)[8] == '4'):
                    motor_on(0, max, "backward")
                    motor_on(1, max, "backward")
                elif(str(p)[8] == '5'):
                    motor_on(0, max, "forward")
                    motor_on(1, max, "forward")
                elif(str(p)[8] == '6'):
                    motor_on(0, max, "forward")
                    motor_on(1, max, "backward")
                elif(str(p)[8] == '7'):
                    motor_on(0, max, "forward")
                    motor_on(1, max, "backward")
                interrupt = True
            else:
                print("off")
                
                interrupt = False
            debounce = ticks_ms()
    '''
    #sleep_ms(500)
    motor_on(0, 0, "forward")
    motor_on(1, 0, "forward")
    #print(interrupt)
    interrupt = False

        
    #print("bkwd")
    #sleep_ms(500)
'''
def rf_irqf1(p):
    motor_on(0, max, "forward")
    motor_on(1, max, "forward")
    print("fwd")
    sleep_ms(500)

def rf_irqf2(p):
    motor_on(0, max, "backward")
    motor_on(1, max, "forward")
    print("right")
    sleep_ms(500)

def rf_irqf3(p):
    motor_on(0, max, "forward")
    motor_on(1, max, "backward")
    print("left")
    sleep_ms(500)

#RF interrupts falling
def rf_irqr0(p):
    motor_off(0)
    motor_off(1)


'''

#---------------------------variables---------------------------
#logic
motor_num = 0

#PWM
pwm_rate = 2000
#PWM duty_u16 min = 0 max = 65535
min = 0             
#max = 65535
max = 43690

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
#RF[0].irq(trigger=Pin.IRQ_RISING, handler=rf_irqr0)
RF[0].irq(trigger=Pin.IRQ_RISING, handler=rf_irqf0)
RF[1].irq(trigger=Pin.IRQ_RISING, handler=rf_irqf0)
RF[2].irq(trigger=Pin.IRQ_RISING, handler=rf_irqf0)
RF[3].irq(trigger=Pin.IRQ_RISING, handler=rf_irqf0)


analogue_input = ADC(28)

low_bat_pin = Pin(17, Pin.OUT)
low_bat_pin.off()

buzzer_pin = PWM(15, freq=500, duty_ns=5000)
buzzer_pin.deinit()
low_bat = False
#---------------------------main while--------------------------
while True:
    
    sensor_value = analogue_input.read_u16()
    voltage = sensor_value * (3.3 / 65535)
    if(voltage <= 3.2 and not low_bat):
        low_bat_pin.on()
        buzzer_pin.init(freq=500, duty_ns=5000, duty_u16 = 8192)
        low_bat = True
    else:
        low_bat_pin.off()
        buzzer_pin.deinit()
        low_bat = False

    sleep_ms(5)
