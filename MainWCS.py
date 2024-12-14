from machine import Pin, ADC
from machine import PWM
from time import sleep_ms, ticks_ms
from ir_rx.nec import NEC_8
from ir_rx import print_error

#TCS34725 Color Sensor
from machine import Pin, I2C, PWM
import time

#TCS3200 Color Sensor
from machine import Pin, Timer, PWM
import time

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

#--------------------------TCS3200 Color Sensor Backwards---------------------#

# Pin definitions
s2 = Pin(20, Pin.OUT)
s3 = Pin(19, Pin.OUT)
signal = Pin(18, Pin.IN)
NUM_CYCLES = 20  # Increased cycles for better accuracy

# Motor control definitions
motor_PWM = [PWM(Pin(15), freq=2000, duty_u16=0), PWM(Pin(13), freq=2000, duty_u16=0)]
motor_PIN = [Pin(14, Pin.OUT), Pin(12, Pin.OUT)]
max_duty = 43690

# Previous and current color flags
previous_color = None
current_color = None

# Time tracking
previous_time = time.time()

def setup():
    print("Setting up...")
    signal.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=color_interrupt)

def color_interrupt(pin):
    pass  # Interrupt handler not needed for this implementation

def motor_on(motor, duty, dir, dur=-1):
    if dir == "forward":
        motor_PIN[motor].low()
    elif dir == "backward":
        motor_PIN[motor].high()
    
    motor_PWM[motor].duty_u16(duty)
    if dur != -1:
        time.sleep_ms(dur)
        motor_off(motor)

def motor_off(motor):
    motor_PWM[motor].duty_u16(0)
    motor_PIN[motor].low()

def read_color():
    # Read Red
    s2.low()
    s3.low()
    time.sleep(0.1)  # Reduced sleep duration
    duration = signal_pulse_width()
    red = NUM_CYCLES / (duration / 1000000) if duration != 0 else 0

    # Read Green
    s2.high()
    s3.high()
    time.sleep(0.1)  # Reduced sleep duration
    duration = signal_pulse_width()
    green = NUM_CYCLES / (duration / 1000000) if duration != 0 else 0

    # Read Blue
    s2.low()
    s3.high()
    time.sleep(0.1)  # Reduced sleep duration
    duration = signal_pulse_width()
    blue = NUM_CYCLES / (duration / 1000000) if duration != 0 else 0

    return red, green, blue

def signal_pulse_width():
    start = time.ticks_us()
    for _ in range(NUM_CYCLES):
        while signal.value() == 0:
            pass
        while signal.value() == 1:
            pass
    return time.ticks_diff(time.ticks_us(), start)

def determine_color(r, g, b):
    # Adjusted thresholds based on provided measurements
    if r > 1.3 * g and r > 1.5 * b and r > 4500:  # Red object detection
        return "Red"
    elif r > 1.3 * g and r > 1.5 * b:  # Gray object detection with red majority
        return "Gray"
    else:
        return "Unknown"

def loop():
    global previous_color, current_color, previous_time

    while True:
        r, g, b = read_color()
        current_color = determine_color(r, g, b)
        
        current_time = time.time()
        
        if current_color != previous_color:
            time_diff = current_time - previous_time
            print(f"Color change detected: {previous_color} to {current_color} | Red: {r} | Green: {g} | Blue: {b} | Time between changes: {time_diff:.2f} seconds")
            
            # Set motors to backward direction for 1 second
            motor_on(0, max_duty, "backward", 1000)
            motor_on(1, max_duty, "backward", 1000)
            
            previous_color = current_color
            previous_time = current_time
        
        time.sleep(1)  # Adjust sleep duration as needed

def endprogram():
    pass  # No cleanup needed in MicroPython

if __name__ == '__main__':
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        endprogram()

#--------------------------TCS34725 Color Sensor Forwards---------------------#

# TCS34725 I2C address
TCS34725_ADDRESS = 0x29

# TCS34725 registers
TCS34725_ENABLE = 0x00
TCS34725_ATIME = 0x01
TCS34725_CONTROL = 0x0F
TCS34725_ID = 0x12
TCS34725_STATUS = 0x13
TCS34725_CDATAL = 0x14
TCS34725_CDATAH = 0x15
TCS34725_RDATAL = 0x16
TCS34725_RDATAH = 0x17
TCS34725_GDATAL = 0x18
TCS34725_GDATAH = 0x19
TCS34725_BDATAL = 0x1A
TCS34725_BDATAH = 0x1B
TCS34725_ENABLE_PON = 0x01
TCS34725_ENABLE_AEN = 0x02

# Command bit
TCS34725_COMMAND_BIT = 0x80

# Initialization of the I2C interface
i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)  # Adjust pins if necessary

def write_register(register, value):
    i2c.writeto_mem(TCS34725_ADDRESS, TCS34725_COMMAND_BIT | register, bytes([value]))

def read_register(register):
    return i2c.readfrom_mem(TCS34725_ADDRESS, TCS34725_COMMAND_BIT | register, 1)[0]

def read_register_word(register):
    low = read_register(register)
    high = read_register(register + 1)
    return (high << 8) | low

def enable_tcs34725():
    write_register(TCS34725_ENABLE, TCS34725_ENABLE_PON)
    time.sleep(0.01)
    write_register(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)
    time.sleep(0.01)

def get_sensor_id():
    return read_register(TCS34725_ID)

def set_integration_time(time):
    write_register(TCS34725_ATIME, time)

def set_gain(gain):
    write_register(TCS34725_CONTROL, gain)

# Initialization with retry
for attempt in range(5):
    print(f"Attempt {attempt + 1}: Enabling TCS34725...")
    enable_tcs34725()
    time.sleep(0.1)  # Increased delay for stability
    print("Setting integration time...")
    set_integration_time(0xEB)
    time.sleep(0.1)
    print("Setting gain...")
    set_gain(0x01)
    time.sleep(0.1)
    sensor_id = get_sensor_id()
    print(f"Sensor ID read: {sensor_id:#x}")  # Print sensor ID in hex
    if sensor_id == 0x4D:  # Recognize the sensor ID 0x4D
        print("TCS34725 found!")
        break
    else:
        print("TCS34725 not found! Retrying...")
        time.sleep(1)
else:
    print("TCS34725 not found after multiple attempts!")

# Check if sensor was detected before proceeding
if sensor_id != 0x4D:
    print("Exiting because the sensor was not detected.")
    exit()

def read_rgbc():
    r = read_register_word(TCS34725_RDATAL)
    g = read_register_word(TCS34725_GDATAL)
    b = read_register_word(TCS34725_BDATAL)
    c = read_register_word(TCS34725_CDATAL)
    return r, g, b, c

def is_color_red(r, g, b):
    return r > 1.5 * g and r > 1.5 * b

def is_color_gray(r, g, b):
    return r > 1 * g and r > 1.1 * b

# Motor control functions
motor_PWM = [PWM(Pin(15), freq=2000, duty_u16=0), PWM(Pin(13), freq=2000, duty_u16=0)]
motor_PIN = [Pin(14, Pin.OUT), Pin(12, Pin.OUT)]
max_duty = 43690

def motor_on(motor, duty, dir, dur=-1):
    if dir == "forward":
        motor_PIN[motor].low()
    elif dir == "backward":
        motor_PIN[motor].high()

    motor_PWM[motor].duty_u16(duty)
    if dur != -1:
        time.sleep_ms(dur)
        motor_off(motor)

def motor_off(motor):
    motor_PWM[motor].duty_u16(0)
    motor_PIN[motor].low()

previous_color = None
previous_time = time.time()

while True:
    print("Reading color data...")
    r, g, b, c = read_rgbc()
    print(f"Red: {r}, Green: {g}, Blue: {b}, Clear: {c}")
    
    current_color = 'Red' if is_color_red(r, g, b) else 'Gray' if is_color_gray(r, g, b) else 'Unknown'
    
    if current_color != previous_color:
        current_time = time.time()
        time_diff = current_time - previous_time
        print(f"Color change detected: {previous_color} to {current_color} | Red: {r} | Green: {g} | Blue: {b} | Clear: {c} | Time between changes: {time_diff:.2f} seconds")
        
        # Set motors to forward direction for 1 second
        motor_on(0, max_duty, "forward", 1000)
        motor_on(1, max_duty, "forward", 1000)
        
        previous_color = current_color
        previous_time = current_time
    
    time.sleep(0.5)