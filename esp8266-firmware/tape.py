from time import sleep
import neopixel
import machine
from machine import Pin, I2C

ledmotor_count = 4
MAX_DUTY = 500
MAX_LED = 255

def set_motor_and_led(i, duty):
    motorpwms[i].duty(int(duty))
    colorval = int((duty / MAX_DUTY) * MAX_LED)
    pixels[i] = (colorval, colorval, colorval)

num_pixels = ledmotor_count
neopixel_pin = Pin(4, machine.Pin.OUT) # D2 = GPIO4
pixels = neopixel.NeoPixel(neopixel_pin, num_pixels)


motor0pin = Pin(15, Pin.OUT) # D8 - GPIO15
motor1pin = Pin(13, Pin.OUT) # D7 - GPIO13
motor2pin = Pin(12, Pin.OUT) # D6 - GPIO12
motor3pin = Pin(14, Pin.OUT) # D5 - GPIO14

motorpins = [motor0pin, motor1pin, motor2pin, motor3pin]

motorpwms = [
    machine.PWM(motor0pin, freq=1000, duty=0),
    machine.PWM(motor1pin, freq=1000, duty=0),
    machine.PWM(motor2pin, freq=1000, duty=0),
    machine.PWM(motor3pin, freq=1000, duty=0)
]

def stop_motors():
    for i in range(ledmotor_count):
        motorpwms[i].duty(0)
        pixels[i] = (0, 0, 0)
    pixels.write()

def cycle_motors(doStop=True):
    for activeIndex in range(ledmotor_count):
        duty = 0
        while duty < MAX_DUTY:
            for i in range(ledmotor_count):
                if i == activeIndex:
                    set_motor_and_led(i, duty)
                    
                else:
                    set_motor_and_led(i, 0)
            duty += 10
            sleep(0.01)
            pixels.write()
    if doStop:
        stop_motors()

stop_motors()

def perform_sequence(sequence, actuation_time = 0.035, inter_phase_time = 0.15):
    for duties in sequence:
        for i in range(ledmotor_count):
            set_motor_and_led(i, duties[i])
        pixels.write()
        sleep(actuation_time)
    stop_motors()
    sleep(inter_phase_time)


LOW = MAX_DUTY * 0.25
MEDIUM = MAX_DUTY * 0.5
HIGH = MAX_DUTY * 0.75
FULL = MAX_DUTY

extension_phases = [
    (0, 0, 0, 0),
    (0, MEDIUM, MEDIUM, 0),
    (0, FULL, FULL, 0),
    (LOW, HIGH, HIGH, LOW),
    (MEDIUM, MEDIUM, MEDIUM, MEDIUM),
    (HIGH, LOW, LOW, HIGH),
    (FULL, 0, 0, FULL),
    (MEDIUM, 0, 0, MEDIUM),
    (0, 0, 0, 0),
]

def show_extension_sequence():
    for _ in range(3):
        perform_sequence(extension_phases)
    stop_motors()

def show_retraction_sequence():
    for _ in range(3):
        perform_sequence(reversed(extension_phases))
    stop_motors()

linear_phases = [
    (0, 0, 0, 0),
    (LOW, 0, 0, 0),
    (HIGH, LOW, 0, 0),
    (LOW, HIGH, LOW, 0, 0),
    (0, LOW, HIGH, LOW),
    (0, 0, LOW, HIGH),
    (0, 0, 0, LOW),
    (0, 0, 0, 0),
]

def show_forward_sequence():
    for _ in range(3):
        perform_sequence(linear_phases)
    stop_motors()

def show_backward_sequence():
    for _ in range(3):
        perform_sequence(reversed(linear_phases))
    stop_motors()
