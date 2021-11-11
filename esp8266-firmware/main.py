
from time import sleep
import neopixel

import machine
from machine import Pin, I2C

ledmotor_count = 4

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

for _ in range(10):
    for activeIndex in range(ledmotor_count):
        duty = 0
        maxduty = 1000
        while duty < maxduty:
            for i in range(ledmotor_count):
                if i == activeIndex:
                    motorpwms[i].duty(duty)
                    colorval = int((duty / maxduty) * 255)
                    pixels[i] = (colorval, colorval, colorval)
                else:
                    motorpwms[i].duty(0)
                    pixels[i] = (0, 0, 0)
            duty += 10
            sleep(0.01)
            pixels.write()

# Turn off at end
for i in range(ledmotor_count):
    motorpwms[i].duty(0)
    pixels[i] = (0, 0, 0)
pixels.write()





# np[0]= (155, 0, 155)
# np[1]= (0, 120, 155)
# np[2]= (155, 120, 155)
# np[3]= (55, 20, 55)

# # for i in range(len(np)):
# #     np[i] = (0, 0, 0)

# np.write()

# haptic_i2c = I2C(scl=Pin(9), sda=Pin(10))

# s = 0
# while s < 20:
#     c = 0
#     while c < 55:
#         c += 1
#         t = [0, 0, 0]
#         t[s % 3] = c
#         np.fill(t)
#         np.write()
#         sleep(0.03)
#     s += 1

# # blink command
# led = machine.Pin(2, machine.Pin.OUT)
# while True:
    # led.value(not led.value())
    # sleep(0.5)

# np.fill((0, 0, 0))
# np.write()

# def short_burst(0:)i