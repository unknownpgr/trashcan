from getch import Getch
import time
import RPi.GPIO as gpio
from motor import StepMotor

gpio.setmode(gpio.BCM)
pin_motor_l = [2, 3, 4, 17]
pin_motor_r = [18, 27, 22, 23]


def escape():
    gpio.setup(pin_motor_l, gpio.OUT)
    gpio.setup(pin_motor_r, gpio.OUT)
    gpio.output(pin_motor_l, gpio.LOW)
    gpio.output(pin_motor_r, gpio.LOW)

# getch = Getch()
# while True:
#     c = ord(getch())
#     if c == 27 or c == 91:
#         continue
#     print(c)


phase = [[0],[0,2],[2],[2,1],[1],[1,3],[3],[3,0]]
motor = StepMotor(pin_motor_l, phase)

try:
    v = 100
    while True:
        motor.forward(1)
        dt = 1/v
        time.sleep(dt)
        v += dt*80
        # print(v)

except Exception as e:
    print(e)

except KeyboardInterrupt:
    pass

finally:
    escape()
    print('Process sucessfully finished')
