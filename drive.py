import RPi.GPIO as gpio
from motor import StepMotor, StepMotorTask
import time


class Driver:
    def __init__(self):
        # v = sys.version_info
        # print('==== [ INFORMATION ] ====')
        # print('Current python version : ', v[0], '.', v[1], '.', v[2], sep='')
        # print()

        # You must set GPIO mode before ouse gpio-associate functions.
        gpio.setmode(gpio.BCM)

        # GPIO number(=BCM). (not BOARD)
        pin_motor_l = [2, 3, 4, 17]
        pin_motor_r = [18, 27, 22, 23]
        phases = [[0], [2], [1], [3]]

        self.motorLeft = StepMotor(pin_motor_l, phases)
        self.taskLeft = StepMotorTask(self.motorLeft)

        self.motorRight = StepMotor(pin_motor_r, phases)
        self.taskLeft = StepMotorTask(self.motorRight)


driver = Driver()
driver.taskLeft.start()

v = 1
while True:
    driver.taskLeft.velocity = v
    v += .1
    time.sleep(.5)
