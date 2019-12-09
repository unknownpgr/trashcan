import RPi.GPIO as gpio
import time


pins = [2, 4, 3, 17]

gpio.setmode(gpio.BCM)


def escape():
    gpio.setup(pins, gpio.OUT)
    gpio.output(pins, gpio.LOW)


index = 0
try:
    escape()

    while True:
        pin = pins[index]
        for p in pins:
            if p != pin:
                gpio.output(p, 0)
            else:
                gpio.output(p, 1)
        time.sleep(.5)
        index += 1
        index %= len(pins)

except Exception as e:
    print(e)

except KeyboardInterrupt:
    pass

finally:
    escape()
    print('Process sucessfully finished')
