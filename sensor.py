import sys
import time
import math
import threading
import numpy as np
import RPi.GPIO as gpio

import spidev
import RPi.GPIO as gpio

# 센서 제어를 어떻게 하면 좋은가?
# 먼저 센서 값을 받아올 수 있도록 하자.
# include <SPI.h>
# define ADC_CS_PIN  8
# define IR_PIN_COUNT  6

class ADC:
    def __init__(self):
        self._spi = spidev.SpiDev()
        self._spi.open(0, 0)
        self._spi.max_speed_hz = 100000

    def read(self, channel):
        r = self._spi.xfer2([0x01, (0x08 + channel) << 4, 0x00])
        adc_out = ((r[1] & 3) << 8) + r[2]
        return adc_out


class IRSensor:
    def __init__(self, adc: ADC, pin_irLEDs):
        self._adc = adc

        # Initialize pin
        gpio.setup(pin_irLEDs, gpio.OUT)

        self._sensor_count = len(pin_irLEDs)
        self.pin_irLEDs = pin_irLEDs
        self.adcResult = [0]*self._sensor_count

    def getADC(self):
        for i in range(self._sensor_count) :
            gpio.output(self.pin_irLEDs[i], 1)
            # time.sleep(0.1)
            self.adcResult[i] = self._adc.read(i)
            # time.sleep(0.1)
            gpio.output(self.pin_irLEDs[i], 0)

gpio.setmode(gpio.BCM)

pins = [5,6,12,13,19,16]
    
values = [-5,-3,-1,1,3,5]

try:
    adc = ADC()
    sensor = IRSensor(adc,pins)
    while True:
        sensor.getADC()
        r= sensor.adcResult

        sv = 0
        t = 0
        for i in range(len(pins)):
            sv+=r[i]*values[i]
            t+=r[i]

        p = sv/t
        print(int(p*1000))

except  KeyboardInterrupt as e:
    print(e)
finally:
    gpio.cleanup()
    print('Program finished safly')