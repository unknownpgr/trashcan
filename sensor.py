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

ir_pin = {2, 3, 4, 5, 6, 7};


class ADC:
    def __init__(self,ce = -1):
        self._spi = spidev.SpiDev()
        self._spi.open(0, 0)

    def read(self, channel):
        r = self._spi.xfer2([1, (8 + channel) << 4, 0])
        adc_out = ((r[1] & 3) << 8) + r[2]
        return adc_out


class IRSensor:
    def __init__(self, adc: ADC, pin_irLEDs):
        self._adc = _adc

        # Initialize pin
        gpio.setup(pin_irLEDs, gpio.OUT)

        self._sensor_count = len(pin_irLEDs)
        self.pin_irLEDs = pin_irLEDs
        self.adcResult = [0]

    def getADC(self):
        for i in range(self._sensor_count) {
            gpio.output(self.pin_irLEDs[i], 1);
            self.
            gpio.output(self.pin_irLEDs[i], 0);
