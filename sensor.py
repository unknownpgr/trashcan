import sys
import time
import math
import threading
import numpy as np
import spidev
import RPi.GPIO as gpio

# 센서 제어를 어떻게 하면 좋은가?
# 먼저 센서 값을 받아올 수 있도록 하자.

class ADC:
    def __init__(self):
        self._spi = spidev.SpiDev()
        self._spi.open(0, 0)
        self._spi.max_speed_hz = 100000

    # Read the adc value of given channel
    def read(self, channel):
        r = self._spi.xfer2([0x01, (0x08 + channel) << 4, 0x00])
        adc_out = ((r[1] & 3) << 8) + r[2]
        return adc_out

    # Read the median of n adc value
    def readMedian(self,channel,n=3):
        sensorValue = []
        for _ in range(n):
            sensorValue.append(self.read(channel))
        sensorValue.sort()
        return sensorValue[int((n-1)/2)]

class IRSensor:
    def __init__(self, adc: ADC, pin_irLEDs):
        self._adc = adc

        # Initialize pin
        gpio.setup(pin_irLEDs, gpio.OUT)

        self.sensor_count = len(pin_irLEDs)
        self.pin_irLEDs = pin_irLEDs

        # Array for adc result
        self.adcResult = [0]*self.sensor_count

        # Array for calibration
        self.isCalibrated = False
        self._valueMin = [2048]*self.sensor_count
        self._valueMax = [0]*self.sensor_count
        self._valueDelta = [0]*self.sensor_count

    def calibration(self):
        input('Press any key to start calibration.')

        loop = True

        # Thread for sensing
        def t():
            while loop:
                self.getRawADC()
                vs = self.adcResult
                for i in range(self.sensor_count):
                    if vs[i]>self._valueMax[i]:
                        self._valueMax[i]=vs[i]
                    if vs[i]<self._valueMin[i]:
                        self._valueMin[i]=vs[i]
        thread = threading.Thread(target=t)
        thread.start()

        input('Press any key to stop calibration.')
        self.isCalibrated = True
        loop = False
        thread.join()

        for i in range(self.sensor_count):
            self._valueDelta[i] = self._valueMax[i]-self._valueMin[i]

        print('Calibration finished.')

    def getRawADC(self):
        for i in range(self.sensor_count) :
            gpio.output(self.pin_irLEDs[i], 1)
            self.adcResult[i] = self._adc.readMedian(i)
            gpio.output(self.pin_irLEDs[i], 0)

    def getADC(self):
        assert self.isCalibrated
        for i in range(self.sensor_count) :
            gpio.output(self.pin_irLEDs[i], 1)
            self.adcResult[i] = (self._adc.readMedian(i)-self._valueMin[i])*1024/self._valueDelta[i]
            if self.adcResult[i]<0:
                self.adcResult[i]=0
            if self.adcResult[i]>1024:
                self.adcResult[i] = 1024
            gpio.output(self.pin_irLEDs[i], 0)

gpio.setmode(gpio.BCM)

pins = [5,6,12,13,19,16]
    
values = [-5,-3,-1,1,3,5]

try:
    adc = ADC()
    sensor = IRSensor(adc,pins)
    
    sensor.calibration()

    while True:
        sensor.getADC()
        print(sensor.adcResult)

    while True:
        sensor.getADC()
        r = list(map(lambda x:1024-x,sensor.adcResult))
        ws = 0
        vs = 0
        # print(r)
        for i in range(len(values)):
            ws+=values[i]*r[i]
            vs+=r[i]
        s = ''
        for i in range(int(ws*5/vs)+25):
            s+=' '
        s+='||'
        print(s)

except  KeyboardInterrupt as e:
    print(e)
finally:
    gpio.cleanup()
    print('Program finished safly')