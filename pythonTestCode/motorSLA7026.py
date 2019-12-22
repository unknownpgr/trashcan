import RPi.GPIO as gpio
import time
from threading import Thread
import math
import os
import sys

v = sys.version_info
print('======== [ INFORMATION ] ========')
print('Current python version : ', v[0], '.', v[1], '.', v[2],sep='')
print()

CORE = 3

# Core assign
print('======== [ Core Assign ] ========')
pid = os.getpid()
print(f'Current PID = {pid}')
is_core_assigned = os.system(f'taskset -p {2**CORE} {pid}')
if is_core_assigned!=0:
    print('Process core assign failed. exit process.')
    exit()
print(f'Process assigned to core {CORE} = affinity mask {2**CORE}')
print()

# Priority update
print('======== [ Priority update ] ========')
is_priority_updated = os.system(f'sudo renice -20 {pid}')
if is_priority_updated!=0:
    print('Process priority update failed. exit process.')
    exit()
print('Priority updated.')
print()

motor_l_pin = [2, 3, 4, 17]
motor_r_pin = [18, 27, 22, 23]

phases = [[0], [0, 2],
          [2], [2, 1],
          [1], [1, 3],
          [3], [3, 0]]


# Phase generator function for unipolar step motor driver
def convertToPhase(pins, phases):
    pinsOn = []
    pinsOff = []

    for phase in phases:
        pinOn = []
        pinOff = []
        for i in range(len(pins)):
            if i in phase:
                pinOn.append(pins[i])
            else:
                pinOff.append(pins[i])
        pinsOn.append(pinOn)
        pinsOff.append(pinOff)

    return pinsOn, pinsOff

def init(pins):
    gpio.setup(pins, gpio.OUT)
    gpio.output(pins, 0)
    gpio.setup(pins, gpio.IN)

class SLA7026Cotroller:
    def __init__(self,pins,phases,steps = 400):
        self.motor_pin = pins
        self.phases = phases
        self.phases_len = len(phases)
        self.phase_on,self.phase_off = convertToPhase(pins, phases)

        self.thraed = None
        self.loop = False

        self._minDt = 0.001
        self._velocity = 0
        self._dt = 0
        self._dir = 1
        self._k = math.pi*2/(steps)

        # Pin setup
        gpio.setup(pins, gpio.OUT)
        gpio.output(pins, 1)

    def start(self):
        self.loop = True
        self.thraed = Thread(target=self._run)
        self.thraed.start()

    def stop(self):
        try:
            self.setVelocity(0)
            self.loop=False
            self.thraed.join()
        except Exception as e:
            print(e)
        except InterruptedError as e:
            print(e)

    def setVelocity(self,omega):
        # omega = dT/dt
        # therefore dt = dT/omega
        # dT = 2PI/400 = PI/200
        # therefore dt = (PI/200)/omega

        self._velocity = omega
        if omega>0:
            self._dir = 1
        if omega<0:
            self._dir = -1
        if omega==0:
            self._dir=0
            self._dt = self._minDt
            return

        self._dt = abs(self._k/self._velocity)
        if self._dt>self._minDt:
            self._dt = self._minDt

    def getVelocity(self,omgea):
        return self._velocity

    def getMinVelocity(self):
        # dt = k / v
        # :. v = k / dt
        # :. minV = k / minDt
        return self._k/self._minDt

    def _run(self):
        try:
            i = 0
            while self.loop:
                # Put inverse phase.
                gpio.output(self.phase_on[i%self.phases_len], 0)
                gpio.output(self.phase_off[i%self.phases_len], 1)
                time.sleep(self._dt)
                i+=self._dir
        except Exception as e:
            print(e)
        except InterruptedError as e:
            print(e)

# Main code
try:
    # Set gpio mode
    gpio.setmode(gpio.BCM)

    MotorL = SLA7026Cotroller(motor_l_pin,phases)
    MotorR = SLA7026Cotroller(motor_r_pin,phases)

    print('Minimum velocity =',MotorL.getMinVelocity())

    MotorL.start()
    MotorR.start()
    
    t = 0
    for i in range(3000):
        v = (math.sin(t*2)+1)*10
        MotorL.setVelocity(15+v)
        MotorR.setVelocity(15+v)
        time.sleep(0.02)
        t+=0.02

    MotorL.stop()
    MotorR.stop()

    init(motor_l_pin+motor_r_pin)

# Error handling
except Exception as e:
    print('Error :', e)
except KeyboardInterrupt:
    pass
finally:
    init(motor_l_pin+motor_r_pin)
    gpio.cleanup()
    print("GPIO successfully cleaned up.")
