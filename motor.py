import sys
import time
import math
import threading
import numpy as np
import RPi.GPIO as gpio

# Class for step motor control
class StepMotor:
    # pins : list of pins to be used for step motor. pins must be in order.
    # phases : list of phase which is list that contains pins to be turned on.
    def __init__(self, pins:list, phases:list):
        self.pins = pins
        self.phase = 0

        # Set pin mode
        gpio.setup(channel=pins,direction=gpio.OUT)

        # Generate phase and inverse phase array
        phasesOn = []
        phasesOff = []
        for phase in phases:
            phaseOn = []
            phaseOff = []
            for i in range(len(pins)):
                if i in phase:
                    phaseOn.append(pins[i])
                else:
                    phaseOff.append(pins[i])
            phasesOn.append(phaseOn)
            phasesOff.append(phaseOff)

        # You can easily set the phase of the stepmotor by turning off the pins in phases_ and then
        # turn on the pins in phases.
        self.phases = phasesOn
        self.phases_ = phasesOff

        self._phase_len = len(phases)
        self.enable(False)

    # Turn on the pins in current phase and turn of the ohters.
    def _set(self):
        gpio.output(self.phases_[self.phase], 0)
        gpio.output(self.phases[self.phase], 1)

    # If enable is True, hold phase.
    # Else, relase phase.
    def enable(self,e: bool):
        if e:
            self._set()
        else:
            gpio.output(self.pins, 0)

    # If d is positive, go to next phase.
    # Else, go to previous phase
    def forward(self,d:int = 1):
        if d > 0:
            self.phase += 1
        else:
            self.phase -= 1
        self.phase %= self._phase_len
        self._set()

class StepMotorTask:
    # r : the radius of the wheel in meter
    # step : the number of steps in one rotation 
    def __init__(self,motor:StepMotor,r=0.025,step=400):
        self.motor = motor
        self.velocity = 0
        self.r = r
        self.step = step

        # angular velocity = dT/dt
        # velocity = angular velocity * radious = r * dT / dt
        # :. velocity = r * 2 * pi / ( step * dt )
        # Let k = r * 2 * pi / step
        # :. velocity = k / dt
        self.k = self.r*2*3.14159265358979323846264338327/self.step

        self._thread =None
        self._running = False

    # Start thraed task
    def start(self):
        if self._running:
            return False
        if self._thread!=None:
            self._thread.join()
        self._running = True
        self._thread = threading.Thread(target=self._threadTask)
        return True

    # Stop running thread
    def stop(self):
        if not self._running:
            return False
        self._running = False
        self._thread.join()
        self.motor.enable(False)
        self._thread = None
        return True

    # ThreadTask.
    def _threadTask(self):
        # :. velocity = k / dt
        # :. dt = k / velocity
        while self._running:
            dt = self.k/self.velocity
            if abs(dt)>0.01:
                # If motor speed is too slow, stop motor for safty.
                self.motor.enable(False)
            time.sleep(dt)
            self.motor.forward(self.velocity)
        self.motor.enable(False)


if __name__ == "__main__":

    # Version check
    v = sys.version_info
    print('==== [ INFORMATION ] ====')
    print('Current python version : ', v[0], '.', v[1], '.', v[2], sep='')
    print()

    # You must set GPIO mode before ouse gpio-associate functions.
    gpio.setmode(gpio.BCM)

    # GPIO number(=BCM). (not BOARD)
    pin_motor_l = [2, 3, 4, 17]
    pin_motor_r = [18, 27, 22, 23]
    phases = [[0], [1], [2], [3]]


    try:
        motorLeft = StepMotor(pin_motor_l,phases)
        taskLeft = StepMotorTask(motorLeft)

        while True:
            try:

                # Get command from commandline
                command = input()
                if command == 'exit' or command =='exit()' or command=='^c' or command=='^C':
                    break
            
                # Parse command
                args = input().split()[0]

                command = args[0]
                params = []
                if len(args)>1:
                    params = args[1:]

                if command == 'E':
                    taskLeft.start()
                    print('Motor enabled')

                elif command =='D':
                    taskLeft.stop()
                    print('Motor disabled')

                elif command =='V':
                    v = float(params[0])
                    taskLeft.velocity = v
                    print('Set motor speed to',v)

                else:
                    print('Got Unknown command :',command)

            except Exception as e:
                print('Wrong command :',e)

    except  KeyboardInterrupt as e:
        print('Program stopped by user interrupt')

    except Exception as e:
        print('Program stopped by exception :',e)

    finally:
        # You must cleanup gpio before end prcess for safty.
        motorLeft.enable(False)
        gpio.cleanup()
        print('GPIO successfully cleaned up.')
