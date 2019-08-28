import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import time

# RpiMotorLib resource
# https://github.com/gavinlyonsrepo/RpiMotorLib/blob/master/Documentation/Nema11DRV8825.md

class MotorControl:
    stepCount = 0
    degreeCount = 0
    steps_second = 1500.0
    speed = (1/steps_second)  # seconds/step

    def __init__ (self, stepPin, directionPin, enablePin, microstep=16):
        print("Setting up motor...")
        self.stepPin = stepPin
        self.directionPin = directionPin
        self.enablePin = enablePin
        self.pins = (0, 0, 0) # dummy, supposed to be for mode control
        self.microstep = microstep
        self.stepsForFullRotation = microstep*360

    def moveCommand(self,inputDegrees):
        steps = inputDegrees * self.microstep
        MotorControl.stepCount += steps
        MotorControl.degreeCount += inputDegrees
        
        return int(steps)

    def setSpeed(self, stepsPerSecond):
        MotorControl.speed = (1/stepsPerSecond)

    def getSpeed(self):
        return MotorControl.speed


class Motor:
    def __init__ (self, stepsPerSecond):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.OUT) # enable pin
        self.drv8825 = MotorControl(3,4,17,microstep=16) # GPIO 5 is step, GPIO 6 is direction
        self.drv8825.setSpeed(stepsPerSecond)
        self.motor = RpiMotorLib.A4988Nema(self.drv8825.directionPin, self.drv8825.stepPin, self.drv8825.pins, "DRV8825")
        self.microstepString = "1/"+str(self.drv8825.microstep)
        GPIO.output(17, 0)  # enable motor driver

    def move(self, degrees):
        print("motor control: moving..")

        if(degrees >= 0):
            forwards = True
        else:
            forwards = False
            degrees = abs(degrees)
        print("motor control: forwards=", forwards)
        steps = self.drv8825.moveCommand(degrees)
        print("motor control: steps=", steps)
        self.motor.motor_go(forwards, self.microstepString, steps, MotorControl.speed, False, 0.5)
        print("motor control: moved")
        #time.sleep(0.005)
        #GPIO.output(17, 1)  # disable motor driver

        # print("Step position:",MotorControl.stepCount, "steps")
        # print("Angular position:",MotorControl.degreeCount,"degrees")
        # print("Speed:", self.drv8825.speed, "seconds/step")
        # print("Microstep resolution:",self.drv8825.microstep,"\n-----------------------------------------")

    def setSpeed(self, steps_per_second):
        self.drv8825.setSpeed(steps_per_second)

    def getCurrentSpeed(self):
        return self.drv8825.getSpeed()

    def cleanup(self):
        GPIO.cleanup()
