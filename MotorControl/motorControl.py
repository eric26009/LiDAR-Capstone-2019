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
        steps = inputDegrees*self.microstep
        MotorControl.stepCount += steps
        MotorControl.degreeCount += inputDegrees
        print("Moving", inputDegrees, "degrees ( steps:",steps, ")\n-----------------------------------------")
        return steps

    def setSpeed(self, stepsPerSecond):
        MotorControl.speed = (1/stepsPerSecond)




def run():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(2, GPIO.OUT) # enable pin
    drv8825 = MotorControl(15,18,2,microstep=16) # GPIO 5 is step, GPIO 6 is direction
    motor = RpiMotorLib.A4988Nema(drv8825.directionPin, drv8825.stepPin, drv8825.pins, "DRV8825")
    microstepString = "1/"+str(drv8825.microstep)
    while (True):
        inputDeg = input("Enter degrees (1 degree increments) to move:")
        if inputDeg is "q":
            break
        if not float(inputDeg).is_integer():
            print("Invalid entry")
            continue

        inputDeg = int(inputDeg)

        if(inputDeg >= 0):
            forwards = True
        else:
            forwards = False
            inputDeg = abs(inputDeg)

        GPIO.output(2, 0)  # enable motor driver
        motor.motor_go(forwards, microstepString, drv8825.moveCommand(inputDeg), MotorControl.speed, False, 0.005)
        time.sleep(0.005)
        GPIO.output(2, 1)  # disable motor driver

        print("Step position:",MotorControl.stepCount, "steps")
        print("Angular position:",MotorControl.degreeCount,"degrees")
        print("Speed:", drv8825.speed, "seconds/step")
        print("Microstep resolution:",drv8825.microstep,"\n-----------------------------------------")
        


run()
GPIO.cleanup()
