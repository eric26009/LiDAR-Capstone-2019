import RPi.GPIO as GPIO
import time
import socketio  # pip install "python-socketio[client]"


class MotorControl:

    stepCount = 0
    degreeCount = 0

    def __init__(self, stepPin, directionPin, enablePin, microStep=16):
        self.stepPin = stepPin
        self.directionPin = directionPin
        self.enablePin = enablePin
        self.microStep = microStep
        # GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(directionPin, GPIO.OUT)
        GPIO.setup(enablePin, GPIO.OUT)
        GPIO.setup(stepPin, GPIO.OUT)
        GPIO.output(directionPin, 1)
        GPIO.output(enablePin, 1)   # disabling the motor while in standby

    #desiredRate: steps/sec

    def moveDeg(self, degrees, desiredRate, acceleration):
        degToSteps = (degrees * self.microStep)
        self.move(degToSteps, degrees, desiredRate, acceleration)

    def move(self, steps, desiredRate, acceleration):
        # init
        GPIO.output(self.enablePin, 0)  # enabling motor
        time.sleep(0.001)
        desiredRateInMicro = desiredRate
        accelerationInMicro = acceleration
        currentRate = 0
        startTime = time.time()
        lastTime = startTime
        stepCount = 0
        signalHigh = False
        stepsToAccel = pow(desiredRateInMicro, 2) / (2*accelerationInMicro)
        decelTime = 0
        capturedDecelTime = False
        print(str(stepsToAccel) + "steps")

        if(steps < stepsToAccel * 2):
            raise Exception(
                "not enough steps to accomidate acceleration curve")

        # transmit signal until finished
        while(stepCount < steps):

            # update speed
            if(stepCount <= stepsToAccel):  # accelerating
                currentRate = accelerationInMicro * (time.time() - startTime)
            elif(stepCount < steps - stepsToAccel):  # at desired speedd
                currentRate = desiredRateInMicro
            else:  # decelerating
                if(not capturedDecelTime):
                    decelTime = time.time()
                    capturedDecelTime = True

                currentRate = desiredRateInMicro - \
                    accelerationInMicro * (time.time() - decelTime)

            # do work
            period = (1/currentRate)
            if(time.time() - lastTime >= period / 2):
                lastTime = time.time()
                stepCount += .5
                signalHigh = not signalHigh
                GPIO.output(
                    self.stepPin, GPIO.HIGH if signalHigh else GPIO.LOW)
        time.sleep(0.001)
        MotorControl.stepCount += steps
        MotorControl.degreeCount += steps/self.microStep
        GPIO.output(self.enablePin, 1)  # disabling motor


motor = MotorControl(15, 18, 2)

# listen for events from server
sio = socketio.Client()
@sio.event
def onMoveByStepsTriggered(payload):
    print("move by steps triggered")
    steps = int(payload["steps"])
    rate = int(payload["rate"])
    accel = int(payload["acceleration"])
    #motor.move(5760, 10000, 25000)
    motor.move(steps, rate, accel)
    #GPIO.cleanup()

motor.move(5760, 3333, 8333)0)
sio.connect('https://capstone-application-server.herokuapp.com')
