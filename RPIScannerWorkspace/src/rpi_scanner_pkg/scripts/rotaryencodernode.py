#!/usr/bin/python
import rospy
import math
import RPi.GPIO as GPIO
from time import time
from std_msgs.msg import String, Header, Float32
from sensor_msgs.msg import Imu

class Encoder:
    def __init__(self, pinAin, pinBin):
        print("Setting up Rotary Encoder...")
        # init pins
        self.pinA = pinAin
        self.pinB = pinBin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pinA, GPIO.IN)
        GPIO.setup(self.pinB, GPIO.IN)
        # init vars
        self.count = 0
        self.degrees = 0
        self.lastDegrees = 0
        self.lastTime = time()
        self.period = 1/5

        self.lastMeasurementTime = time()
        self.lastAB = 0b00
        self.outcome = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0]
        # init ros
        rospy.init_node('imu', anonymous=True)
        self.imuPublisher = rospy.Publisher('/imu', Imu, queue_size=10)
        self.degreePublisher = rospy.Publisher('/degree',Float32,queue_size=10)
        self.seq = 0
        # run until closed
        while not rospy.is_shutdown():
            self.runEncoder()
            if(time() - self.lastTime > self.period):
                self.imuPublisher.publish(self.createMessage())
                self.degreePublisher.publish(self.degrees)
                self.seq += 1
                self.lastTime = time()

        # clean up
        GPIO.cleanup()

    def runEncoder(self):
        currentA = GPIO.input(self.pinA)
        currentB = GPIO.input(self.pinB)
        currentAB = (currentA << 1) | currentB
        position = (self.lastAB << 2) | currentAB
        self.count += self.outcome[position]
        self.lastAB = currentAB
        # encoder is 200 counts/turn *2 for AB phases * 1.8 ratio = 720
        self.degrees = (self.count/1440.0)*360.0

    def createMessage(self):
        xQ, yQ, zQ, wQ = self.toQuaternion(self.degrees)

        deltaAngle = math.radians(float(self.degrees - self.lastDegrees))
        deltaTime = float(self.lastMeasurementTime - time())
        wz =  deltaAngle / deltaTime
        self.lastMeasurementTime = time()
        self.lastDegrees = self.degrees
        
        data = Imu()
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = "imu_link"
        data.header.seq = self.seq
        data.orientation.w = wQ
        data.orientation.x = xQ
        data.orientation.y = yQ
        data.orientation.z = zQ
        data.orientation_covariance = [0,0,0,0,0,0,0,0,0]
        data.linear_acceleration.x = 0.0
        data.linear_acceleration.y = 0.0
        data.linear_acceleration.z = 9.8
        data.linear_acceleration_covariance = [0,0,0,0,0,0,0,0,0]
        data.angular_velocity.x = 0.0
        data.angular_velocity.y = 0.0
        data.angular_velocity.z = wz
        data.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]
        return data

    def toQuaternion(self, degrees):
        x = 0
        y = 0
        z = degrees
        cosZ = math.cos(math.radians(z) * 0.5)
        sinZ = math.sin(math.radians(z) * 0.5)
        cosY = math.cos(math.radians(y) * 0.5)
        sinY = math.sin(math.radians(y) * 0.5)
        cosX = math.cos(math.radians(x) * 0.5)
        sinX = math.sin(math.radians(x) * 0.5)

        wQ = cosZ * cosY * cosX + sinZ * sinY * sinX
        xQ = cosZ * cosY * sinX - sinZ * sinY * cosX
        yQ = sinZ * cosY * sinX + cosZ * sinY * cosX
        zQ = sinZ * cosY * cosX - cosZ * sinY * sinX

        return xQ, yQ, zQ, wQ


Encoder(27,22)
