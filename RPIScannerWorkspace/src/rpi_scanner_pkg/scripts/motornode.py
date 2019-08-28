#!/usr/bin/env python
import rospy
from math import pi,fabs,cos,sin,asin,atan2
from std_msgs.msg import String, Float32,Bool
from sensor_msgs.msg import Imu
from motorControl import Motor,MotorControl


# init ---
rospy.init_node('motor', anonymous=True)
rotationCompletePublisher = rospy.Publisher('motor_done_rotating',Bool,queue_size=10)
lastImu = None
lastDegree = None
platformMotor = Motor(2000.0)


# ROS subscribers ---
def onRotateMotor(amt) :
    print("------------------------")
    print("motor: moved " + str(amt) + " degrees")
    # move motor requested amount
    startAngle = lastDegree
    platformMotor.move(amt.data)
    endAngle = lastDegree
    print("Start angle:", startAngle, "End Angle:", endAngle)
    deltaAngle = endAngle - startAngle
    overshoot = deltaAngle - amt.data
    # oldSpeed = platformMotor.getCurrentSpeed()
    print ("First overshoot:", abs(overshoot))
    while(abs(overshoot) > 0.075):
        # platformMotor.setSpeed(oldSpeed/2)
        newMove = -1.0*(overshoot)
        platformMotor.move(newMove)
        updatedDelta = lastDegree - startAngle
        overshoot = updatedDelta - amt.data
        print("moving:",newMove, "updatedDelta:", updatedDelta, "overshoot:",overshoot )

    # platformMotor.setSpeed(oldSpeed)

    # update the listeners that move was completed
    rotationCompletePublisher.publish(True)

def getCurrentAngleInDegrees():
    global lastImu
    orientation = (lastImu.orientation.x,lastImu.orientation.y,lastImu.orientation.z,lastImu.orientation.w)
    print("orientation",orientation)
    angleInRadians = quaternionToEuler(orientation) # (x,y,z) tuple in radians
    print("angleInRadians",angleInRadians)
    # print(angleInRadians)
    return (angleInRadians[2] * (180/pi)) 

def onImu(data):
    global lastImu
    lastImu = data

def onDegree(data):
    global lastDegree
    lastDegree = data.data

def quaternionToEuler(q):
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    # roll (x-axis rotation)
    sinr_cosp =  2.0 * (qw * qx + qy * qz)
    cosr_cosp =  1.0 - 2.0 * (qx * qx + qy * qy)
    roll = atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if (fabs(sinp) >= 1):
        pitch = copysign(pi / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = atan2(siny_cosp, cosy_cosp)

    return (roll,pitch,yaw)

rospy.Subscriber('rotate_platform', Float32, onRotateMotor)
rospy.Subscriber('imu', Imu, onImu)
rospy.Subscriber('/degree', Float32, onDegree)


# wait and listen ---
rospy.spin()
