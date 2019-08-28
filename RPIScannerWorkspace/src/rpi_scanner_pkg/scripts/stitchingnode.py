#!/usr/bin/env python
import rospy
import time
import numpy as np
import socketio
from math import sin, cos, floor, radians,ceil, pi
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import PointCloud2, Imu
from sensor import Sensor
from pointcloud import PointCloud
from AWS_S3 import *
import datetime
import RPi.GPIO as GPIO
# state variables
framesPerRotation = 10
framesTaken = 0
scanfilename = "scan"+str(time.time())
numberAveragedFrames = 1
hfov = 30
totalhfov = 360.0
vfov = 36
lastImu = None
scanFrame = None


sio = socketio.Client()
@sio.event
def connect():
    print("stitching_node: I'm connected to socket.io!")

@sio.event
def ON_SCAN_TRIGGERED(payload):
    print("stitching_node: scan triggered")
    global scanfilename,framesTaken,framesPerRotation,numberAveragedFrames
    # initialize state 
    framesTaken = 0
    framesPerRotation = int(payload["resolution"])
    scanfilename = str(payload["scanName"]) if len(str(payload["scanName"])) > 0 else "scan"+str(time.time())
    numberAveragedFrames = int(payload["averagedFrames"])

    # intialize enable pin
    GPIO.setup(17, GPIO.OUT) 
    GPIO.output(17, 0)

    #start taking images
    initializeScan()

def ON_SNAPSHOT_FRAME_TRIGGERED(payload):
    print("stitching_node: snapshot triggered")
    while (not lidarSensor.snapFrame()):
        continue
    sio.emit('ON_POINT_CLOUD_STREAM',lidarSensor.frame.tolist() )


# ROS subscribers
def initializeScan():
    global scanFrame,lastImu,framesTaken,numberAveragedFrames
    print("stitching node: scan beginning..")
    # collect frames  
    addToScan(lidarSensor.getTimeAveragedFrameProperly(numberAveragedFrames))
    

    # update state
    framesTaken += 1
    sio.emit("ON_SCAN_PROGRESS_CHANGE", {"progress": float(framesTaken) / float(framesPerRotation)})
    print("stitching node: frame added..")

    # next rotation
    rotatePlatformPublisher.publish(totalhfov / framesPerRotation)

def onMotorDoneRotating(data):
    global scanFrame,lastImu,framesTaken,numberAveragedFrames
    print("stitching node: motor done rotating..")
    print("stitching node: taking next frame: ",framesTaken )
    # check if done
    if(framesTaken >= framesPerRotation):
        saveToFile(toPCD(frameToPointCloud(scanFrame)))
        GPIO.output(17, 1)
        return
    addToScan(lidarSensor.getTimeAveragedFrameProperly(numberAveragedFrames))

    
    # update state
    framesTaken += 1
    sio.emit("ON_SCAN_PROGRESS_CHANGE", {"progress": float(framesTaken) / float(framesPerRotation)})
    sio.emit('ON_POINT_CLOUD_STREAM',lidarSensor.frame.tolist()) 
    print("stitching node: frame added..")

    # next rotation
    rotatePlatformPublisher.publish(360.0 / framesPerRotation)

def onImu(data):
    # store the imu data for verifying
    global lastImu
    lastImu = data


# helper methods
def saveToFile(s):
        filename = 'snapshot.pcd'
        with open(filename, 'w') as f:
            f.write(s)
            print('saved to: ' + str(filename))
            f.close()
        sio.emit("ON_SCAN_UPLOAD_PROGRESS_CHANGE",{"progress":.25})
        upload(filename,"Scan_PCD", scanfilename)
        sio.emit("ON_SCAN_UPLOAD_PROGRESS_CHANGE",{"progress":1})

def toPCD(points):
    print("converting to pcd..")
    outputList = []
    pointCount = len(points)
    outputList.append('# .PCD v.7 - Point Cloud Data file format')
    outputList.append('VERSION .7')
    outputList.append('FIELDS x y z')
    outputList.append('SIZE 4 4 4')
    outputList.append('TYPE F F F')
    outputList.append('WIDTH {}'.format(pointCount))
    outputList.append("HEIGHT 1")
    outputList.append('VIEWPOINT 0 0 0 1 0 0 0')
    outputList.append('POINTS {}'.format(pointCount))
    outputList.append('DATA ascii')
    for p in points:
        outputList.append(' '.join(map(str, p)))
    return '\n'.join(outputList)


def addToScan(frame):
    print("stitching node: adding frame to scan..")
    global lastImu, scanFrame, framesTaken,hfov
    # check if first scan
    if(framesTaken == 0):
        scanFrame = frame
        return

    # average
    # calculate overlap
    rotationDegrees = totalhfov / framesPerRotation
    overlapDegrees = hfov - rotationDegrees
    pixelsPerDegree = len(frame[0]) / hfov
    overlapPixels = int(ceil(pixelsPerDegree * overlapDegrees))
    # # initialize
    # inputFrame = frame[:,:overlapPixels]
    # frameSum = scanFrame[:,len(scanFrame[0])-overlapPixels:]
    # frameCount = np.zeros_like(inputFrame)
    # MIN_VALID_DEPTH = 1
    # 
    # # mask out the small values and accumulate the others
    # depthMasked = np.ma.masked_less(inputFrame, MIN_VALID_DEPTH, copy=True)
    # frameCount[depthMasked.mask == False] += 1
    # frameSum[depthMasked.mask == False] += inputFrame[depthMasked.mask == False]
    # # For elements with zero count, sum is also zero. Set count=1 to avoid div0
    # frameCount[frameCount==0] = 1
    # frameAverage = frameSum / frameCount
    # scanFrame = np.concatenate((scanFrame[:,:len(scanFrame[0])-overlapPixels], frameAverage), axis=1)

    # append extra to scanframe
    trimmedFrame = frame[:,overlapPixels:]
    scanFrame = np.concatenate((scanFrame,trimmedFrame), axis=1)


def frameToPointCloud(frame):
    GPIO.setmode(GPIO.BCM )
    GPIO.setup(17, GPIO.OUT) # enable pin
    GPIO.output(17, 1)

    print("stitching node: parsing now...")
    global totalhfov, vfov
    pointArray = []
    width = float(len(frame[0]))
    height = float(len(frame))
     
    sio.emit("ON_PROCESSING_PROGRESS_CHANGE",{"progress":1/3})
    for index,depth in np.ndenumerate(frame):
            if(depth == 0):
                continue
            x = index[1]
            y = index[0]
            thetax = radians((totalhfov / width ) * x )
            thetay = radians((vfov / height) * y - vfov/2)

            # # calculate a correction vector to fix offset of sensor
            # offsetAmount = 10
            # unitVector = [0,0,offsetAmount]
            # correctionVector = rotateVector(unitVector,radians(thetax),1)

            # vx = correctionVector[0] + float(sin(thetax) * depth)
            # vy = correctionVector[1] + float(cos(thetax) * depth)
            # vz = correctionVector[2] + float(sin(thetay) * depth)

            pointArray.append([-float(sin(thetax) * depth), float(cos(thetax) * depth), float(sin(thetay) * depth)])
    sio.emit("ON_PROCESSING_PROGRESS_CHANGE",{"progress":1})
    return pointArray

def rotateVector(vec, rad, axis):
    # x-axis = 0
    # y-axis = 1
    # z-axis = 2
    rotationMatrix = []
    if(axis == 0): #x
        rotationMatrix = [[1,0,0],[0,cos(rad),-sin(rad)],[0,sin(rad),cos(rad)]]
    elif(axis == 1): #y
        rotationMatrix = [[cos(rad),0,sin(rad)],[0,1,0],[-sin(rad),0,cos(rad)]]
    elif(axis == 2): #z
        rotationMatrix = [[cos(rad),-sin(rad),0],[sin(rad),cos(rad),0],[0,0,1]]
    else:
        print("error: invalid axis...")
        return []
    return  np.matmul(vec,rotationMatrix)

# initialization
lidarSensor = Sensor()
rospy.init_node('sensor', anonymous=True)
rotatePlatformPublisher = rospy.Publisher('rotate_platform', Float32, queue_size=10)
rospy.Subscriber('motor_done_rotating', Bool, onMotorDoneRotating)
rospy.Subscriber('imu', Imu, onImu)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT) # enable pin
GPIO.output(17, 1)

sio.connect('https://capstone-application-server.herokuapp.com')

# wait until first IMU
while(lastImu == None):
    continue

def onShutdown():
    lidarSensor.cleanup()

rospy.on_shutdown(onShutdown)
rospy.spin()
