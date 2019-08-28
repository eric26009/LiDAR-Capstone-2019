#!/usr/bin/env python
import rospy
import time
import socketio 
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import PointCloud2, Imu
from sensor import Sensor
from pointcloud import PointCloud
from AWS_S3 import * 
import datetime
# state variables
framesPerRotation = 30
framesTaken = 0
hfov = 64
totalhfov = 360
vfov = 32
lastImu = None
scanFrame = None

# ROS subscribers
def onMotorDoneRotating(data):
    print("stitching node: motor done rotating..")
    print("stitching node: taking next frame..")
    global scanFrame,lastImu,framesTaken
    # start next frame
    if(framesTaken == framesPerRotation):
        saveToFile(toPCD(frameToPointCloud(scanFrame)))
    while(not lidarSensor.snapFrame()): # keep trying until an image is captured
        continue
    addToScan(lidarSensor.frame)
    framesTaken += 1
    print("stitching node: frame added..")
    rotatePlatformPublisher.publish(360 / framesPerRotation)

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
        upload(filename,"Scan_PCD", "scan"+str(time.time()))

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


# core methods
def initializeScan():
    print("stitching node: scan beginning..")
    onMotorDoneRotating(True)

def addToScan(frame):
    print("stitching node: adding frame to scan..")
    global lastImu, scanFrame, framesTaken,hfov
    # check if first scan 
    if(framesTaken == 0):
        scanFrame = frame
        return
    
    # calculate overlap 
    rotationDegrees = 360 / framesPerRotation
    overlapDegrees = hfov - rotationDegrees
    pixelsPerDegree = len(frame) / hfov
    overlapPixels = pixelsPerDegree * overlapDegrees
    
    # add frame to scan frame row by row
    scanFrameRowLength = len(scanFrame)
    for (y, row) in enumerate(frame):
        # average overlap
        frameIndex = 0
        for( i in range(scanFrameRowLength -1 - overlapPixels, scanFrameRowLength)):
            scanFrame[y][i] = (scanFrame[y][i] + row[frameIndex])/2 # TODO consider zero values for proper averaging
        # append extra to scanframe
        rowLength = len(row)
        for( i in range(rowLength - 1 - overlapPixels, rowLength)):
            scanFrame[y].append(row[i])

def frameToPointCloud(frame):
    global totalhfov, vfov
    pointArray = []
    for (y, row) in enumerate(frame):
        for x, depth in enumerate(row):
            if(depth == 0):
                continue
            thetax = radians((totalhfov / float(len(row)))* x) - totalhfov/2
            thetay = radians((vfov / float(len(frame))) * y) - vfov/2
            vx = float(sin(thetax) * depth)
            vy = float(sin(thetay) * depth)
            vz = abs(float(cos(thetay) * depth))
            vector3 = [vx, vy, vz]
            pointArray.append(vector3)
    return pointArray

# initialization
lidarSensor = Sensor()
rospy.init_node('sensor', anonymous=True)
rotatePlatformPublisher = rospy.Publisher('rotate_platform', Float32, queue_size=10)
rospy.Subscriber('motor_done_rotating', Bool, onMotorDoneRotating)
rospy.Subscriber('imu', Imu, onImu)

# wait until first IMU 
while(lastImu == None):
    continue

#start taking images
initializeScan()

def onShutdown():
    lidarSensor.cleanup()

rospy.on_shutdown(onShutdown)
rospy.spin()