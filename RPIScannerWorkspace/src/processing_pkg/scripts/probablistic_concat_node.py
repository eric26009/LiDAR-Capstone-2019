#!/usr/bin/env python
import rospy
import time
from math import sin, cos, radians
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2, Imu
import tf
import point_cloud2 as pc2
import numpy as np
from AWS_S3 import *

lastIMU = Imu()
pointCloud = []
isStitching = False
rospy.init_node('sticher', anonymous=False)

def onStartStitching(data):
    global isStitching
    if(isStitching):
        return
    isStitching = True
    pointCloud = []



def onFinishStitching(data):
    global isStitching
    if(not isStitching):
        return
    isStitching = False
    saveToFile(toPCD(pointCloud))

def onImu(data):
    global lastIMU
    lastIMU = data

def onPointCloud(data):
    global isStitching
    if(isStitching):
        points = pc2.read_points(data)
        orientation = (lastIMU.orientation.x,lastIMU.orientation.y,lastIMU.orientation.z,lastIMU.orientation.w)
        angle = tf.transformations.euler_from_quaternion(orientation)

        for p in points:
            correctedPoint = rotateVector(p,22.5,0) # to correct clouds pointing up...
            rotatedPoint = rotateVector(correctedPoint, -angle[2],2)
            pointCloud.append(rotatedPoint)

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

def saveToFile(s):
    print("saving to file..")
    filename = 'snapshot.pcd'
    with open(filename, 'w') as f:
        f.write(s)
        print('saved to: ' + str(filename))
        f.close()
    upload(filename,"Snapshot_PCD")

rospy.Subscriber('imu', Imu, onImu)
rospy.Subscriber('points2', PointCloud2, onPointCloud)
rospy.Subscriber('start_stitching', Bool, onStartStitching)
rospy.Subscriber('finish_stitching', Bool, onFinishStitching)

def onShutdown():
    print("shutdown triggered, saving pointcloud..")
    global pointCloud
    saveToFile(toPCD(pointCloud))

#rospy.on_shutdown(onShutdown)
rospy.spin()
