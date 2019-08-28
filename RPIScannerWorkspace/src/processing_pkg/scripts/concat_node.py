#!/usr/bin/env python
import rospy
import time
from math import sin, cos, radians
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2, Imu
import tf
import point_cloud2 as pc2
import numpy as np
from ProbabilityMap import ProbabilityMap
from AWS_S3 import *
import socketio
sio = socketio.Client()

lastIMU = Imu()
pointCloud = []
probablityMap = None
isStitching = False
useFilter = False
rospy.init_node('sticher', anonymous=False)

@sio.event
def ON_SET_PROCESSING_FILTER_CHANGE(payload):
    global useFilter
    print("use filter: ",payload)
    useFilter = payload
    

def updateProcessingProgress(prog):
    sio.emit("ON_PROCESSING_PROGRESS_CHANGE",{"progress":prog})

def onStartStitching(data):
    global isStitching,pointCloud, probablityMap
    if(isStitching): 
        return
    # initialize
    sio.emit("ON_PROCESSING_PROGRESS_CHANGE",{"progress":.05})
    initialResidualProbability = 0.35
    residualDissapationRate = 2
    hitProbability = .6
    threshold = 0.95
    groupAmt = 5
    probablityMap = ProbabilityMap(
                        initialResidualProbability,
                        residualDissapationRate,
                        hitProbability,
                        threshold,
                        groupAmt
                        )
    pointCloud = []
    
    # start
    isStitching = True
    
def onFinishStitching(data):
    global isStitching, pointCloud, useFilter
    if(not isStitching):
        return 
    isStitching = False
    
    # save the output
    if(useFilter):
        saveToFile(toPCD(probablityMap.createMap(updateProcessingProgress)))
    else:
        saveToFile(toPCD(pointCloud))

def onImu(data):
    global lastIMU
    lastIMU = data

def onPointCloud(data):
    global isStitching, probablityMap,pointCloud,useFilter
    if(isStitching):
        points = pc2.read_points(data)
        orientation = (lastIMU.orientation.x,lastIMU.orientation.y,lastIMU.orientation.z,lastIMU.orientation.w)
        angle = tf.transformations.euler_from_quaternion(orientation)
        
        for p in points:
            correctedPoint = rotateVector(p,22.5,0) # to correct clouds pointing up...
            rotatedPoint = rotateVector(correctedPoint, -angle[2],2)
            if(useFilter):
                probablityMap.addPointHit(rotatedPoint[0],rotatedPoint[1],rotatedPoint[2])
            else :
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
    filename = 'scan.pcd'
    with open(filename, 'w') as f:
        f.write(s)
        f.close()
    sio.emit("ON_SCAN_UPLOAD_PROGRESS_CHANGE",{"progress":.25})
    upload(filename,"Scan_PCD")
    sio.emit("ON_SCAN_UPLOAD_PROGRESS_CHANGE",{"progress":1})

rospy.Subscriber('imu', Imu, onImu)
rospy.Subscriber('points2', PointCloud2, onPointCloud)
rospy.Subscriber('start_stitching', Bool, onStartStitching)
rospy.Subscriber('finish_stitching', Bool, onFinishStitching)
sio.connect('https://capstone-application-server.herokuapp.com')

# def onShutdown():
#     print("shutdown triggered, saving pointcloud..")
#     global pointCloud,probablityMap
#     saveToFile(toPCD(probablityMap.createMap()))
#     #saveToFile(toPCD(pointCloud))

#rospy.on_shutdown(onShutdown)
rospy.spin()