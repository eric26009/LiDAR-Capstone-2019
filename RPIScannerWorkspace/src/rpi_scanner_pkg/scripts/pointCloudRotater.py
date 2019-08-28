#!/usr/bin/env python
import rospy
import time
from math import sin, cos, radians
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import Header
import tf
import point_cloud2 as pc2
import numpy as np

# state variables
lastIMU = Imu()

# initialize ros stuff
rospy.init_node('rotator', anonymous=False)
blamPublisher = rospy.Publisher('velodyne_points', PointCloud2, queue_size=10)

# topic subscribers
def onImu(data):
    global lastIMU
    lastIMU = data

def onPointCloud(data):
    # convert pointcloud2 to an array and get rotation in radians
    points = pc2.read_points(data) 
    orientation = (lastIMU.orientation.x,lastIMU.orientation.y,lastIMU.orientation.z,lastIMU.orientation.w)
    angle = tf.transformations.euler_from_quaternion(orientation)
    # initialize point cloud
    pointCloud = []
    # rotate every point in point cloud 
    for p in points:
        correctedPoint = rotateVector(p,22.5,0) # to correct clouds pointing up...
        rotatedPoint = rotateVector(correctedPoint, -angle[2],2)
        pointCloud.append(rotatedPoint)
    # create pointcloud2 message and publish for BLAM
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "lidar_link"
    blamPublisher.publish(pc2.create_cloud_xyz32(header,pointCloud))

# helper methods
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

# final ros setup
rospy.Subscriber('imu', Imu, onImu)
rospy.Subscriber('points2', PointCloud2, onPointCloud)
rospy.spin()