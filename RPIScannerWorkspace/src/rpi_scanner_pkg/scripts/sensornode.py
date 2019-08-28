#!/usr/bin/env python
import rospy
import time
import socketio 
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2
from sensor import Sensor
from pointcloud import PointCloud
from AWS_S3 import * 
import datetime


sio = socketio.Client()


# state variables
streaming = False 


# socket events 
@sio.event
def connect():
    print("I'm connected!")


# ROS subscribers
def onPointCloudRequest(data):
    print("sensor: frame request")
    # capture point cloud for scan
    if (not lidarSensor.snapPointCloud()):
        return
    points2Publisher.publish(lidarSensor.pointCloud.toPointCloud2())
    # take a snapshot for console
    if (not lidarSensor.snapFrame()):
        return
    sio.emit('ON_POINT_CLOUD_STREAM',lidarSensor.frame.tolist() )
    print("sensor: finished frame request")
        
def onFrameRequest(data):
    print("sensor: frame request")
    # upload a frame snapshot
    if (not lidarSensor.snapFrame()):
        return
    sio.emit('ON_POINT_CLOUD_STREAM',lidarSensor.frame.tolist() )
    # save a file to aws s3
    if (not lidarSensor.snapPointCloud()):
        return
    saveToFile(lidarSensor.pointCloud.toPCD())

def onStreamingChangeRequest(streamSensorData) :
    global streaming
    streaming = streamSensorData


# helper methods 
def streamData():
        if (not lidarSensor.snapFrame()):
                return
        sio.emit('ON_POINT_CLOUD_STREAM',lidarSensor.frame.tolist() )
        print("sensor: streamed data")

def saveToFile(s):
        filename = 'snapshot.pcd'
        with open(filename, 'w') as f:
            f.write(s)
            print('saved to: ' + str(filename))
            f.close()
        upload(filename,"Snapshot_PCD", "snapshot"+str(time.time()))
        


# initialization
lidarSensor = Sensor()
rospy.init_node('sensor', anonymous=True)
points2Publisher = rospy.Publisher('points2', PointCloud2, queue_size=10)
points2StreamPublisher = rospy.Publisher('points2_stream', PointCloud2, queue_size=10)

rospy.Subscriber('capture_frame', Bool, onFrameRequest)
rospy.Subscriber('capture_points2', Bool, onPointCloudRequest)
rospy.Subscriber('request_sensor_streaming_change', Bool, onStreamingChangeRequest)

sio.connect('https://capstone-application-server.herokuapp.com') # production
#sio.connect('http://localhost:5000') # developement


def onShutdown():
        lidarSensor.cleanup()

# update loop
period = .05
lastTime = time.time() 
rospy.on_shutdown(onShutdown)
while (not rospy.core.is_shutdown()) : 
        if streaming and time.time() -lastTime > period : 
                streamData()
                lastTime = time.time()
        
