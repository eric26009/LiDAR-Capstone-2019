#!/usr/bin/env python
import threading
import rospy
import rosbag
import json
import point_cloud2 as pc2
from time import time, sleep
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import PointCloud2, Imu
import socketio  # pip install "python-socketio[client]"
import RPi.GPIO as GPIO

from AWS_S3 import *
from AWS_EC2 import *
import requests
import os
import sys


# state variables ---
lastImu = None
lastPoints2 = None
snapsPerRotation = 12
scanningInProcess = False
lastSnapTaken = 0
totalSnapsToTake = 0
bagfilename = "bag"


# init ---
rospy.init_node('main', anonymous=False)
rotatePlatformPublisher = rospy.Publisher('rotate_platform', Float32, queue_size=10)
sensorPoints2EventHandler = rospy.Publisher('capture_points2', Bool, queue_size=10)
sensorFrameEventHandler = rospy.Publisher('capture_frame', Bool, queue_size=10)
sensorStreamingChangeEventHandler = rospy.Publisher('request_sensor_streaming_change', Bool, queue_size=10)


# socket events ---
sio = socketio.Client()
@sio.event
def connect():
    print("I'm connected!")
    sio.emit('ON_LIDAR_STATE_CHANGED', "payload")
    # waiting untill ec2 is either already running or stopped
    waitCount = 0
    while checkRunning() == 2 and  waitCount < 300: # will try for 5 min
        waitCount = waitCount + 1
        sleep(1) # sleep for a sec
        if waitCount == 300:
            print "Error starting EC2, reboot system and check AWS EC2"
            sys.exit(0)

    ec2_start()
    os.system("/home/pi/Documents/capstone-slam/EC2BOOT.sh")

@sio.event
def ON_MOVE_BY_STEPS_TRIGGERED(payload):
    print("move by steps triggered")
    rotatePlatformPublisher.publish(3)

@sio.event
def ON_SNAPSHOT_POINTS2_TRIGGERED(payload):
    print("snapshot triggered")
    sensorPoints2EventHandler.publish(True)

@sio.event
def ON_SNAPSHOT_FRAME_TRIGGERED(payload):
    print("snapshot frame triggered")
    sensorFrameEventHandler.publish(True)
    return

@sio.event
def ON_REQUEST_POINT_CLOUD_STREAM(payload):
    print("request stream change with: " + str(payload))
    sensorStreamingChangeEventHandler.publish(payload)
    return

@sio.event
def ON_SCAN_TRIGGERED(payload):
    print("main: scan initiated")
    # update state
    global scanningInProcess,lastSnapTaken,lastPoints2,lastImu,totalSnapsToTake,bag,snapsPerRotation, bagfilename
    bag = rosbag.Bag('test.bag', 'w')
    snapsPerRotation = int(payload["resolution"])
    bagfilename = str(payload["scanName"])
    scanningInProcess = True
    lastSnapTaken = 0
    lastPoints2 = None
    lastImu = None
    totalSnapsToTake = snapsPerRotation
    GPIO.setup(17, GPIO.OUT) # enable pin
    GPIO.output(17, 0)
    # request a points2
    sensorPoints2EventHandler.publish(True)
    

# ROSBAG (syncronized) ---
bag = rosbag.Bag('test.bag', 'w')
lock = threading.Lock()

def addRotationDataToBag(data):
    lock.acquire()
    global scanningInProcess
    if(scanningInProcess):
        bag.write('imu', data)
    lock.release()

def addPointCloudToBag(data):
    lock.acquire()
    global scanningInProcess
    if(scanningInProcess):
        bag.write('points2', data)
    lock.release()

# ROS subscribers ---
def onPointCloud(data):
    global scanningInProcess,lastSnapTaken,lastPoints2,snapsPerRotation, bagfilename
    if scanningInProcess :
        # save points2
        addPointCloudToBag(data)
        # update the 
        sio.emit("ON_SCAN_PROGRESS_CHANGE", {"progress": float(lastSnapTaken) / float(snapsPerRotation)})
         # check if finished
        if lastSnapTaken == totalSnapsToTake :
            print("main: scan finished" )
            # lock for the sake of scanningInProgress
            lock.acquire()
            scanningInProcess = False
            # clean up
            GPIO.output(17, 1)
            bag.close()
            lock.release()
            print("mainnode: start upload to aws")
            # updload status starting
            sio.emit("ON_DATA_UPLOAD_PROGRESS_CHANGE", {"progress": .25})
            # upload bag
            key = upload("test.bag","Bag_files", bagfilename)
            # updload status complete
            sio.emit("ON_DATA_UPLOAD_PROGRESS_CHANGE", {"progress": 1})
            print("mainnode: done upload to aws")
            # tell processing server to begin processing the new bag
            requests.get(url="http://ec2-54-156-115-99.compute-1.amazonaws.com:3000/process/"+str(key)+".bag")
        else :
            # begin next iteration and update snap counter
            lastSnapTaken += 1
            rotatePlatformPublisher.publish(360 / snapsPerRotation)

def onMotorDoneRotating(data):
    print("motor_done_rotating event")
    # request a points2
    sensorPoints2EventHandler.publish(True)

def onImu(data):
    # store the imu data for verifying

    global lastImu
    lastImu = data
    addRotationDataToBag(lastImu)

def onPointCloudStream(pointCloud) : # not in use
    print("streaming point_cloud")
    points_list = []
    for data in pc2.read_points(pointCloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2]])
    sio.emit('ON_POINT_CLOUD_STREAM',points_list )

rospy.Subscriber('motor_done_rotating', Bool, onMotorDoneRotating)
rospy.Subscriber('imu', Imu, onImu)
rospy.Subscriber('points2', PointCloud2, onPointCloud)
rospy.Subscriber('points2_stream', PointCloud2, onPointCloudStream)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT) # enable pin
GPIO.output(17, 1)
# Connect to server ---
sio.connect('https://capstone-application-server.herokuapp.com')
#sio.connect('http://localhost:5000')

# clean up
def onShutdown():
    GPIO.output(17, 1)
    bag.close()
    ec2_stop()

rospy.on_shutdown(onShutdown)
# wait until closed with ctrl-c
rospy.spin()