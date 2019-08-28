#!/usr/bin/env python
import rospy
import subprocess
import boto3
from std_msgs.msg import String, Bool
from AWS_S3 import *

startScanningPub = rospy.Publisher('start_stitching', Bool, queue_size=10)
stopScanningPub = rospy.Publisher('finish_stitching', Bool, queue_size=10)
rospy.init_node('data', anonymous=False)

def onS3Key(msg):
    filename = msg.data
    startScanningPub.publish(True)
    download(filename,"Bag_files")
    subprocess.call(["rosbag play -r 10 '" + str(filename) + "'"], shell=True)
    stopScanningPub.publish(True)

rospy.Subscriber('s3bagkey', String, onS3Key)
rospy.spin()
