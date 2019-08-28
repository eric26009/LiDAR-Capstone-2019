#!/usr/bin/env python
import rospy
from std_msgs.msg import String

rospy.init_node('test', anonymous=False)
def subscriberExample(data):
    print("data")
    
rospy.Subscriber('testnode', String, subscriberExample)
rospy.spin()