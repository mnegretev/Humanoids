#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np


#splines_z_feet = Float32MultiArray()
def callback_left_foot(data):
    #splines_z_feet = data.data[0]
    print('left - >',data.data)

def callback_right_foot(data):
    #splines_z_feet = data.data[0]
    print('right - >',data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener_splines', anonymous=True)

    rospy.Subscriber("spline_right_foot'", Float32MultiArray, callback_right_foot)
    rospy.Subscriber("spline_left_foot", Float32MultiArray, callback_left_foot)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()