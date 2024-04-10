#!/usr/bin/env python
import rospy
import numpy
import cv2
import math
import random
import argparse
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from cv_bridge import CvBridge   
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point32

def head_k_generator(centroid_msg):
    center_x=centroid_msg.x
    center_y=centroid_msg.y
    Cx=320
    Cy=240
    ex=center_x-Cx
    ey=center_y-Cy
    Kpan=1/320
    Ktilt=0.5/240
    if ex>0: #Ball is at right
        pan=(Kpan*ex)*(-1)
        if ey<0: #Ball is up right
            tilt=(Ktilt*ey)*(-1)
            print ("Ball is up right : ",[pan,tilt])
        if ey>0: #Ball is down right
            tilt=(Ktilt*ey)
            print ("Ball is down right : ",[pan,tilt])
    if ex<0: #Ball is at left
        pan=(Kpan*ex)
        if ey<0: #Ball is up left
            tilt=(Ktilt*ey)*(-1)
            print ("Ball is up left : ",[pan,tilt])
        if ey>0: #Ball is down left
            tilt=(Ktilt*ey)
            print ("Ball is down left : ",[pan,tilt])
    if 315 <= center_x <= 325 and 235 <= center_y <= 245:
        print ("Ball is centering : ",[pan,tilt])


def main ():
    rospy.init_node("head_goal_ball_position_node")  
    rospy.Subscriber("/centroid_publisher", Point32, head_k_generator)
    rospy.spin()

if __name__=="__main__":
    main()