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
from std_msgs.msg import Float32MultiArray
def head_k_generator(centroid_msg):
    center_x=centroid_msg.x
    center_y=centroid_msg.y
    head_cmd = Float32MultiArray()
    Cx=320
    Cy=240
    ex=center_x-Cx
    ey=center_y-Cy
    Kpan=0.5/320
    Ktilt=0.25/240
    while center_x!=0 and center_y!=0:
        if ex > 0 and ey < 0: #Ball is up right
            pan=(Kpan*ex)
            tilt=(Ktilt*ey)
            print ("Ball is up right : ",[pan,tilt])
            print ("Center is: ",[center_x,center_y])
            goal=[pan,tilt]
            head_cmd.data=goal
            pub_head_goal.publish(head_cmd) 
        elif ex > 0 and ey > 0: #Ball is down right
            pan=(Kpan*ex)
            tilt=(Ktilt*ey)
            print ("Ball is down right : ",[pan,tilt])
            print ("Center is: ",[center_x,center_y])
            goal=[pan,tilt]
            head_cmd.data=goal
            pub_head_goal.publish(head_cmd) 
        elif ex < 0 and ey < 0: #Ball up left
            pan=(Kpan*ex)
            tilt=(Ktilt*ey)
            print ("Ball is up left : ",[pan,tilt])
            print ("Center is: ",[center_x,center_y])
            goal=[pan,tilt]
            head_cmd.data=goal
            pub_head_goal.publish(head_cmd)
        elif ex < 0 and ey > 0: #Ball is down left
            pan=(Kpan*ex)
            tilt=(Ktilt*ey)
            print ("Ball is down left : ",[pan,tilt])
            print ("Center is: ",[center_x,center_y])
            goal=[pan,tilt]
            head_cmd.data=goal
            pub_head_goal.publish(head_cmd) 
        elif 315 <= center_x <= 325 and 235 <= center_y <= 245:
            print ("Ball is centering : ",[center_x,center_y]) 

        else:
            print("Any ball detected")
        break        
        #return pan,tilt
def main ():
    global pub_head_goal,rate
    rospy.init_node("head_goal_ball_position_node")  
    rospy.Subscriber("/centroid_publisher", Point32, head_k_generator)
    pub_head_goal = rospy.Publisher("/head_goal_pose", Float32MultiArray, queue_size=1)   
    rospy.spin()

if __name__=="__main__":
    main()
