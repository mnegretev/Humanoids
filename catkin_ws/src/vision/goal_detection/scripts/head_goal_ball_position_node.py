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

new_head_pose=[0.00,0.00]

def head_k_generator(centroid_msg):
    rate=rospy.Rate(4)
    ball_found=Bool
    global new_head_pose
    center_x=centroid_msg.x
    center_y=centroid_msg.y
    Cx=320
    Cy=240
    ex=center_x-Cx
    ey=center_y-Cy
    Kpan=-0.07/320
    Ktilt=0.07/240
    center_x0=320
    center_y0=240
    if center_x==0 and center_y==0:
        for i in range(10):
            center_x0+=20
            if 0<center_x0<521:
                print(f"Looking for a ball... in {center_x0}")
                ex0=center_x0-Cx
                ey0=center_y0-Cy
                pan = (ex0*Kpan)
                tilt = (ey0*Ktilt)
            if center_x0>=520:
                print("Returning...")
                ex0=center_x0-Cx
                ey0=center_y0-Cy
                pan = (ex0*Kpan)*-1
                tilt = (ey0*Ktilt)

            new_head_pose = [sum(x) for x in zip(new_head_pose, [pan, tilt])]
            head_cmd = Float32MultiArray()
            goal=new_head_pose
            head_cmd.data=goal
            pub_head_goal.publish(head_cmd) 
            rate.sleep()
    else:
        pan = (ex*Kpan)
        tilt = (ey*Ktilt)
        print(f"Error ex is {ex}")
        print(f"Error ey is {ey}")
        print(f"Moving pan angle by {pan} radians")
        print(f"Moving tilt angle by {tilt} radians")

        new_head_pose = [sum(x) for x in zip(new_head_pose, [pan, tilt])]
    
    print(new_head_pose)
    head_cmd = Float32MultiArray()
    goal=new_head_pose
    head_cmd.data=goal
    pub_head_goal.publish(head_cmd) 
    

def main ():
    global pub_head_goal,ball_found_pub
    rospy.init_node("head_goal_ball_position_node")    
    rospy.Subscriber("/centroid_publisher", Point32, head_k_generator)
    pub_head_goal = rospy.Publisher("/hardware/head_goal_pose", Float32MultiArray, queue_size=1)  
    ball_found_pub = rospy.Publisher("/ball_found", Bool, queue_size=1)
 
    rospy.spin()

if __name__=="__main__":
    main()