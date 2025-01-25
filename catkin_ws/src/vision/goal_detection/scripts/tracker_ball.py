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

pan_robot=0.0
tilt_robot=0.0
buffer = 5 #Measurements to be  averaged

def head_k_generator(centroid_msg):
	global pan_robot, tilt_robot
	center_x = centroid_msg.x
	center_y = centroid_msg.y
	print(f"Centroid received {center_x}, {center_y}")
	head_cmd = Float32MultiArray()
	Cx = 320
	Cy = 240
	ex = center_x-Cx
	ey = center_y-Cy
	Kpan = 0.2/320
	Ktilt = 0.1/240
	dead_zone_x = 5
	dead_zone_y= 5


	pan = -Kpan * ex
	tilt = Ktilt * ey

	pan_robot  += pan
	tilt_robot += tilt

	head_cmd.data = [pan_robot, tilt_robot]
	print(f"Publishing pan_angle: {pan_robot}, tilt: {tilt_robot}")
	pub_head_goal.publish(head_cmd)

	

def main ():
	global pub_head_goal,rate
	rospy.init_node("head_goal_ball_position_node")  
	rospy.Subscriber("/centroid_publisher", Point32, head_k_generator)
	pub_head_goal = rospy.Publisher("/hardware/head_goal_pose", Float32MultiArray, queue_size=1)   
	rospy.spin()

if __name__=="__main__":
    main()