#!/usr/bin/env python
import rospy
import numpy as np
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
import time
pan_robot=0.0
tilt_robot=0.0
searching = True

def head_k_generator(centroid_msg):
	global pan_robot, tilt_robot,searching
	searching = False
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

def tracker_ball():
	start_pose1_file = rospy.get_param("~head_point1")
	start_pose2_file = rospy.get_param("~head_point2")
	start_pose3_file = rospy.get_param("~head_point3")
	start_pose4_file = rospy.get_param("~head_point4")
	start_pose1 = np.load(start_pose1_file)
	start_pose2 = np.load(start_pose2_file)
	start_pose3 = np.load(start_pose3_file)
	start_pose4 = np.load(start_pose4_file)


	timstep = start_pose1["timestep"]
	rate = rospy.Rate(int(0.3/(timstep)))
	if searching :
		for head in zip(start_pose1["head"]):     
			print(head)
			cmd_head = Float32MultiArray()
			cmd_head.data = head[0]
			pub_head_goal.publish(cmd_head)
			rate.sleep()    

		for head in zip(start_pose2["head"]):     
			print(head)
			cmd_head = Float32MultiArray()
			cmd_head.data = head[0]
			pub_head_goal.publish(cmd_head)
			rate.sleep()
		for head in zip(start_pose3["head"]):     
			print(head)
			cmd_head = Float32MultiArray()
			cmd_head.data = head[0]
			pub_head_goal.publish(cmd_head)
			rate.sleep()
		for head in zip(start_pose4["head"]):     
			print(head)
			cmd_head = Float32MultiArray()
			cmd_head.data = head[0]
			pub_head_goal.publish(cmd_head)
			rate.sleep()

def main ():
	global pub_head_goal,rate, head_move_pub, searching
	rospy.init_node("head_goal_ball_position_node")  
	pub_head_goal = rospy.Publisher("/hardware/head_goal_pose", Float32MultiArray, queue_size=1)      
	rospy.Subscriber("/centroid_publisher", Point32, head_k_generator)  
	while not rospy.is_shutdown():
		tracker_ball()
	
		rospy.sleep(1)
	rospy.spin()

if __name__=="__main__":
    main()
