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

pan_robot=0
tilt_robot=0
buffer = 5 #Measurements to be  averaged
centroid_buffer = [] #Buffer to alocate earlier positions
avrg_x=0
avrg_y=0
def moving_average (center_x, center_y):
	global centroid_buffer, avrg_x, avrg_y
	if len(centroid_buffer) > buffer :
		centroid_buffer.pop(0) #Removes the lastest position
	centroid_buffer.append((center_x, center_y))
	print ("Buffer: {centroid_buffer}")
	if len(centroid_buffer) == buffer:
		avrg_x = sum([pos[0] for pos in centroid_buffer])/buffer
		avrg_y = sum([pos[1] for pos in centroid_buffer])/buffer
	return avrg_x, avrg_y

def head_k_generator(centroid_msg):
	global pan_robot, tilt_robot
	center_x = centroid_msg.x
	center_y = centroid_msg.y
	head_cmd = Float32MultiArray()
	Cx = 320
	Cy = 240
	ex = center_x-Cx
	ey = center_y-Cy
	Kpan = 0.2/320
	Ktilt = 0.1/240
	dead_zone_x = 5
	dead_zone_y= 5
	max_pan = 1.0
	max_tilt = 0.5


	if center_x != 0 and center_y != 0:
		if abs(ex) < dead_zone_x and abs(ey) < dead_zone_y:
			print ("Ball is centered:", [center_x, center_y])
			head_cmd.data = [0.0, 0.0]
		else:
			pan  = -Kpan * ex #Inverted corrections for movement to the ball
			tilt = -Ktilt * ey
			#Take the max range of movement to avoid abrupt movements
		#	pan =  max(min(pan, max_pan), -max_pan)
		#	tilt = max(min(tilt, max_tilt), -max_tilt)

			print (f"Error: ex={ex}, ey={ey}")
			print (f"Coordinates: pan={pan}, tilt={tilt}")
			pan_robot += (pan/2)
			tilt_robot += (tilt/2)
			avrg_x, avrg_y = moving_average(pan_robot, tilt_robot)
			print (f"avrg_x = {avrg_x}, avrg_y = {avrg_y}")

		if 0.2 < (abs(avrg_x)/abs(pan_robot)) < 2.5 and  0.2 < (abs(avrg_y)/abs(tilt_robot)) < 2.5  :
			print ("****************")
			head_cmd.data = [pan_robot , tilt_robot]
			print (f"Coordinates: pan_robot={pan_robot}, tilt_robot={tilt_robot}")
		if avrg_x < 20 or avrg_x > 620 or avrg_y <20 or avrg_y > 460:
			print ("Ball near edge")
			head_cmd.data = [0.0, 0.0]
		pub_head_goal.publish (head_cmd)

        #return pan,tilt

def main ():
	global pub_head_goal,rate
	rospy.init_node("head_goal_ball_position_node")  
	rospy.Subscriber("/centroid_publisher", Point32, head_k_generator)
	pub_head_goal = rospy.Publisher("/head_goal_pose", Float32MultiArray, queue_size=1)   
	rospy.spin()

if __name__=="__main__":
    main()
