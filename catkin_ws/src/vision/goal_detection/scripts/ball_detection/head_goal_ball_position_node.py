#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32MultiArray

pan_robot=0.0
tilt_robot=0.0

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
	Kpan = 0.02/Cx
	Ktilt = 0.01/Cy

	pan = -Kpan * ex
	tilt = Ktilt * ey

	pan_robot  += pan
	tilt_robot += tilt

	head_cmd.data = [pan_robot, tilt_robot]
	print(f"Publishing pan_angle: {pan_robot}, tilt: {tilt_robot}")
	pub_head_goal.publish(head_cmd)

def main ():
	global pub_head_goal,rate, head_move_pub, searching
	rospy.init_node("head_goal_ball_position_node")  
	pub_head_goal = rospy.Publisher("/hardware/head_goal_pose", Float32MultiArray, queue_size=1)      
	rospy.Subscriber("/centroid_publisher", Point32, head_k_generator)  
	rospy.spin()

if __name__=="__main__":
    main()
