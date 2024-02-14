#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *

def callback_ball_position(msg):
    global pub_head_goal
    print("Position received")
    print(msg)
    error_x = msg.x - 640
    error_y = msg.y - 360
    goal_pan = -1/1000.0 * error_x
    goal_tilt = 1/700.0 * error_y

    if goal_pan > 1.5:
        goal_pan = 1.5
    
    if goal_pan < -1.5:
        goal_pan = -1.5

    if goal_tilt > 1:
        goal_tilt = 1
    
    if goal_tilt < -1:
        goal_tilt = -1
    

    msg_goal = Float32MultiArray()
    msg_goal.data = [goal_pan, goal_tilt]
    print(msg_goal.data)
    pub_head_goal.publish(msg_goal)

def main():
    global pub_head_goal
    print("Initializing ball tracker by Mike")
    rospy.init_node("ball_tracker")

    rospy.Subscriber("/vision/ball_img_position", Point, callback_ball_position)
    pub_head_goal = rospy.Publisher("/hardware/head_goal_pose", Float32MultiArray, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    exit(main())