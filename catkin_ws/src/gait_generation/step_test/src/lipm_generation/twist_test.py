#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
#subscribe to a boolean topic
#make launch with global parameter

def main():
    rospy.init_node("step_test_node")
    pub_leg_left_goal_pose = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_leg_right_goal_pose = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray , queue_size=1)
    start_pose_file = rospy.get_param("~twist_start_pose")
    start_pose = np.load(start_pose_file)
    timstep = start_pose["timestep"]
    rate = rospy.Rate(int(1/(timstep*5)))
    for right, left in zip(start_pose["right"], start_pose["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        rate.sleep()

    left_first_step_pose_file = rospy.get_param("~twist_left_first_step")
    left_first_step = np.load(left_first_step_pose_file)
    timstep = left_first_step["timestep"]
    rate = rospy.Rate(int(1/(timstep*3)))
    for right, left in zip(left_first_step["right"], left_first_step["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        rate.sleep()

if __name__ == '__main__':
    exit(main())