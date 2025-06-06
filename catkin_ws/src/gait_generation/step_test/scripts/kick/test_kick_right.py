#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
#subscribe to a boolean topic
#make launch with global parameter

def main():
    rospy.init_node("test_kick_node")
    pub_leg_left_goal_pose = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_leg_right_goal_pose = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray , queue_size=1)
    pub_arms_goal = rospy.Publisher("/hardware/arms_goal_pose", Float32MultiArray, queue_size=1)
    #start_pose_file = rospy.get_param("~standup")
    #start_pose = np.load(start_pose_file)
    #timstep = start_pose["timestep"]
    #rate = rospy.Rate(int(1/(timstep*5)))
    #for right, left in zip(start_pose["right"], start_pose["left"]):
    #    right_leg_goal_pose = Float32MultiArray()
    #    right_leg_goal_pose.data = right
    #    pub_leg_right_goal_pose.publish(right_leg_goal_pose)

    #    left_leg_goal_pose = Float32MultiArray()
    #    left_leg_goal_pose.data = left
    #    pub_leg_left_goal_pose.publish(left_leg_goal_pose)
    #    rate.sleep()
    time.sleep(4)
    zero_pose = Float32MultiArray()
    zero_pose.data = [0.0, 0.3, 0.0, 0.0, -0.3, 0.0]
    pub_arms_goal.publish(zero_pose)
    zero_pose.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pub_leg_right_goal_pose.publish(zero_pose)
    pub_leg_left_goal_pose.publish(zero_pose)
    time.sleep(4)

    start_pose_file = rospy.get_param("~kick_right_start_pose")
    start_pose = np.load(start_pose_file)
    timstep = start_pose["timestep"]
    rate = rospy.Rate(int(1/(timstep*3)))
    for right, left in zip(start_pose["right"], start_pose["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        rate.sleep()

    left_first_step_pose_file = rospy.get_param("~kick_right_raise_foot")
    left_first_step = np.load(left_first_step_pose_file)
    timstep = left_first_step["timestep"]
    rate = rospy.Rate(int(1/(timstep*2)))
    for right, left in zip(left_first_step["right"], left_first_step["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        rate.sleep()

    twist_move_com_left_file = rospy.get_param("~kick_right_do_kick")
    twist_move_com_left = np.load(twist_move_com_left_file)
    timstep = twist_move_com_left["timestep"]
    rate = rospy.Rate(int(1/(timstep))*8)
    for right, left in zip(twist_move_com_left["right"], twist_move_com_left["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        rate.sleep()
    
    #twist_right_third_step_file = rospy.get_param("~kick_right_final_stop")
    #twist_right_third_step = np.load(twist_right_third_step_file)
    #timstep = twist_right_third_step["timestep"]
    #rate = rospy.Rate(int(1/(timstep))*3)
    #for right, left in zip(twist_right_third_step["right"], twist_right_third_step["left"]):
    #    right_leg_goal_pose = Float32MultiArray()
    #    right_leg_goal_pose.data = right
    #    pub_leg_right_goal_pose.publish(right_leg_goal_pose)

    #    left_leg_goal_pose = Float32MultiArray()
    #    left_leg_goal_pose.data = left
    #    pub_leg_left_goal_pose.publish(left_leg_goal_pose)
    #    rate.sleep()

if __name__ == '__main__':
    exit(main())
