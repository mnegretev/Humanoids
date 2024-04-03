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
    start_pose_file = rospy.get_param("~start_pose")
    start_pose = np.load(start_pose_file)
    timstep = start_pose["timestep"]
    rate = rospy.Rate(int(1/(timstep)))
    middle_rate = rospy.Rate(int(1/(timstep)))
    fast_rate = rospy.Rate(int(1/(timstep/3)))
    for right, left in zip(start_pose["right"], start_pose["left"]):
        print(right)
        print(left)
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        rate.sleep()
    
    time.sleep(2)
    first_half_step_file = rospy.get_param("~left_first_halfstep")
    first_half_step = np.load(first_half_step_file)
    for right, left in zip(first_half_step["right"], first_half_step["left"]):
        print(right)
        print(left)
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        fast_rate.sleep()
    time.sleep(2)
    
    while not rospy.is_shutdown():

        right_full_step_file = rospy.get_param("~right_full_step")
        second_step = np.load(right_full_step_file)
        for right, left in zip(second_step["right"], second_step["left"]):
            print(right)
            print(left)
            right_leg_goal_pose = Float32MultiArray()
            right_leg_goal_pose.data = right
            pub_leg_right_goal_pose.publish(right_leg_goal_pose)

            left_leg_goal_pose = Float32MultiArray()
            left_leg_goal_pose.data = left
            pub_leg_left_goal_pose.publish(left_leg_goal_pose)
            middle_rate.sleep()

        # time.sleep(2)
        left_full_step_file = rospy.get_param("~left_full_step")
        third_step = np.load(left_full_step_file)
        for right, left in zip(third_step["right"], third_step["left"]):
            print(right)
            print(left)
            right_leg_goal_pose = Float32MultiArray()
            right_leg_goal_pose.data = right
            pub_leg_right_goal_pose.publish(right_leg_goal_pose)

            left_leg_goal_pose = Float32MultiArray()
            left_leg_goal_pose.data = left
            pub_leg_left_goal_pose.publish(left_leg_goal_pose)
            middle_rate.sleep()
    

if __name__ == '__main__':
    exit(main())