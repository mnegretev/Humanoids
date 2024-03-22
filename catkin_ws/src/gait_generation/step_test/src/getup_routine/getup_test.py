#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

def main():
    print("Starting get up routine...")
    rospy.init_node("getup_test_node")
    pub_leg_left_goal_pose = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_leg_right_goal_pose = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray , queue_size=1)
    pub_arm_left_goal_pose = rospy.Publisher("/hardware/arm_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_arm_right_goal_pose = rospy.Publisher("/hardware/arm_right_goal_pose", Float32MultiArray , queue_size=1)

    start_pose_legs = np.load("legs_start_pose1.npz")
    start_pose_arms = np.load("arms_start_pose1.npz")
    start_pose_arms2 = np.load("arms_start_pose2.npz")
    start_pose_legs2 = np.load("legs_start_pose2.npz")

    timstep = start_pose_legs["timestep"]
    rate = rospy.Rate(int(1/(timstep*3)))
    #--------------------------pose 1-------------------------------------------------
    for right_leg, left_leg, right_arm, left_arm in zip(start_pose_legs["right_leg"], start_pose_legs["left_leg"],start_pose_arms["right_arm"], start_pose_arms["left_arm"]):
        print(right_leg)
        print(left_leg)
      
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right_leg
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left_leg
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
    
        rate.sleep()
        print(right_arm)
        print(left_arm)
        right_arm_goal_pose = Float32MultiArray()
        right_arm_goal_pose.data = right_arm
        pub_arm_right_goal_pose.publish(right_arm_goal_pose)

        left_arm_goal_pose = Float32MultiArray()
        left_arm_goal_pose.data = left_arm
        pub_arm_left_goal_pose.publish(left_arm_goal_pose)
        rate.sleep()      
    #------------------pose 2--------------------------------
    for right_leg, left_leg,right_arm, left_arm in zip(start_pose_legs2["right_leg"],start_pose_legs2["left_leg"],start_pose_arms2["right_arm"],start_pose_arms2["left_arm"]):
        print(right_leg)
        print(left_leg)
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right_leg
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left_leg
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
    
        rate.sleep()
        print(right_arm)
        print(left_arm)
        right_arm_goal_pose = Float32MultiArray()
        right_arm_goal_pose.data = right_arm
        pub_arm_right_goal_pose.publish(right_arm_goal_pose)

        left_arm_goal_pose = Float32MultiArray()
        left_arm_goal_pose.data = left_arm
        pub_arm_left_goal_pose.publish(left_arm_goal_pose)
    # for right_arm, left_arm in zip(start_pose_arms2["right_arm"], start_pose_arms2["left_arm"]):
    #     print(right_arm)
    #     print(left_arm)
    #     right_arm_goal_pose = Float32MultiArray()
    #     right_arm_goal_pose.data = right_arm
    #     pub_arm_right_goal_pose.publish(right_arm_goal_pose)

    #     left_arm_goal_pose = Float32MultiArray()
    #     left_arm_goal_pose.data = left_arm
    #     pub_arm_left_goal_pose.publish(left_arm_goal_pose)
        
        rate.sleep()
    # # time.sleep(2)
    
    return    
if __name__ == '__main__':
    main()