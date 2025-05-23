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

    start_pose_legs = np.load("/home/k1000/Humanoids/catkin_ws/src/gait_generation/step_test/src/getup/poses/legs_pose2.npz")
    start_pose_arms = np.load("/home/k1000/Humanoids/catkin_ws/src/gait_generation/step_test/src/getup/poses/arms_pose2.npz")

    start_pose_legs2 = np.load("/home/k1000/Humanoids/catkin_ws/src/gait_generation/step_test/src/getup/poses/legs_pose3.npz")
    start_pose_arms2 = np.load("/home/k1000/Humanoids/catkin_ws/src/gait_generation/step_test/src/getup/poses/arms_pose3.npz")

    start_pose_legs3 = np.load("/home/k1000/Humanoids/catkin_ws/src/gait_generation/step_test/src/getup/poses/legs_pose4.npz")
    start_pose_arms3 = np.load("/home/k1000/Humanoids/catkin_ws/src/gait_generation/step_test/src/getup/poses/arms_pose4.npz")

    start_pose_legs4 = np.load("/home/k1000/Humanoids/catkin_ws/src/gait_generation/step_test/src/getup/poses/legs_pose4.npz")
    start_pose_arms4 = np.load("/home/k1000/Humanoids/catkin_ws/src/gait_generation/step_test/src/getup/poses/arms_pose4.npz")
    timstep = start_pose_legs["timestep"]
    rate = rospy.Rate(int(1/(timstep*1.5)))
    rate_pose = rospy.Rate(int(1/(timstep/2)))
    fast_pose = rospy.Rate(int(1/(timstep/3)))
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
        rate_pose.sleep()
        print(right_arm)
        print(left_arm)
        right_arm_goal_pose = Float32MultiArray()
        right_arm_goal_pose.data = right_arm
        pub_arm_right_goal_pose.publish(right_arm_goal_pose)

        left_arm_goal_pose = Float32MultiArray()
        left_arm_goal_pose.data = left_arm
        pub_arm_left_goal_pose.publish(left_arm_goal_pose)
        rate.sleep()
    #------------------pose 3--------------------------------
    for right_leg, left_leg,right_arm, left_arm in zip(start_pose_legs3["right_leg"],start_pose_legs3["left_leg"],start_pose_arms3["right_arm"],start_pose_arms3["left_arm"]):
        print(right_leg)
        print(left_leg)
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right_leg
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left_leg
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
    
        rate_pose.sleep()
        print(right_arm)
        print(left_arm)
        right_arm_goal_pose = Float32MultiArray()
        right_arm_goal_pose.data = right_arm
        pub_arm_right_goal_pose.publish(right_arm_goal_pose)

        left_arm_goal_pose = Float32MultiArray()
        left_arm_goal_pose.data = left_arm
        pub_arm_left_goal_pose.publish(left_arm_goal_pose)
        rate.sleep()
    #------------------pose 4--------------------------------
    for right_leg, left_leg,right_arm, left_arm in zip(start_pose_legs4["right_leg"],start_pose_legs4["left_leg"],start_pose_arms4["right_arm"],start_pose_arms4["left_arm"]):
        print(right_leg)
        print(left_leg)
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right_leg
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left_leg
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
    
        print(right_arm)
        print(left_arm)
        right_arm_goal_pose = Float32MultiArray()
        right_arm_goal_pose.data = right_arm
        pub_arm_right_goal_pose.publish(right_arm_goal_pose)

        left_arm_goal_pose = Float32MultiArray()
        left_arm_goal_pose.data = left_arm
        pub_arm_left_goal_pose.publish(left_arm_goal_pose)
   
        rate.sleep()
    return    
if __name__ == '__main__':
    main()