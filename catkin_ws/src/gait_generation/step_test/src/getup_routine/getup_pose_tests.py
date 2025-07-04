#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

def main():
    print("Starting get up routine...")
    rospy.init_node("getup_test_node")
    pub_leg_left_goal_pose = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_leg_right_goal_pose = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray , queue_size=1)
    pub_arm_left_goal_pose = rospy.Publisher("/hardware/arm_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_arm_right_goal_pose = rospy.Publisher("/hardware/arm_right_goal_pose", Float32MultiArray , queue_size=1)

    rate = rospy.Rate(10)
    
    right_leg_goal_pose = Float32MultiArray()
    right_leg_goal_pose.data = [1.00, 0.00, 0.00, 0.00, 0.00, 0.00]
    pub_leg_right_goal_pose.publish(right_leg_goal_pose)
    print("ok")
    left_leg_goal_pose = Float32MultiArray()
    left_leg_goal_pose.data = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pub_leg_left_goal_pose.publish(left_leg_goal_pose)

    right_arm_goal_pose = Float32MultiArray()
    right_arm_goal_pose.data = [0.0, 0.0, 0.0]
    pub_arm_right_goal_pose.publish(right_arm_goal_pose)

    left_arm_goal_pose = Float32MultiArray()
    left_arm_goal_pose.data = [0.0, 0.0, 0.0]
    pub_arm_left_goal_pose.publish(left_arm_goal_pose)
    rate.sleep()
    rospy.spin()

    return    
if __name__ == '__main__':
    main()