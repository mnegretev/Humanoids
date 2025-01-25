#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time


def searching_trajectory(head_move)


def main():
    print("Starting get up routine...")
    rospy.init_node("search_trajectory_test")
    pub_head_goal = rospy.Publisher("/hardware/head_goal_pose", Float32MultiArray, queue_size=1)   
    rospy.Subscriber("/head_ball_publisher", Point32, searching_trajectory)
    start_pose1 = np.load("head_point1.npz")
    start_pose2 = np.load("head_point2.npz")
    start_pose3 = np.load("head_point3.npz")
    start_pose4 = np.load("head_point4.npz")
    timstep = start_pose1["timestep"]
    rate = rospy.Rate(int(0.3/(timstep)))
    
    for head in zip(start_pose1["head"]):     
            print(head)
            cmd_head = Float32MultiArray()
            cmd_head.data = head[0]
            pub_head_goal.publish(cmd_head)
            rate.sleep()     
        print ("First move finished")
        for head in zip(start_pose2["head"]):     
            print(head)
            cmd_head = Float32MultiArray()
            cmd_head.data = head[0]
            pub_head_goal.publish(cmd_head)
            rate.sleep()
        print ("Second move finished")
        for head in zip(start_pose3["head"]):     
            print(head)
            cmd_head = Float32MultiArray()
            cmd_head.data = head[0]
            pub_head_goal.publish(cmd_head)
            rate.sleep()
        print ("Third move finished")
        for head in zip(start_pose4["head"]):     
            print(head)
            cmd_head = Float32MultiArray()
            cmd_head.data = head[0]
            pub_head_goal.publish(cmd_head)
            rate.sleep()
    rospy.spin()

     
    

    return    
if __name__ == '__main__':
    main()