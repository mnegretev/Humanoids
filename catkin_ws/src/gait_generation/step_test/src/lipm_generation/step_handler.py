#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import time

class WalkingController():
    def __init__(self):
        self.sub = rospy.Subscriber("/walk_enable", String, self.callback, queue_size=1)
        self.pub_leg_left_goal_pose = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size=1)
        self.pub_leg_right_goal_pose = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray , queue_size=1)
        self.is_walking = False
    
    def callback(self, msg):
        command = msg.data
        if command == "forward":
            if not self.is_walking:
                self.is_walking=True
                self.execute_npz_file("~start_pose")
                self.execute_npz_file("~left_first_halfstep")
            self.execute_npz_file("~right_full_step")
            self.execute_npz_file("~left_full_step")
            
        else:
            print("Could not process command")
            
    def execute_npz_file(self, param: str):
        trajectory_file = rospy.get_param(param)
        trajectory = np.load(trajectory_file)
        print(f"Executing {trajectory_file}")
        timestep = trajectory["timestep"]
        rate = rospy.Rate(int(1/(timestep)))
        for right, left in zip(trajectory["right"], trajectory["left"]):
            #Publish right leg
            right_leg_goal_pose = Float32MultiArray()
            right_leg_goal_pose.data = right
            self.pub_leg_right_goal_pose.publish(right_leg_goal_pose)
            #Publish left leg
            left_leg_goal_pose = Float32MultiArray()
            left_leg_goal_pose.data = left
            self.pub_leg_left_goal_pose.publish(left_leg_goal_pose)
            rate.sleep()
        return True

def main():
    rospy.init_node("step_test_node")
    step_controller = WalkingController()
    rospy.spin()

if __name__ == '__main__':
    exit(main())