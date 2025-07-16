#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, String
from ctrl_msgs.srv import Getup
from darwin_gait.srv import WalkGains, WalkGainsRequest
import numpy as np
import time

def callback_imu(msg):
    global humanoid_pose
    print(msg.data)
    humanoid_pose = msg.data

def main():

    global humanoid_pose
    rospy.init_node("Darwin_lock_in")
    rospy.Subscriber("/imu/state", String, callback_imu)
    rospy.wait_for_service("/getup")
    rospy.wait_for_service("/walk_engine")
    get_up_service = rospy.ServiceProxy("/getup", Getup)
    walk_service = rospy.ServiceProxy("/walk_engine", WalkGains)
    humanoid_pose = "NOT_POSE"

    req = WalkGainsRequest()
    req.enabled_gain = 1.0
    req.time = 1.0

    while not rospy.is_shutdown:


        while humanoid_pose != "straight": 

            if humanoid_pose == "fall_from_front":
                get_up_service("front")
            elif humanoid_pose == "fall_from_back":
                get_up_service("back")
            else:
                print("Literally how??????????")

            print("Aight its up")
            
            rospy.sleep(1)
        
        walk_service(req)
        print("Walking !!!")

            

