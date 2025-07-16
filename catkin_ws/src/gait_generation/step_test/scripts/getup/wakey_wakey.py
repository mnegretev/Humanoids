#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, String
from ctrl_msgs.srv import Getup
from darwin_gait.srv import WalkGains, WalkGainsRequest
import numpy as np
import time


def main():

    global humanoid_pose
    rospy.init_node("Darwin_lock_in")
    #rospy.Subscriber("/imu/state", String, callback_imu)
    rospy.wait_for_service("/getup")
    rospy.wait_for_service("/walk_engine")
    get_up_service = rospy.ServiceProxy("/getup", Getup)
    walk_service = rospy.ServiceProxy("/walk_engine", WalkGains)
    humanoid_pose = "NOT_POSE"

    req = WalkGainsRequest()
    req.enabled_gain = 1.0
    req.time = 1.0

    while not rospy.is_shutdown():
        print("Entered loop")

        humanoid_pose = rospy.wait_for_message("/imu/state", String, timeout=5).data


        if humanoid_pose != "straight": 

            if humanoid_pose == "fall_front":
                get_up_service("front")
            elif humanoid_pose == "fall_back":
                get_up_service("back")
            else:
                print("Literally how??????????")

            print("Aight its up")
            
            rospy.sleep(3)
        
        walk_service(req)
        print("Walking !!!")

if __name__ == '__main__':
    main()

            

