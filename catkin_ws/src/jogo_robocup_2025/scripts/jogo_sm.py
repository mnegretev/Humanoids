#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Float32MultiArray, String
from ctrl_msgs.srv import Getup
import numpy as np
import time

SM_INIT = 0
SM_CHECK_INITIAL_POSTURE = 1
SM_INITIAL_MOVEMENT = 2
SM_WAIT_FOR_INITIAL_MOVEMENT = 3
SM_WAIT_FOR_KICKOFF = 4

HUMANOID_STANDING = "straight"
HUMANOID_FALL_BACK = "fall_back"
HUMANOID_FALL_FRONT = "fall_front"

def callback_imu(msg):
    global humanoid_posture
    print(msg.data)
    humanoid_posture = msg.data

def call_getup_service(humanoid_pos):
    rospy.wait_for_service("\getup")

    if humanoid_pose == HUMANOID_STANDING:
        result = getup("straight")
    elif humanoid_pose == HUMANOID_FALL_BACK:
        result = getup("back")
    elif humanoid_pose == HUMANOID_FALL_FRONT:
        result = getup("front")
    return result

def is_in_zero_position():
    tol = 0.1
    try:
        print("In in zero?")
        position = rospy.wait_for_message("/hardware/joint_current_angles", Float32MultiArray, timeout=10)
        print(np.max(np.abs(position.data)) < tol)
        return np.max(np.abs(position.data)) < tol
    except:
        return False

def main():
    global clt_geutp, humanoid_posture
    print("initialazing main state machine")
    rospy.init_node("jogo_bonito")

    rospy.Subscriber("/imu/state", String, callback_imu)

    clt_geutp = rospy.ServiceProxy('/getup', Getup)

    loop = rospy.Rate(10)
    
    state = SM_INIT
    timer_counter = 0

    humanoid_posture = ""
    while not rospy.is_shutdown():

        if state == SM_INIT:
            print("Initialazing state machine")
            state = SM_CHECK_INITIAL_POSTURE
        elif state == SM_CHECK_INITIAL_POSTURE:
            if is_in_zero_position() and (humanoid_posture == HUMANOID_STANDING):
                state = SM_WAIT_FOR_KICKOFF
            else:
                print("Current main position: ", humanoid_posture)
                state = SM_INITIAL_MOVEMENT
        elif state == SM_INITIAL_MOVEMENT:
            if humanoid_posture == HUMANOID_STANDING:
                print("calling service for ready position")
            elif humanoid_posture == HUMANOID_FALL_BACK:
                print("calling getup from back")
                
            elif humanoid_posture == HUMANOID_FALL_FRONT:
                print("calling getup from front")
            
            call_getup_service(humanoid_posture)

        elif state == SM_WAIT_FOR_KICKOFF:
            print("Ready for kickoff")




        loop.sleep()

if __name__ == "__main__":
    main()