#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import os
import re
import glob
from ctrl_msgs.srv import Getup, GetupResponse

def get_linear_traj(q0,q1,step):
    deltas = [abs(q0[i]-q1[i]) for i in range(len(q0))]
    n=int(np.max(deltas)/step)+1
    Q=np.zeros((n,len(q0)))
    for i in range(len(q0)):
        Q[:,i] = np.linspace(q0[i],q1[i],n)
    return Q


def read_poses(dir_name):
    pose_array = []
    f = open(dir_name, "r")
    a = (f.read()).split("\n")

    for s in a[:-1]:
        pose = np.array([float(x) for x in s.strip('[]').split(',')])
        pose_array.append(pose)

    return pose_array

def handle(req):
    if req.path == "back":
        leg_left_waypoints  = back_leg_left_waypoints 
        leg_right_waypoints = back_leg_right_waypoints
        arm_left_waypoints  = back_arm_left_waypoints 
        arm_right_waypoints = back_arm_right_waypoints
    elif req.path == "front":
        leg_left_waypoints  = front_leg_left_waypoints 
        leg_right_waypoints = front_leg_right_waypoints
        arm_left_waypoints  = front_arm_left_waypoints 
        arm_right_waypoints = front_arm_right_waypoints
    else:
        return "ERROR NOT a valid argument"


    step = 0.1
    try:

        rate = rospy.Rate(40)
        max_k = np.max([len(leg_left_waypoints), len(leg_right_waypoints), len(arm_left_waypoints), len(arm_right_waypoints) ])
        for k in range(1,max_k):
            i = min(k,len(leg_left_waypoints)-1)
            leg_left_Q = get_linear_traj(leg_left_waypoints[i-1],leg_left_waypoints[i], step )
            i = min(k,len(leg_right_waypoints)-1)
            leg_right_Q = get_linear_traj(leg_right_waypoints[i-1],leg_right_waypoints[i], step )
            i = min(k,len(arm_left_waypoints)-1)
            arm_left_Q = get_linear_traj(arm_left_waypoints[i-1],arm_left_waypoints[i], step )
            i = min(k,len(arm_right_waypoints)-1)
            arm_right_Q = get_linear_traj(arm_right_waypoints[i-1],arm_right_waypoints[i], step )

            max_j = np.max([len(leg_left_Q), len(leg_right_Q), len(arm_left_Q), len(arm_right_Q) ])

            for j in range(max_j):
                head_goal_pose = Float32MultiArray()
                head_goal_pose.data = [0,0]
                pub_head_goal_pose.publish(head_goal_pose)

                left_leg_goal_pose = Float32MultiArray()
                left_leg_goal_pose.data = leg_left_Q[min(j,len(leg_left_Q)-1)]
                pub_leg_left_goal_pose.publish(left_leg_goal_pose)

                right_leg_goal_pose = Float32MultiArray()
                right_leg_goal_pose.data = leg_right_Q[min(j,len(leg_right_Q)-1)]
                pub_leg_right_goal_pose.publish(right_leg_goal_pose)

                left_arm_goal_pose = Float32MultiArray()
                left_arm_goal_pose.data = arm_left_Q[min(j,len(arm_left_Q)-1)]
                pub_arm_left_goal_pose.publish(left_arm_goal_pose)

                right_arm_goal_pose = Float32MultiArray()
                right_arm_goal_pose.data = arm_right_Q[min(j,len(arm_right_Q)-1)]
                pub_arm_right_goal_pose.publish(right_arm_goal_pose)
                rate.sleep()

        resp=GetupResponse()
        resp.succes = True
    except Exception as e:
        print(e)
        resp=GetupResponse()
        resp.succes = False
    return resp   

def main():
    #publishers
    global pub_arm_left_goal_pose, pub_arm_right_goal_pose, pub_leg_left_goal_pose, pub_leg_right_goal_pose, pub_head_goal_pose
    #files
    global front_arm_right_waypoints, front_arm_left_waypoints, front_leg_right_waypoints, front_leg_left_waypoints, back_arm_right_waypoints, back_arm_left_waypoints, back_leg_right_waypoints, back_leg_left_waypoints  

    pub_head_goal_pose = rospy.Publisher("/hardware/head_goal_pose", Float32MultiArray, queue_size=1)
    pub_leg_left_goal_pose = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_leg_right_goal_pose = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray , queue_size=1)
    pub_arm_left_goal_pose = rospy.Publisher("/hardware/arm_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_arm_right_goal_pose = rospy.Publisher("/hardware/arm_right_goal_pose", Float32MultiArray , queue_size=1)

    rospy.init_node("getup_server")
    path = rospy.get_param("~path", "/home/k1000/Humanoids/catkin_ws/src/gait_generation/step_test/src/getup/poses")

    front_arm_right_waypoints = read_poses(path+"/front/r_arm.txt")
    front_arm_left_waypoints  = read_poses(path+"/front/l_arm.txt")
    front_leg_right_waypoints = read_poses(path+"/front/r_leg.txt")
    front_leg_left_waypoints  = read_poses(path+"/front/l_leg.txt")

    back_arm_right_waypoints = read_poses(path+"/back/r_arm.txt")
    back_arm_left_waypoints  = read_poses(path+"/back/l_arm.txt")
    back_leg_right_waypoints = read_poses(path+"/back/r_leg.txt")
    back_leg_left_waypoints  = read_poses(path+"/back/l_leg.txt")

    print("Iniciando servicio")
    service = rospy.Service('getup', Getup, handle)
    rospy.spin()
if __name__ == '__main__':
    main()
