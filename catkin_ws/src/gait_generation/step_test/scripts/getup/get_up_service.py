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


def getnum(file):
    num=int(re.findall(r'\d+', file)[0])
    return num
def read_poses(trajectory_dir):
    global arms, legs
    arms=[]
    legs=[]
    files=[]
    files = sorted(glob.glob(os.path.join(trajectory_dir, '*.npz')), key=getnum)
    for traj in files[0:2]:
        if "arms" in traj:
            arms.append(np.load(traj))
        else:
            legs.append(np.load(traj))   

def handle(req):
    leg_left_waypoints = [
        [0.0,0.0,0.0,0.0,0.0,0.0],
        [0.0,0.0,0.0,0.0,0.0,0.0],
        [0.0,0.0,-2.6,2.2,-0.8,0.0],
        [0.0,0.0,-2.6,2.2,-1.4,0.0],
        [0.0,0.0,-2.3,2.1,-0.8,0.0],
        [0.0,0.0,-1.8,1.7,-0.6,0.0],
        [0.0,0.0,-1.7,1.7,-0.6,0.0],
        [0.0,0.0,-1.6,1.7,-0.6,0.0],
        [0.0,0.0,-1.5,1.7,-0.6,0.0],
        [0.0,0.0,-1.4,1.5,-0.6,0.0],
        [0.0,0.0,0.0,0.0,0.0,0.0],
    ]
    leg_right_waypoints = [
        [0.0,0.0,0.0,0.0,0.0,0.0],
        [0.0,0.0,0.0,0.0,0.0,0.0],
        [0.0,0.0,-2.6,2.2,-0.8,0.0],
        [0.0,0.0,-2.6,2.2,-1.4,0.0],
        [0.0,0.0,-2.3,2.1,-0.8,0.0],
        [0.0,0.0,-1.8,1.7,-0.6,0.0],
        [0.0,0.0,-1.7,1.7,-0.6,0.0],
        [0.0,0.0,-1.6,1.7,-0.6,0.0],
        [0.0,0.0,-1.5,1.7,-0.6,0.0],
        [0.0,0.0,-1.4,1.5,-0.6,0.0],
        [0.0,0.0,0.0,0.0,0.0,0.0],
    ]
    arm_left_waypoints = [
        [0.0,0.0,0.0],
        [1.6,0.0,-2.7],
        [-2.0,0.0,-1.4],
        [-0.8,0.0,-0.3],
        [-0.0,0.0,-0.3],
        [-0.0,0.0,-0.3],
        [-0.0,0.0,-0.3],
        [-0.0,0.0,-0.3],
        [-0.0,0.0,-0.3],
        [-0.0,0.0,-0.3],
        [0.0,0.0,0.0],
    ]
    arm_right_waypoints = [
        [0.0,0.0,0.0],
        [1.6,0.0,-2.7],
        [-2.0,0.0,-1.4],
        [-0.8,0.0,-0.3],
        [-0.0,0.0,-0.3],
        [-0.0,0.0,-0.3],
        [-0.0,0.0,-0.3],
        [-0.0,0.0,-0.3],
        [-0.0,0.0,-0.3],
        [-0.0,0.0,-0.3],
        [0.0,0.0,0.0],
    ]
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
    global pub_arm_left_goal_pose, pub_arm_right_goal_pose, pub_leg_left_goal_pose, pub_leg_right_goal_pose, pub_head_goal_pose
    pub_head_goal_pose = rospy.Publisher("/hardware/head_goal_pose", Float32MultiArray, queue_size=1)
    pub_leg_left_goal_pose = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_leg_right_goal_pose = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray , queue_size=1)
    pub_arm_left_goal_pose = rospy.Publisher("/hardware/arm_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_arm_right_goal_pose = rospy.Publisher("/hardware/arm_right_goal_pose", Float32MultiArray , queue_size=1)
    rospy.init_node("getup_server")
    path = rospy.get_param("~path", "/home/k1000/Humanoids/catkin_ws/src/gait_generation/step_test/src/getup/poses")
    read_poses(path)
    print("Iniciando servicio")
    service = rospy.Service('getup', Getup, handle)
    rospy.spin()
if __name__ == '__main__':
    main()
