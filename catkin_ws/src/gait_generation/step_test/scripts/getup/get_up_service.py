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
        [0,0,0,0,0,0],
        [0,0,0,0,0,0],
        [0,0,0,0,0,0],
        [0,0,-0.8,0.8,0,0],
        [0,0,-1.5,1.4,-0.5,0],
        [0,0,-1.5,2.2,-1.4,0],
        [0,0,-0.5,2.2,-1.5,0],
        [0,0,-0.5,2.2,-1.5,0],
        [0,0,-0.5,2.1,-1.5,0],
        [0,0,0.1,1.9,-1.5,0],
        [0,0,-0.1,1.9,-1.5,0],
        [0,0,-0.1,1.9,-1.5,0],
    ]
    leg_right_waypoints = [
        [0,0,0,0,0,0],
        [0,0,0,0,0,0],
        [0,0,0,0,0,0],
        [0,0,-0.8,0.8,0,0],
        [0,0,-1.5,1.4,-0.5,0],
        [0,0,-1.5,2.2,-1.4,0],
        [0,0,-0.5,2.2,-1.5,0],
        [0,0,-0.5,2.2,-1.5,0],
        [0,0,-0.5,2.1,-1.5,0],
        [0,0,0.1,1.9,-1.5,0],
        [0,0,-0.1,1.9,-1.5,0],
        [0,0,-0.1,1.9,-1.5,0],
    ]
    arm_left_waypoints = [
        [0,0,0],
        [1,0,0],
        [1,0,-1.8],
        [1,0,-1.8],
        [0.4,0,-1.8],
        [0,0,-2.0],
        [-1.0,0,0.0],
        [1.5,0,-1.5],
        [1.7,0,-1.5],
        [1.7,0,-1.5],
        [1.7,0,-1.5],
    ]
    arm_right_waypoints = [
        [0,0,0],
        [1,0,0],
        [1,0,-1.8],
        [1,0,-1.8],
        [0.4,0,-1.8],
        [0,0,-2.0],
        [-1.0,0,0.0],
        [1.5,0,-1.5],
        [1.7,0,-1.5],
        [1.7,0,-1.5],
        [1.7,0,-1.5],
    ]
    #leg_left_Q = get_linear_traj([0,0,0,0,0,0], [0,0,-1,0,0,0], 0.05)
    step = 0.1
    try:

        # timestep = legs[0]["timestep"]
        rate = rospy.Rate(20)
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

        # for i in range(len(arms)):
        #     for right_leg, left_leg, right_arm, left_arm in zip(legs[i]["right_leg"], legs[i]["left_leg"],arms[i]["right_arm"], arms[i]["left_arm"]):     
        #         if i ==3:
        #             rate = rospy.Rate(int(1/(timestep*1.5)))
        #         else:
        #             rate = rospy.Rate(int(1/(timestep*0.5)))       
        #         right_leg_goal_pose = Float32MultiArray()
        #         right_leg_goal_pose.data = right_leg
        #         pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        #         left_leg_goal_pose = Float32MultiArray()
        #         left_leg_goal_pose.data = left_leg
        #         pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        #         rate.sleep()      

        #         right_arm_goal_pose = Float32MultiArray()
        #         right_arm_goal_pose.data = right_arm
        #         pub_arm_right_goal_pose.publish(right_arm_goal_pose)

        #         left_arm_goal_pose = Float32MultiArray()
        #         left_arm_goal_pose.data = left_arm
        #         pub_arm_left_goal_pose.publish(left_arm_goal_pose)
        #         rate.sleep()   
        resp=GetupResponse()
        resp.succes = True
    except Exception as e:
        print(e)
        resp=GetupResponse()
        resp.succes = False
    return resp   

def main():
    global pub_arm_left_goal_pose, pub_arm_right_goal_pose, pub_leg_left_goal_pose, pub_leg_right_goal_pose
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
    #get_linear_traj([0,0],[1,1.5],0.1)
