#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import os
import re
import glob
from ctrl_msgs.srv import Getup, GetupResponse
def getnum(file):
    num=int(re.findall(r'\d+', file)[0])
    return num
def read_poses(trajectory_dir):
    global arms, legs
    arms=[]
    legs=[]
    files=[]
    files = sorted(glob.glob(os.path.join(trajectory_dir, '*.npz')), key=getnum)
    for traj in files:
        print(traj)
        if "arms" in traj:
            arms.append(np.load(traj))
        else:
            legs.append(np.load(traj))   

def handle(req):
    try:
        read_poses(req.path)
        timestep = legs[0]["timestep"]
        rate = rospy.Rate(int(1/(timestep*0.5)))

        for i in range(len(arms)):
            for right_leg, left_leg, right_arm, left_arm in zip(legs[i]["right_leg"], legs[i]["left_leg"],arms[i]["right_arm"], arms[i]["left_arm"]):     
                if i ==3:
                    rate = rospy.Rate(int(1/(timestep*1.5)))
                else:
                    rate = rospy.Rate(int(1/(timestep*0.5)))       
                right_leg_goal_pose = Float32MultiArray()
                right_leg_goal_pose.data = right_leg
                pub_leg_right_goal_pose.publish(right_leg_goal_pose)

                left_leg_goal_pose = Float32MultiArray()
                left_leg_goal_pose.data = left_leg
                pub_leg_left_goal_pose.publish(left_leg_goal_pose)
                rate.sleep()      

                right_arm_goal_pose = Float32MultiArray()
                right_arm_goal_pose.data = right_arm
                pub_arm_right_goal_pose.publish(right_arm_goal_pose)

                left_arm_goal_pose = Float32MultiArray()
                left_arm_goal_pose.data = left_arm
                pub_arm_left_goal_pose.publish(left_arm_goal_pose)
                rate.sleep()   
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
    print("Iniciando servicio")
    service = rospy.Service('getup', Getup, handle)
    rospy.spin()
if __name__ == '__main__':
    main()
