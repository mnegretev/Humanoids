#!/usr/bin/env python
import rospy
import numpy
import yaml
import rospkg

from std_msgs.msg import *
from humanoid_msgs.srv import *

file_positions = ""

name = []
all_positions = []

def main():
    print("Starting get_predef_poses_server by Luis Nava")
    rospy.init_node('get_predef_poses_server', anonymous=True)    
    server = rospy.Service('/humanoid/get_predef_poses', predefPoses, get_predef_poses)

    rospy.spin()

def get_predef_poses(req):
    global file_positions, all_positions
    res = predefPosesResponse()
   
    if file_positions != req.pose_to_load:
        all_positions = []
        read_positions_from_disk(req, res)
        
        file_positions = req.pose_to_load

    allocate_positions(req, res)

    return res

def read_positions_from_disk(req, res):
    global name, all_positions, number_poses, pose_duration
    print "Open file ../"+ req.pose_to_load+ ".yaml"

    rospack = rospkg.RosPack()
    rospack.list()
    
    predef_poses = yaml.load(open(rospack.get_path('config_files') + '/predef_poses/'+req.pose_to_load+'.yaml', 'r'))
    name = predef_poses['motion'][0]['joints'].keys()

    pose_duration = []
    number_poses = len(predef_poses['motion'])        

    for pose in range(0, number_poses):
        position = []
        for i in range(0, 20):
             position.append(predef_poses['motion'][pose]['joints'][name[i]]['position'])

        all_positions.append(position)   
        pose_duration.append(predef_poses['motion'][pose]['duration'])

    res.delay = pose_duration[0] 
    res.number_poses = number_poses


def allocate_positions(req, res):
    global name, all_positions, number_poses, pose_duration

    joint_name          = [""]*20
    joint_goal_position = Float32MultiArray()
    joint_goal_position = numpy.zeros(20)

    for id in range(0, 20):
        #Legs position
        if name[id] == 'left_hip_yaw':
            joint_name[0] = name[id]
            joint_goal_position[0] = all_positions[req.robot_pose][id]
        if name[id] == 'left_hip_roll':
            joint_name[1] = name[id]
            joint_goal_position[1] = all_positions[req.robot_pose][id]
        if name[id] == 'left_hip_pitch':
            joint_name[2] = name[id]
            joint_goal_position[2] = all_positions[req.robot_pose][id]
        if name[id] == 'left_knee_pitch':
            joint_name[3] = name[id]
            joint_goal_position[3] = all_positions[req.robot_pose][id]
        if name[id] == 'left_ankle_pitch':
            joint_name[4] = name[id]
            joint_goal_position[4] = all_positions[req.robot_pose][id]
        if name[id] == 'left_ankle_roll':
            joint_name[5] = name[id]
            joint_goal_position[5] = all_positions[req.robot_pose][id]
        if name[id] == 'right_hip_yaw':
            joint_name[6] = name[id]
            joint_goal_position[6] = all_positions[req.robot_pose][id]
        if name[id] == 'right_hip_roll':
            joint_name[7] = name[id]
            joint_goal_position[7] = all_positions[req.robot_pose][id]
        if name[id] == 'right_hip_pitch':
            joint_name[8] = name[id]
            joint_goal_position[8] = all_positions[req.robot_pose][id]
        if name[id] == 'right_knee_pitch':
            joint_name[9] = name[id]
            joint_goal_position[9] = all_positions[req.robot_pose][id]
        if name[id] == 'right_ankle_pitch':
            joint_name[10] = name[id]
            joint_goal_position[10] = all_positions[req.robot_pose][id]
        if name[id] == 'right_ankle_roll':
            joint_name[11] = name[id]
            joint_goal_position[11] = all_positions[req.robot_pose][id]
        # Left arm position
        if name[id] == 'left_shoulder_pitch':
            joint_name[12] = name[id]
            joint_goal_position[12] = all_positions[req.robot_pose][id]
        if name[id] == 'left_shoulder_roll':
            joint_name[13] = name[id]
            joint_goal_position[13] = all_positions[req.robot_pose][id]
        if name[id] == 'left_elbow_pitch':
            joint_name[14] = name[id]
            joint_goal_position[14] = all_positions[req.robot_pose][id]     
        # Right arm position
        if name[id] == 'right_shoulder_pitch':
            joint_name[15] = name[id]
            joint_goal_position[15] = all_positions[req.robot_pose][id]
        if name[id] == 'right_shoulder_roll':
            joint_name[16] = name[id]
            joint_goal_position[16] = all_positions[req.robot_pose][id]
        if name[id] == 'right_elbow_pitch':
            joint_name[17] = name[id]
            joint_goal_position[17] = all_positions[req.robot_pose][id]  
        # Head position
        if name[id] == 'neck_yaw':
            joint_name[18] = name[id]
            joint_goal_position[18] = all_positions[req.robot_pose][id]
        if name[id] == 'head_pitch':
            joint_name[19] = name[id]
            joint_goal_position[19] = all_positions[req.robot_pose][id]

    res.delay = pose_duration[req.robot_pose] 
    res.number_poses = number_poses
    res.joint_name = joint_name
    res.joint_goal_position.data = joint_goal_position


if __name__ == '__main__':
    main()
