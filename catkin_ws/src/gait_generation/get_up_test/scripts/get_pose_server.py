#!/usr/bin/env python
import rospy
import numpy
import yaml
import rospkg

from std_msgs.msg import *
from get_up_test.srv import *



def read_config_files():
    global prone_poses, supine_poses, name
    rospack = rospkg.RosPack()
    rospack.list()
    
    prone_poses = yaml.load(open(rospack.get_path('config_files') + '/predef_poses/PRONE_GETUP.yaml', 'r'))
    supine_poses = yaml.load(open(rospack.get_path('config_files') + '/predef_poses/SUPINE_GETUP.yaml', 'r'))
    name = prone_poses['motion'][0]['joints'].keys()


def main():
    print("--------get_pose_server by Luis Nava--------")
    rospy.init_node('get_pose_server', anonymous=True)    
    server = rospy.Service('/get_up_test/get_pose', getPose, allocate_positions)
    rospy.spin()


def allocate_positions(req):
    res = getPoseResponse()

    position = []

    joint_name          = [""]*20
    joint_goal_position = Float32MultiArray()
    joint_goal_position = numpy.zeros(20)

    if req.get_up_mode == 'prone': 
        for i in range(0, 20):
            position.append(prone_poses['motion'][req.robot_pose]['joints'][name[i]]['position'])
        res.number_poses = len(prone_poses['motion'])        
        res.delay = prone_poses['motion'][req.robot_pose]['duration']

    elif req.get_up_mode == 'supine':
        for i in range(0, 20):
            position.append(supine_poses['motion'][req.robot_pose]['joints'][name[i]]['position'])
        res.number_poses = len(supine_poses['motion'])       
        res.delay = supine_poses['motion'][req.robot_pose]['duration']

    else:
        print "Invalid get up mode"

    for id in range(0, 20):
        #Legs positions
        if name[id] == 'left_hip_yaw':
            joint_name[0] = name[id]
            joint_goal_position[0] = position[id]
        if name[id] == 'left_hip_roll':
            joint_name[1] = name[id]
            joint_goal_position[1] = position[id]
        if name[id] == 'left_hip_pitch':
            joint_name[2] = name[id]
            joint_goal_position[2] = position[id]
        if name[id] == 'left_knee_pitch':
            joint_name[3] = name[id]
            joint_goal_position[3] = position[id]
        if name[id] == 'left_ankle_pitch':
            joint_name[4] = name[id]
            joint_goal_position[4] = position[id]
        if name[id] == 'left_ankle_roll':
            joint_name[5] = name[id]
            joint_goal_position[5] = position[id]
        if name[id] == 'right_hip_yaw':
            joint_name[6] = name[id]
            joint_goal_position[6] = position[id]
        if name[id] == 'right_hip_roll':
            joint_name[7] = name[id]
            joint_goal_position[7] = position[id]
        if name[id] == 'right_hip_pitch':
            joint_name[8] = name[id]
            joint_goal_position[8] = position[id]
        if name[id] == 'right_knee_pitch':
            joint_name[9] = name[id]
            joint_goal_position[9] = position[id]
        if name[id] == 'right_ankle_pitch':
            joint_name[10] = name[id]
            joint_goal_position[10] = position[id]
        if name[id] == 'right_ankle_roll':
            joint_name[11] = name[id]
            joint_goal_position[11] = position[id]
        # Left arm position
        if name[id] == 'left_shoulder_pitch':
            joint_name[12] = name[id]
            joint_goal_position[12] = position[id]
        if name[id] == 'left_shoulder_roll':
            joint_name[13] = name[id]
            joint_goal_position[13] = position[id]
        if name[id] == 'left_elbow_pitch':
            joint_name[14] = name[id]
            joint_goal_position[14] = position[id]     
        # Right arm position
        if name[id] == 'right_shoulder_pitch':
            joint_name[15] = name[id]
            joint_goal_position[15] = position[id]
        if name[id] == 'right_shoulder_roll':
            joint_name[16] = name[id]
            joint_goal_position[16] = position[id]
        if name[id] == 'right_elbow_pitch':
            joint_name[17] = name[id]
            joint_goal_position[17] = position[id]  
        # Head position
        if name[id] == 'neck_yaw':
            joint_name[18] = name[id]
            joint_goal_position[18] = position[id]
        if name[id] == 'head_pitch':
            joint_name[19] = name[id]
            joint_goal_position[19] = position[id]




    print"Robot state: " , req.robot_pose
    print"get_up_mode: ", req.get_up_mode
    print "Delay: " , res.delay
    print "----------------------"

    res.joint_name = joint_name
    res.joint_goal_position.data = joint_goal_position
    return res

if __name__ == '__main__':
    read_config_files()
    main()
    
