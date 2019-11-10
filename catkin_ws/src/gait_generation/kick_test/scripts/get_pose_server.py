#!/usr/bin/env python
import rospy
import numpy
import yaml
import rospkg

from std_msgs.msg import *
from kick_test.srv import *



def read_config_files():
    global left_poses, right_poses, joint_name
    rospack = rospkg.RosPack()
    rospack.list()
    
    left_poses = yaml.load(open(rospack.get_path('config_files') + '/predef_poses/LEFT_KICK.yaml', 'r'))
    right_poses = yaml.load(open(rospack.get_path('config_files') + '/predef_poses/RIGHT_KICK.yaml', 'r'))
    joint_name = left_poses['motion'][0]['joints'].keys()


def main():
    print("--------get_pose_server by Luis Nava--------")
    rospy.init_node('get_pose_server', anonymous=True)    
    server = rospy.Service('/kick_test/get_pose', getPose, allocate_positions)
    rospy.spin()


def allocate_positions(req):
    res = getPoseResponse()

    joint_position = []

    head_position = Float32MultiArray()
    legs_position = Float32MultiArray()
    left_arm_position = Float32MultiArray()
    right_arm_position = Float32MultiArray()

    head_position = numpy.zeros(2)
    legs_position = numpy.zeros(12)
    left_arm_position = numpy.zeros(3)
    right_arm_position = numpy.zeros(3)

    if req.kick_mode == 'left': 
        for i in range(0, 20):
            joint_position.append(left_poses['motion'][req.robot_pose]['joints'][joint_name[i]]['position'])
        res.number_poses = len(left_poses['motion'])        
        res.delay = left_poses['motion'][req.robot_pose]['duration']

    elif req.kick_mode == 'right':
        for i in range(0, 20):
            joint_position.append(right_poses['motion'][req.robot_pose]['joints'][joint_name[i]]['position'])
        res.number_poses = len(right_poses['motion'])       
        res.delay = right_poses['motion'][req.robot_pose]['duration']

    else:
        print "Invalid kick mode"

    for id in range(0, 20):
        if joint_name[id] == 'neck_yaw':
            head_position[0] = joint_position[id]
        if joint_name[id] == 'head_pitch':
            head_position[1] = joint_position[id]

        if joint_name[id] == 'left_shoulder_pitch':
            left_arm_position[0] = joint_position[id]
        if joint_name[id] == 'left_shoulder_roll':
            left_arm_position[1] = joint_position[id]
        if joint_name[id] == 'left_elbow_pitch':
            left_arm_position[2] = joint_position[id]     

        if joint_name[id] == 'right_shoulder_pitch':
            right_arm_position[0] = joint_position[id]
        if joint_name[id] == 'right_shoulder_roll':
            right_arm_position[1] = joint_position[id]
        if joint_name[id] == 'right_elbow_pitch':
            right_arm_position[2] = joint_position[id]  

        if joint_name[id] == 'left_hip_yaw':
            legs_position[0] = joint_position[id]
        if joint_name[id] == 'left_hip_roll':
            legs_position[1] = joint_position[id]
        if joint_name[id] == 'left_hip_pitch':
            legs_position[2] = joint_position[id]
        if joint_name[id] == 'left_knee_pitch':
            legs_position[3] = joint_position[id]
        if joint_name[id] == 'left_ankle_pitch':
            legs_position[4] = joint_position[id]
        if joint_name[id] == 'left_ankle_roll':
            legs_position[5] = joint_position[id]
        if joint_name[id] == 'right_hip_yaw':
            legs_position[6] = joint_position[id]
        if joint_name[id] == 'right_hip_roll':
            legs_position[7] = joint_position[id]
        if joint_name[id] == 'right_hip_pitch':
            legs_position[8] = joint_position[id]
        if joint_name[id] == 'right_knee_pitch':
            legs_position[9] = joint_position[id]
        if joint_name[id] == 'right_ankle_pitch':
            legs_position[10] = joint_position[id]
        if joint_name[id] == 'right_ankle_roll':
            legs_position[11] = joint_position[id]


    print"Robot state: " , req.robot_pose
    print"kick_mode: ", req.kick_mode
    print "Delay: " , res.delay
    print "----------------------"


    res.head_position.data = head_position
    res.legs_position.data = legs_position    
    res.left_arm_position.data = left_arm_position
    res.right_arm_position.data = right_arm_position

    return res

if __name__ == '__main__':
    read_config_files()
    main()
    