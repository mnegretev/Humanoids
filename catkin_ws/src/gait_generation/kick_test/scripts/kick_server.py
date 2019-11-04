#!/usr/bin/env python
import rospy
import yaml
import rospkg

from kick_test.srv import *

def read_predef_poses():
    global left_poses, right_poses, left_states, right_states, joint_names
    rospack = rospkg.RosPack()
    rospack.list()
    
    left_poses = yaml.load(open(rospack.get_path('config_files') + '/predef_poses/LEFT_KICK.yaml', 'r'))
    left_states = len(left_poses['motion'])

    right_poses = yaml.load(open(rospack.get_path('config_files') + '/predef_poses/RIGHT_KICK.yaml', 'r'))
    right_states = len(right_poses['motion'])
    
    joint_names = left_poses['motion'][0]['joints'].keys()


def handle_kick(req):
    print"Robot state: " , req.robot_state

    res = kickResponse()
    positions = []

    if req.kick_mode == 'left':
        print"kick_mode: ", req.kick_mode
        
        for index in range(0, 20):
            positions.append(left_poses['motion'][req.robot_state]['joints'][joint_names[index]]['position'])
        
        res.states = left_states        
        res.delay = left_poses['motion'][req.robot_state]['duration']

    elif req.kick_mode == 'right':
        print"kick_mode: ", req.kick_mode
        
        for index in range(0, 20):
            positions.append(right_poses['motion'][req.robot_state]['joints'][joint_names[index]]['position'])

        res.states = right_states        
        res.delay =right_poses['motion'][req.robot_state]['duration']

    else:
        print "Invalid argument"

    print "Delay: " , res.delay
    print "----------------------"

    res.name = joint_names
    res.position = positions
    return res


def left_kick():
    print("Starting kick_server by Luis Nava...")
    rospy.init_node('kick_server', anonymous=True)    

    s = rospy.Service('/joints_states_profiles', kick, handle_kick)
    rospy.spin()


if __name__ == '__main__':
    read_predef_poses()
    left_kick()
    