#!/usr/bin/env python
import numpy as np
import os
from scipy import signal, interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import smach

#ROS
import rospy
from trajectory_planner import trajectory_planner
from sensor_msgs.msg import JointState
import smach_ros

Y_BODY_TO_FEET  = 0.0555
Z_ROBOT_STATIC= 0.56 # m

# Tiempo de muestreo maximo para escribir a los servomotores
SERVO_SAMPLE_TIME = 0.025 # [s]

def calculate_cartesian(duration, q_inicial, q_final):

    trajectory,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_inicial, q_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    return trajectory


def callback(data):
    global r_leg, l_leg, r_arm, l_arm, head
    positions = data.position
    r_arm=[]
    l_arm=[]
    r_leg=[]
    l_leg=[]
    head=[positions[-2], positions[-1]]
    for i in range(1,7,2):
        r_arm.append(positions[i])
    for i in range(2,7,2):
        l_arm.append(positions[i])
    for i in range(7,19,2):
        r_leg.append(positions[i])
    for i in range(8,19,2):
        l_leg.append(positions[i])
 


class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'])
        self.state = "INIT"

    def execute(self, userdata):
        global poses, actual_pose
        print('STATE MACHINE GET UP POSES ->' + self.state)
        poses=0
        actual_pose=int(input("Ingrese el numero de poses:_"))
        poses=actual_pose+1

        return 'succ'
    
class Start_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'])
        self.state = "START_POSE"

    def execute(self, userdata):
        global initial_l_arm, initial_l_leg, initial_r_arm, initial_r_leg
        print('STATE MACHINE GET UP POSES ->' + self.state + ' POSE:_' + str(poses-actual_pose))
        input("Presione enter cuando el humanoide esté en la pose inicial:_")
        initial_r_leg=r_leg
        initial_l_leg=l_leg
        initial_r_arm=r_arm
        initial_l_arm=l_arm
        return 'succ'


class End_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail', 'end'])
        self.state = "END_POSE"

    def execute(self, userdata):
        global actual_pose
        print('STATE MACHINE GET UP POSES ->' + self.state + ' POSE:_' + str(poses-actual_pose))
        input("Presione enter cuando el humanoide esté en la pose final:_")
        if actual_pose ==1:
            return 'end'
        else:
            actual_pose-=1
            final_r_leg=r_leg
            final_l_leg=l_leg
            final_r_arm=r_arm
            final_l_arm=l_arm

            q_left=calculate_cartesian(1,initial_l_leg, final_l_leg)
            q_right=calculate_cartesian(1,initial_r_leg, final_r_leg)
            np.savez(os.path.join(trajectory_dir,"legs_pose"+str(poses-actual_pose)), right_leg=q_right, left_leg=q_left, timestep=SERVO_SAMPLE_TIME)

            q_left=calculate_cartesian(1,initial_l_arm, final_l_arm)
            q_right=calculate_cartesian(1,initial_r_arm, final_r_arm)
            np.savez(os.path.join(trajectory_dir,("arms_pose"+str(poses-actual_pose))), right_arm=q_right, left_arm=q_left, timestep=SERVO_SAMPLE_TIME)
            return 'succ'


def main():
    global trajectory_dir
    rospy.init_node("generate_getup_poses_sm")
    trajectory_dir = rospy.get_param("~trajectory_dir")
    rospy.Subscriber("/joint_states", JointState, callback)
    sm = smach.StateMachine(outcomes=['End'])


    with sm:
    
        smach.StateMachine.add('Initial', Init(),
                               transitions={'succ': 'Start_pose',
                                            'fail': 'Initial'})
        smach.StateMachine.add('Start_pose', Start_pose(),
                               transitions={'succ': 'End_pose',
                                            'fail': 'Start_pose'}),
        smach.StateMachine.add('End_pose', End_pose(),
                               transitions={'succ': 'Start_pose', 
                                            'end' : 'End',
                                            'fail': 'End_pose'})


    outcome = sm.execute()

if __name__ == '__main__':
    main()
