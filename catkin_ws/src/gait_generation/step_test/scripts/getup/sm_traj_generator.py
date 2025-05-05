#!/usr/bin/env python
import numpy as np                                                                                                      #     (\)_(/)  
import os                                                                                                              #     (__´_`__)  
import glob                                                                                                            #     _ooo_ooo_ 
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
        smach.State.__init__(self, outcomes=['new','edit', 'fail'])
        self.state = "INIT"

    def execute(self, userdata):
        global actual_pose, edit_pose
        print('STATE MACHINE GET UP POSES ->' + self.state)
        actual_pose=0
        action=int(input("Ingrese 1 para editar todas las poses (Todas las poses anteriores serán eliminadas), ingrese 2 para editar una sola pose:_"))
        if action == 1:
            archivos_npz = glob.glob(os.path.join(trajectory_dir, '*.npz'))
            for archivo in archivos_npz:
                try:
                    os.remove(archivo)
                except Exception as e:
                    print(f'Error al eliminar {archivo}: {e}')
            return 'new'
        elif action == 2:
            edit_pose=int(input("Ingrese la pose a editar:_"))
            return 'edit'
        else:
            print("No ingreso una opción correcta")
            return 'fail'
    
class Start_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'])
        self.state = "START_POSE"

    def execute(self, userdata):
        global initial_l_arm, initial_l_leg, initial_r_arm, initial_r_leg
        print('STATE MACHINE GET UP POSES ->' + self.state + ' POSE:_' + str(actual_pose-1))
        input("Presione enter cuando el humanoide esté en la pose inicial:_")
        initial_r_leg=r_leg
        initial_l_leg=l_leg
        initial_r_arm=r_arm
        initial_l_arm=l_arm
        return 'succ'

class New_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail', 'end'])
        self.state = "NEW_POSE?"

    def execute(self, userdata):
        global actual_pose
        global initial_l_arm, initial_l_leg, initial_r_arm, initial_r_leg
        print('STATE MACHINE GET UP POSES ->' + self.state + ' POSE:_' + str(actual_pose))
        new_pose=input("Nueva pose? y/n:_")
        if new_pose =='y':
            actual_pose+=1
            initial_r_leg=final_r_leg
            initial_l_leg=final_l_leg
            initial_r_arm=final_r_arm
            initial_l_arm=final_l_arm
            return 'succ'
        elif new_pose=='n':
            return 'end'
        else:
            print("Ingrese una opción válida")
            return 'fail'


class End_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'])
        self.state = "POSE"

    def execute(self, userdata):
        global actual_pose
        global final_l_arm, final_l_leg, final_r_arm, final_r_leg
        print('STATE MACHINE GET UP POSES ->' + self.state + ' POSE:_' + str(actual_pose))
        input("Presione enter cuando el humanoide esté en la siguiente pose:_")
        try:
            final_r_leg=r_leg
            final_l_leg=l_leg
            final_r_arm=r_arm
            final_l_arm=l_arm

            q_left=calculate_cartesian(1,initial_l_leg, final_l_leg)
            q_right=calculate_cartesian(1,initial_r_leg, final_r_leg)
            np.savez(os.path.join(trajectory_dir,"legs_pose"+str(actual_pose)), right_leg=q_right, left_leg=q_left, timestep=SERVO_SAMPLE_TIME)

            q_left=calculate_cartesian(1,initial_l_arm, final_l_arm)
            q_right=calculate_cartesian(1,initial_r_arm, final_r_arm)
            np.savez(os.path.join(trajectory_dir,("arms_pose"+str(actual_pose))), right_arm=q_right, left_arm=q_left, timestep=SERVO_SAMPLE_TIME)
            return 'succ'
        except Exception as e:
            print(f"Error al guardar las poses {e}")
            return 'fail'

class Edit_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'])
        self.state = "EDIT_POSE"

    def execute(self, userdata):
        global actual_pose
        global final_l_arm, final_l_leg, final_r_arm, final_r_leg
        print('STATE MACHINE GET UP POSES ->' + self.state + ' POSE:_' + str(edit_pose))
        try:
            past_arms=np.load(trajectory_dir+"arms_pose"+str(edit_pose-1))
            past_legs=np.load(trajectory_dir+"legs_pose"+str(edit_pose-1))
            initial_r_leg=past_legs["right_leg"][-1]
            initial_l_leg=past_legs["left_leg"][-1]
            initial_r_arm=past_arms["right_arm"][-1]
            initial_l_arm=past_arms["left_arm"][-1]
            final_r_leg=r_leg
            final_l_leg=l_leg
            final_r_arm=r_arm
            final_l_arm=l_arm

            q_left=calculate_cartesian(1,initial_l_leg, final_l_leg)
            q_right=calculate_cartesian(1,initial_r_leg, final_r_leg)
            np.savez(os.path.join(trajectory_dir,"legs_pose"+str(edit_pose)), right_leg=q_right, left_leg=q_left, timestep=SERVO_SAMPLE_TIME)

            q_left=calculate_cartesian(1,initial_l_arm, final_l_arm)
            q_right=calculate_cartesian(1,initial_r_arm, final_r_arm)
            np.savez(os.path.join(trajectory_dir,("arms_pose"+str(edit_pose))), right_arm=q_right, left_arm=q_left, timestep=SERVO_SAMPLE_TIME)
            return 'succ'
        except Exception as e:
            print(f"Error al guardar las poses {e}")
            return 'fail'

def main():
    global trajectory_dir
    rospy.init_node("generate_getup_poses_sm")
    trajectory_dir = rospy.get_param("~trajectory_dir")
    rospy.Subscriber("/joint_states", JointState, callback)
    sm = smach.StateMachine(outcomes=['End'])


    with sm:
    
        smach.StateMachine.add('Initial', Init(),
                               transitions={'new': 'Start_pose',
                                            'edit': 'Edit_pose',
                                            'fail': 'Initial'})
        smach.StateMachine.add('Edit_pose', Edit_pose(),
                               transitions={'succ': 'New_pose?',
                                            'fail': 'Edit_pose'})
        smach.StateMachine.add('New_pose?', New_pose(),
                               transitions={'succ': 'End_pose',
                                            'end': 'End',
                                            'fail': 'New_pose?'})
        smach.StateMachine.add('Start_pose', Start_pose(),
                               transitions={'succ': 'End_pose',
                                            'fail': 'Start_pose'}),
        smach.StateMachine.add('End_pose', End_pose(),
                               transitions={'succ': 'New_pose?', 
                                            'fail': 'End_pose'})


    outcome = sm.execute()

if __name__ == '__main__':
    main()
