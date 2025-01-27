#!/usr/bin/env python
import numpy as np
from scipy import signal, interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import rospy
from std_msgs.msg import String, Float32MultiArray
from trajectory_planner import trajectory_planner

SERVO_SAMPLE_TIME = 0.025 # [s]

def calculate_cartesian_head_point1 (duration):
    
    q_head_inicial = [1.36, 0.87]
    q_head_final = [-1.53, 0.87]
    head_movement,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_head_inicial, q_head_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return head_movement

def calculate_cartesian_head_point2 (duration):
    
    q_head_inicial = [-1.53, 0.87]
    q_head_final = [-1.53, 1.23]
    head_movement,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_head_inicial, q_head_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return head_movement

def calculate_cartesian_head_point3 (duration):
    
    q_head_inicial = [-1.53, 1.23]
    q_head_final = [1.22, 1.23]
    head_movement,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_head_inicial, q_head_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return head_movement
def calculate_cartesian_head_point4 (duration):
    
    q_head_inicial = [1.22, 1.23]
    q_head_final = [1.36, 0.87]
    head_movement,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_head_inicial, q_head_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return head_movement



def main (args = None):
	rospy.init_node("search_tgenerate_search_trajectoryrajectory")  

	head_movement= calculate_cartesian_head_point1(1)
	np.savez("head_point1", head=head_movement, timestep=SERVO_SAMPLE_TIME)

	head_movement= calculate_cartesian_head_point2(1)
	np.savez("head_point2", head=head_movement, timestep=SERVO_SAMPLE_TIME)

	head_movement= calculate_cartesian_head_point3(1)
	np.savez("head_point3", head=head_movement, timestep=SERVO_SAMPLE_TIME)

	head_movement= calculate_cartesian_head_point4(1)
	np.savez("head_point4", head=head_movement, timestep=SERVO_SAMPLE_TIME)
	print("Process finished")


if __name__=="__main__":
    main()
