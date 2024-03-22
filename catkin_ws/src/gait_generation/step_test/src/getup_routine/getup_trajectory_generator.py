#!/usr/bin/env python
import numpy as np
from scipy import signal, interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

#ROS
import rospy
from std_msgs.msg import String, Float32MultiArray
from trajectory_planner import trajectory_planner

Y_BODY_TO_FEET  = 0.0555
Z_ROBOT_STATIC= 0.56 # m

# Tiempo de muestreo maximo para escribir a los servomotores
SERVO_SAMPLE_TIME = 0.025 # [s]
#---------------------------------POSE 1----------------------------------------------------------------------------
def calculate_cartesian_legs_pose1(duration):
    #legs
    q_right_inicial = [-0.002, 0.009, 0.016, 0.054, 0.005, 0.0]
    q_right_final = [-0.250,-0.004,0.009,0.309,-0.109,0.110]
    leg_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_right_inicial,q_right_final,  duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    q_left_inicial = [0, 0.006, 0.048, 0.051, 0.010, 0.002]
    q_left_final = [0.014, 0.051, -0.014, 0.345, 0.011, -0.014]
    leg_left_q,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_left_inicial, q_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return leg_left_q,leg_right_q
def calculate_cartesian_arms_pose1(duration):
    #arms
    q_arm_right_inicial =[-0.015,-0.011,0.010]
    q_arm_right_final   =[1.155,-0.166,-1.570]
    arm_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_right_inicial, q_arm_right_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    q_arm_left_inicial  =[-0.016,0,0]
    q_arm_left_final    =[1.101,0.018,-1.55]
    arm_left_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_left_inicial, q_arm_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    return arm_left_q, arm_right_q

#--------------------------------------POSE 2---------------------------------------------------------------------
def calculate_cartesian_legs_pose2(duration):
    #legs
    q_right_inicial     =  [-0.250,-0.004,0.009,0.308,-0.109,0.110]
    q_right_final       = [-0.235, -0.044, -0.853, 1.367, -0.95, 0.029]
    leg_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_right_inicial,q_right_final,  duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    q_left_inicial  =[0.015, 0.051, -0.014, 0.345, 0.011, -0.014]
    q_left_final    = [-0.004,-0.003,-0.871,1.339,-0.844,-0.015]
    leg_left_q,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_left_inicial, q_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return leg_left_q,leg_right_q
def calculate_cartesian_arms_pose2(duration):
    #arms
    q_arm_right_inicial=[1.153,-0.166,-1.570]
    q_arm_right_final=[0.078,-0.243,-0.779]
    arm_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_right_inicial, q_arm_right_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    q_arm_left_inicial=[1.101,0.018,-1.57]
    q_arm_left_final=[0.009,-0.009,-0.814]
    arm_left_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_left_inicial, q_arm_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    return arm_left_q, arm_right_q
#---------------------------------------------POSE 3----------------------------------------------------------------------------------
# def calculate_cartesian_legs_pose3(duration):
#     #legs
#     q_right_inicial = [-0.236,0.038,0.087,0.160,0.302,0.013]
#     q_right_final = [-0.152, -0.064, -0.413, 1.057, -0.394, 0.127]
#     leg_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_right_inicial,q_right_final,  duration=duration, time_step=SERVO_SAMPLE_TIME)
    
#     q_left_inicial = [-0.011, 0.092, 0.023, 0.301, 0.201, -0.013]
#     q_left_final = [-0.02,0.043,-0.478,1.108,-0.420,-0.008]
#     leg_left_q,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_left_inicial, q_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
#     return leg_left_q,leg_right_q
# def calculate_cartesian_arms_pose3(duration):  
#     #arms
#     q_arm_right_inicial=[0.923,-0.370,-1.553]
#     q_arm_right_final=[0.189,-0.5,-1.5]
#     arm_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_right_inicial, q_arm_right_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

#     q_arm_left_inicial=[0.867,0.298,-1.465]
#     q_arm_left_final=[0.172,0.5,-1.365]
#     arm_left_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_left_inicial, q_arm_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    return arm_left_q, arm_right_q



def main(args = None):
    rospy.init_node('getup_test_node')
    #-------------------POSE 1-------------------------------------------------
    #legs1
    leg_left_q, leg_right_q= calculate_cartesian_legs_pose1(1)
    np.savez("legs_start_pose1", right_leg=leg_right_q, left_leg=leg_left_q, timestep=SERVO_SAMPLE_TIME)
    #arms1
    arm_left_q, arm_right_q= calculate_cartesian_arms_pose1(1)
    np.savez("arms_start_pose1", right_arm=arm_right_q, left_arm=arm_left_q, timestep=SERVO_SAMPLE_TIME)
    #--------------------POSE 2-------------------------------------------------------
     #legs2
    leg_left_q, leg_right_q= calculate_cartesian_legs_pose2(1)
    np.savez("legs_start_pose2", right_leg=leg_right_q, left_leg=leg_left_q, timestep=SERVO_SAMPLE_TIME)
    #arms2
    arm_left_q, arm_right_q= calculate_cartesian_arms_pose2(1)
    np.savez("arms_start_pose2", right_arm=arm_right_q, left_arm=arm_left_q, timestep=SERVO_SAMPLE_TIME)

if __name__ == "__main__":
    main()