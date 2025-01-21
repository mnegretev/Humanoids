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
    q_right_final = [0.041,-0.071,0.133,1.509,-0.029,-0.080]
    leg_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_right_inicial,q_right_final,  duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    q_left_inicial = [0, 0.006, 0.048, 0.051, 0.010, 0.002]
    q_left_final = [0.008, 0.040, 0.015, 1.496, 0.005, -0.083]
    leg_left_q,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_left_inicial, q_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return leg_left_q,leg_right_q
def calculate_cartesian_arms_pose1(duration):
    #arms
    q_arm_right_inicial =[-0.015,-0.011,0.010]
    q_arm_right_final   =[0.911,-0.118,-1.088]
    arm_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_right_inicial, q_arm_right_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    q_arm_left_inicial  =[-0.016,0,0]
    q_arm_left_final    =[0.830,0.044,-1.298]
    arm_left_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_left_inicial, q_arm_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    return arm_left_q, arm_right_q

#--------------------------------------POSE 2---------------------------------------------------------------------
def calculate_cartesian_legs_pose2(duration):
    #legs
    q_right_inicial     =  [0.041,-0.071,0.133,1.509,-0.029,-0.080]
    q_right_final       = [-0.002, -0.104, -1.517, 1.562, -0.021, -0.083]
    leg_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_right_inicial,q_right_final,  duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    q_left_inicial  =[0.008, 0.040, 0.015, 1.496, 0.005, -0.083]
    q_left_final    = [0.060,-0.012,-1.570,1.563,0.014,-0.089]
    leg_left_q,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_left_inicial, q_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return leg_left_q,leg_right_q
def calculate_cartesian_arms_pose2(duration):
    #arms
    q_arm_right_inicial=[0.911,-0.118,-1.088]
    q_arm_right_final=[-0.163,-0.825,-1.135]
    arm_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_right_inicial, q_arm_right_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    q_arm_left_inicial=[0.830,0.044,-1.298]
    q_arm_left_final=[-0.250,0.609,-1.138]
    arm_left_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_left_inicial, q_arm_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    return arm_left_q, arm_right_q
#---------------------------------------------POSE 3----------------------------------------------------------------------------------
def calculate_cartesian_legs_pose3(duration):
    #legs
    q_right_inicial = [-0.002, -0.104, -1.517, 1.562, -0.021, -0.083]
    q_right_final = [0.002, -0.097, -1.542, 1.570, -0.310, -0.064]
    leg_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_right_inicial,q_right_final,  duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    q_left_inicial =[0.060,-0.012,-1.570,1.563,0.014,-0.089]
    q_left_final = [0.063,-0.025,-1.560,1.570,-0.500,-0.075]
    leg_left_q,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_left_inicial, q_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return leg_left_q,leg_right_q
def calculate_cartesian_arms_pose3(duration):  
    #arms
    q_arm_right_inicial=[-0.163,-0.825,-1.135]
    q_arm_right_final=[-0.335,-0.353,-0.641]
    arm_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_right_inicial, q_arm_right_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    q_arm_left_inicial=[-0.250,0.609,-1.138]
    q_arm_left_final=[-0.498,0.032,-0.704]
    arm_left_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_left_inicial, q_arm_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    return arm_left_q, arm_right_q

#---------------------------------------------POSE 4----------------------------------------------------------------------------------
def calculate_cartesian_legs_pose4(duration):
    #legs
    q_right_inicial = [0.002, -0.097, -1.542, 1.570, -0.310, -0.064]
    q_right_final = [0.015, -0.097, 0.221, 1.552, -1.057, -0.057]
    leg_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_right_inicial,q_right_final,  duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    q_left_inicial =[0.063,-0.025,-1.560,1.570,-0.500,-0.075]
    q_left_final = [0.057,-0.037,0.138,1.571,-1.054,-0.062]
    leg_left_q,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_left_inicial, q_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return leg_left_q,leg_right_q
def calculate_cartesian_arms_pose4(duration):  
    #arms
    q_arm_right_inicial=[-0.335,-0.353,-0.641]
    q_arm_right_final=[-0.934,-0.362,0.049]
    arm_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_right_inicial, q_arm_right_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    q_arm_left_inicial=[-0.498,0.032,-0.704]
    q_arm_left_final=[-1.315,0.025,0.176]
    arm_left_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_left_inicial, q_arm_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

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
    #--------------------POSE 3-------------------------------------------------------
     #legs3
    leg_left_q, leg_right_q= calculate_cartesian_legs_pose3(1)
    np.savez("legs_start_pose3", right_leg=leg_right_q, left_leg=leg_left_q, timestep=SERVO_SAMPLE_TIME)
    #arms3
    arm_left_q, arm_right_q= calculate_cartesian_arms_pose3(1)
    np.savez("arms_start_pose3", right_arm=arm_right_q, left_arm=arm_left_q, timestep=SERVO_SAMPLE_TIME)
    #--------------------POSE 4-------------------------------------------------------
     #legs4
    leg_left_q, leg_right_q= calculate_cartesian_legs_pose4(1)
    np.savez("legs_start_pose4", right_leg=leg_right_q, left_leg=leg_left_q, timestep=SERVO_SAMPLE_TIME)
    #arms4
    arm_left_q, arm_right_q= calculate_cartesian_arms_pose4(1)
    np.savez("arms_start_pose4", right_arm=arm_right_q, left_arm=arm_left_q, timestep=SERVO_SAMPLE_TIME)
    print("Process finished")

if __name__ == "__main__":
    main()