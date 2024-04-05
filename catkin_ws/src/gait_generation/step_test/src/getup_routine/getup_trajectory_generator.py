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
    q_right_final = [-0.232,-0.071,-0.107,1.57,-0.311,-0.043]
    leg_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_right_inicial,q_right_final,  duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    q_left_inicial = [0, 0.006, 0.048, 0.051, 0.010, 0.002]
    q_left_final = [-0.018, 0.040, -0.138, 1.552, -0.123, -0.083]
    leg_left_q,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_left_inicial, q_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return leg_left_q,leg_right_q
def calculate_cartesian_arms_pose1(duration):
    #arms
    q_arm_right_inicial =[-0.015,-0.011,0.010]
    q_arm_right_final   =[1.009,-1.301,-1.566]
    arm_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_right_inicial, q_arm_right_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    q_arm_left_inicial  =[-0.016,0,0]
    q_arm_left_final    =[0.853,1.224,-1.488]
    arm_left_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_left_inicial, q_arm_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    return arm_left_q, arm_right_q

#--------------------------------------POSE 2---------------------------------------------------------------------
def calculate_cartesian_legs_pose2(duration):
    #legs
    q_right_inicial     =  [-0.232,-0.071,-0.107,1.57,-0.311,-0.043]
    q_right_final       = [0.017, -0.092, -0.925, 1.502, -0.887, 0.118]
    leg_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_right_inicial,q_right_final,  duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    q_left_inicial  =[-0.018, 0.040, -0.138, 1.552, -0.123, -0.083]
    q_left_final    = [0.032,0.123,-0.941,1.562,-0.795,-0.083]
    leg_left_q,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_left_inicial, q_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return leg_left_q,leg_right_q
def calculate_cartesian_arms_pose2(duration):
    #arms
    q_arm_right_inicial=[1.009,-1.301,-1.566]
    q_arm_right_final=[0.528,-0.801,-1.570]
    arm_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_right_inicial, q_arm_right_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    q_arm_left_inicial=[0.853,1.224,-1.488]
    q_arm_left_final=[0.592,0.573,-1.57]
    arm_left_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_left_inicial, q_arm_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    return arm_left_q, arm_right_q
#---------------------------------------------POSE 3----------------------------------------------------------------------------------
def calculate_cartesian_legs_pose3(duration):
    #legs
    q_right_inicial = [0.017, -0.092, -0.925, 1.502, -0.887, 0.118]
    q_right_final = [-0.058, -0.078, -1.48, 0.674, -0.624, 0.044]
    leg_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_right_inicial,q_right_final,  duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    q_left_inicial =[0.032,0.123,-0.941,1.562,-0.795,-0.083]
    q_left_final = [0.138,0.052,-1.518,0.821,-0.590,-0.072]
    leg_left_q,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_left_inicial, q_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return leg_left_q,leg_right_q
def calculate_cartesian_arms_pose3(duration):  
    #arms
    q_arm_right_inicial=[0.528,-0.801,-1.570]
    q_arm_right_final=[0.274,-0.49,-1.4547]
    arm_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_right_inicial, q_arm_right_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    q_arm_left_inicial=[0.592,0.573,-1.57]
    q_arm_left_final=[0.174,0.183,-1.54]
    arm_left_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_left_inicial, q_arm_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    return arm_left_q, arm_right_q

#---------------------------------------------POSE 4----------------------------------------------------------------------------------
def calculate_cartesian_legs_pose4(duration):
    #legs
    q_right_inicial = [-0.058, -0.078, -1.48, 0.674, -0.624, 0.044]
    q_right_final = [-0.100, -0.081, -1.44, 1.564, -0.614, 0.075]
    leg_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_right_inicial,q_right_final,  duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    q_left_inicial =[0.138,0.052,-1.518,0.821,-0.590,-0.072]
    q_left_final = [0.011,0.147,-1.508,1.571,-0.459,-0.132]
    leg_left_q,T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_left_inicial, q_left_final, duration=duration, time_step=SERVO_SAMPLE_TIME)
 
    return leg_left_q,leg_right_q
def calculate_cartesian_arms_pose4(duration):  
    #arms
    q_arm_right_inicial=[0.274,-0.49,-1.4547]
    q_arm_right_final=[-1.117,-0.175,0.769]
    arm_right_q, T = trajectory_planner.get_polynomial_trajectory_multi_dof(q_arm_right_inicial, q_arm_right_final, duration=duration, time_step=SERVO_SAMPLE_TIME)

    q_arm_left_inicial=[0.174,0.183,-1.54]
    q_arm_left_final=[-0.991,0.014,0.545]
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