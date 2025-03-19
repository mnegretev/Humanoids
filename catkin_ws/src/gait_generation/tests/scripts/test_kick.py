#!/usr/bin/python3
import numpy as np
import math
import os
import time

#ROS
import rospy
from std_msgs.msg import String, Float32MultiArray
from ctrl_msgs.srv import CalculateIK, CalculateIKRequest
from trajectory_planner import trajectory_planner
from manip_msgs.srv import *

Z_ROBOT_WALK    = None
Z_ROBOT_STATIC  = None
Y_BODY_TO_FEET  = None
com_x_offset    = None #original=0.02
com_y_offset    = None # 
kick_length     = None
kick_height     = None

# Tiempo de muestreo maximo para escribir a los servomotores
SERVO_SAMPLE_TIME = 0.025 # [s]

def calculate_ik(P, service_client):
    joint_values = np.zeros((len(P),6))
    for i, vector in enumerate(P):
        print(f"[{i}]: {vector}")
        req = CalculateIKRequest(x = vector[0], y = vector[1], z = vector[2],
                                 roll = 0, pitch = 0, yaw = 0)                         
        try:
            response = service_client(req)
            print(response)
            if len(response.joint_values) == 6:
                aux = np.array([list(response.joint_values)])
                joint_values[i] = aux
        except Exception as e:
            print("Could not calculate inverse kinematics for pose {vector}")
            joint_values[i] = joint_values[i-1]
            #raise ValueError("Error calculating inverse kinematics for point {vector}")
    return joint_values

def calculate_cartesian_right_start_pose(y_body_to_feet_percent, ik_client_left, ik_client_right):
    p_start = [0 + com_x_offset, 0, Z_ROBOT_STATIC]
    p_end = [0 + com_x_offset, -y_body_to_feet_percent*Y_BODY_TO_FEET, Z_ROBOT_WALK]

    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_end, time_step=SERVO_SAMPLE_TIME)
    
    left_leg_relative_pos   = [0,  Y_BODY_TO_FEET, 0] - P_CoM
    right_leg_relative_pos  = [0, -Y_BODY_TO_FEET, 0] - P_CoM

    left_q  = calculate_ik(left_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(right_leg_relative_pos, ik_client_right)

    return left_q, right_q, P_CoM[-1]

def calculate_cartesian_left_first_step_pose(p_start, ik_client_left, ik_client_right):
    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_start, time_step=SERVO_SAMPLE_TIME)
    
    initial_r_foot_pos  = np.array([0, -Y_BODY_TO_FEET, 0])
    initial_l_foot_pos  = np.array([0,  Y_BODY_TO_FEET, 0])

    #final_r_foot_pos    = initial_r_foot_pos
    final_l_foot_pos    = np.array([-kick_length, Y_BODY_TO_FEET*1.1, kick_height])

    r_leg_abs_pos = np.full((len(T), 3), initial_r_foot_pos)
    l_leg_abs_pos, T = trajectory_planner.get_polynomial_trajectory_multi_dof(initial_l_foot_pos, final_l_foot_pos, time_step=SERVO_SAMPLE_TIME)

    r_leg_relative_pos  = r_leg_abs_pos - P_CoM
    l_leg_relative_pos  = l_leg_abs_pos - P_CoM

    left_q = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, l_leg_abs_pos

def executeTrajectories(left_foot_q, right_foot_q, rate: rospy.Rate, legs_publisher: rospy.Publisher):
    for right, left in zip(left_foot_q, right_foot_q):
        legs_msg = Float32MultiArray()
        legs_msg.data = [*left] + [*right]
        print(legs_msg.data)
        legs_publisher.publish(legs_msg)
        rate.sleep()

def main(args = None):
    global Y_BODY_TO_FEET, Z_ROBOT_WALK, Z_ROBOT_STATIC
    global kick_length, kick_height, com_x_offset, com_y_offset
    rospy.init_node('step_test_node')
    kick_height     = rospy.get_param("/kick/kick_height")
    kick_length     = rospy.get_param("/kick/kick_length")
    com_x_offset    = rospy.get_param("/kick/com_x_offset")
    com_y_offset    = rospy.get_param("/kick/com_y_offset")
    Z_ROBOT_WALK    = rospy.get_param("/kick/z_robot_walk")
    Z_ROBOT_STATIC  = rospy.get_param("/kick/z_robot_static")
    Y_BODY_TO_FEET  = rospy.get_param("/kick/y_body_to_feet")

    print(f"Kick height is:\t\t {kick_height}")
    print(f"Kick length is:\t\t {kick_length}")
    print(f"com_x_offset is:\t {com_x_offset}")
    print(f"com_y_offset is:\t {com_y_offset}")
    print(f"z_robot_walk is:\t {Z_ROBOT_WALK}")
    print(f"z_robot_static is:\t {Z_ROBOT_STATIC}")
    print(f"y_body_to_feet is:\t {Y_BODY_TO_FEET}")

    arms_goal_pose      = rospy.Publisher("/hardware/arms_goal_pose", Float32MultiArray , queue_size=1)
    pub_legs_goal       = rospy.Publisher("/hardware/legs_goal_pose", Float32MultiArray, queue_size=1)
    right_leg_client    = rospy.ServiceProxy('/manipulation/ik_leg_right_pose', CalculateIK)
    left_leg_client     = rospy.ServiceProxy('/manipulation/ik_leg_left_pose', CalculateIK)
    
    rospy.wait_for_service("/manipulation/ik_leg_right_pose")
    rospy.wait_for_service("/manipulation/ik_leg_left_pose")

    rate = rospy.Rate(10)

    first_left_q, first_right_q, last_p_com = calculate_cartesian_right_start_pose(0.8, left_leg_client, right_leg_client)
    second_left_q, second_right_q, _        = calculate_cartesian_left_first_step_pose(last_p_com, left_leg_client, right_leg_client)

    arms_msg = Float32MultiArray()
    arms_msg.data = [0.0, 0.3, 0.0, 0.0, -0.3, 0.0]
    arms_goal_pose.publish(arms_msg)

    executeTrajectories(first_left_q,  first_right_q,  rate, pub_legs_goal)
    executeTrajectories(second_left_q, second_right_q, rate, pub_legs_goal)

    


if __name__ == "__main__":
    main()