#!/usr/bin/python3
import numpy as np
import math
import os
import time

#ROS
import rospy
from std_msgs.msg import String, Float32MultiArray
from ctrl_msgs.srv import CalculateIK, CalculateIKRequest, Lateral, LateralResponse
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

def handle_execute_kick(req):
    try:
        
        executeTrajectories(first_left_q,  first_right_q,  rate2, pub_legs_goal)
        executeTrajectories(second_left_q, second_right_q, rate, pub_legs_goal)
        executeTrajectories(third_left_q[-2:],  third_right_q[-2:],  rate, pub_legs_goal)
        succes=LateralResponse()
        succes.succes=True
        return succes
    except:

        succes = LateralResponse()
        succes.succes=False

        return succes
def calculate_ik(P, service_client):
    failed_counts = 0
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
            failed_counts += 1
            if (failed_counts > len(joint_values)*0.5):
                raise ValueError(f"{failed_counts} failed calculations. Aborting")
            print(f"Could not calculate inverse kinematics for pose {vector}")
            joint_values[i] = joint_values[i-1]
    return joint_values

def calculate_cartesian_right_start_pose(y_body_to_feet_percent, ik_client_left, ik_client_right):
    p_start = [0 + com_x_offset, 0, Z_ROBOT_STATIC]
    p_end = [0 + com_x_offset, -(y_body_to_feet_percent*Y_BODY_TO_FEET + com_y_offset), Z_ROBOT_WALK]

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
    final_l_foot_pos    = np.array([-0.02, Y_BODY_TO_FEET*1.1, kick_height])

    r_leg_abs_pos = np.full((len(T), 3), initial_r_foot_pos)
    l_leg_abs_pos, T = trajectory_planner.get_polynomial_trajectory_multi_dof(initial_l_foot_pos, final_l_foot_pos, time_step=SERVO_SAMPLE_TIME)

    r_leg_relative_pos  = r_leg_abs_pos - P_CoM
    l_leg_relative_pos  = l_leg_abs_pos - P_CoM

    left_q = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, P_CoM[-1], final_l_foot_pos

def calculate_cartesian_do_kick(p_start, final_foot_pos, ik_client_left, ik_client_right):
    final_l_foot_pos = [kick_length*5, Y_BODY_TO_FEET, kick_height*0.3]
    p_end = p_start
    P_CoM , T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_end, time_step=SERVO_SAMPLE_TIME)

    initial_r_foot_pos = np.array([0, -Y_BODY_TO_FEET, 0])
    initial_l_foot_pos = final_foot_pos

    #final_r_foot_pos = initial_r_foot_pos
    r_leg_abs_pos = np.full((len(T), 3), initial_r_foot_pos)
    l_leg_abs_pos , T = trajectory_planner.get_polynomial_trajectory_multi_dof(initial_l_foot_pos, final_l_foot_pos, time_step=SERVO_SAMPLE_TIME)

    r_leg_relative_pos  = r_leg_abs_pos - P_CoM
    l_leg_relative_pos  = l_leg_abs_pos - P_CoM

    left_q  = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, P_CoM[-1]

def executeTrajectories(left_foot_q, right_foot_q, rate: rospy.Rate, legs_publisher: rospy.Publisher):
    for left, right in zip(left_foot_q, right_foot_q):
        np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
        legs_msg = Float32MultiArray()
        legs_msg.data = [*left] + [*right]
        print(legs_msg.data)
        legs_publisher.publish(legs_msg)
        rate.sleep()

def main(args = None):
    global Y_BODY_TO_FEET, Z_ROBOT_WALK, Z_ROBOT_STATIC
    global kick_length, kick_height, com_x_offset, com_y_offset
    global first_left_q, first_right_q, second_left_q, second_right_q, rate, rate2, third_left_q, third_right_q, pub_legs_goal
    rospy.init_node('step_test_node')
    service_execute     = rospy.Service("execute_kick_service", Lateral, handle_execute_kick)
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

    time.sleep(5)

    arms_msg = Float32MultiArray()
    arms_msg.data = [0.0, 0.3, 0.0, 0.0, -0.3, 0.0]
    arms_goal_pose.publish(arms_msg)

    zero_msg = Float32MultiArray()
    zero_msg.data = [0.0 for i in range(12)]
    pub_legs_goal.publish(zero_msg)

    rate = rospy.Rate(40)
    rate2 = rospy.Rate(20)

    first_left_q, first_right_q, last_p_com = calculate_cartesian_right_start_pose(1.0, left_leg_client, right_leg_client)
    second_left_q, second_right_q, last_p_com , final_foot_pos = calculate_cartesian_left_first_step_pose(last_p_com, left_leg_client, right_leg_client)
    third_left_q, third_right_q, last_p_com = calculate_cartesian_do_kick(last_p_com, final_foot_pos, left_leg_client, right_leg_client)

    time.sleep(5)

    zero_msg = Float32MultiArray()
    zero_msg.data = [0.0 for i in range(12)]
    pub_legs_goal.publish(zero_msg)

if __name__ == "__main__":
    main()
