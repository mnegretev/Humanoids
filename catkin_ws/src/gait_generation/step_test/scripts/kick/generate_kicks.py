#!/usr/bin/env python
import numpy as np
import math
import os

#ROS
import rospy
from std_msgs.msg import String, Float32MultiArray
from ctrl_msgs.srv import CalculateIK, CalculateIKRequest
from trajectory_planner import trajectory_planner

# Y_BODY_TO_FEET_RIGHT  = 0.0555 # [m]
Y_BODY_TO_FEET  = 0.055 #Mínimo valor =0.056 #Máximo valor =0.125#= 0.09
# Z_ROBOT_WALK  = 0.55 # m
Z_ROBOT_WALK    = 0.54
Z_ROBOT_STATIC  = 0.575 # m
Z_ROBOT_SIT     = 0.28

Y_BODY_TO_FEET_RIGHT  = 0.09 #Mínimo valor =0.056 #Máximo valor =0.125#= 0.09
# Z_ROBOT_WALK  = 0.55 # m
Z_ROBOT_WALK_RIGHT    = 0.53
Z_ROBOT_STATIC_RIGHT  = 0.575 # m

stepHeight = 0.14
kickLength = 0.14

# Tiempo de muestreo maximo para escribir a los servomotores
SERVO_SAMPLE_TIME = 0.025 # [s]

def calculate_ik(P, service_client):
    joint_values = np.zeros((len(P),6))
    for i, vector in enumerate(P):
        print(vector)
        req = CalculateIKRequest(x = vector[0], y = vector[1], z = vector[2],
                                 roll = vector[3], pitch = vector[4], yaw = vector[5])
        response = service_client(req)
        if len(response.joint_values) == 6:
            aux = np.array([list(response.joint_values)])
            joint_values[i] = aux
        else:
            print("Could not calculate inverse kinematics for pose {vector}")
            raise ValueError("Error calculating inverse kinematics for point {vector}")
    return joint_values

# RIGHT SIDE ------------------------------------------------------------------------------

def get_kick_standup_trajectory(duration, ik_client_left, ik_client_right):
    com_start   =   [0, 0, Z_ROBOT_SIT]
    com_end     =   [0, 0, Z_ROBOT_STATIC]

    P_CoM, T    = trajectory_planner.get_polynomial_trajectory_multi_dof(com_start, com_end, duration=duration, time_step=SERVO_SAMPLE_TIME)

    left_leg_relative_pos   =   [0, Y_BODY_TO_FEET_RIGHT, 0]  - P_CoM
    right_leg_relative_pos  =   [0, -Y_BODY_TO_FEET_RIGHT, 0] - P_CoM

    r_leg_pose = np.concatenate((right_leg_relative_pos, np.full((len(T), 3), [0,0,0])), axis=1)
    l_leg_pose = np.concatenate((left_leg_relative_pos,  np.full((len(T), 3), [0,0,0])), axis=1)

    left_q  = calculate_ik(l_leg_pose, ik_client_left)
    right_q = calculate_ik(r_leg_pose, ik_client_right)

    return left_q, right_q, P_CoM[-1]

def get_kick_right_trajectory_start_pose(duration, stepHeight, ik_client_left, ik_client_right):
    com_start   =   [0, 0, Z_ROBOT_STATIC]
    com_end     =   [0.02 , Y_BODY_TO_FEET_RIGHT + 0.01, Z_ROBOT_WALK]

    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(com_start, com_end, duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    left_leg_relative_pos   =   [0, Y_BODY_TO_FEET, 0]  - P_CoM
    right_leg_relative_pos  =   [0, -Y_BODY_TO_FEET, 0] - P_CoM

    r_leg_pose = np.concatenate((right_leg_relative_pos, np.full((len(T), 3), [0,0,0])), axis=1)
    l_leg_pose = np.concatenate((left_leg_relative_pos,  np.full((len(T), 3), [0,0,0])), axis=1)

    left_q  = calculate_ik(l_leg_pose, ik_client_left)
    right_q = calculate_ik(r_leg_pose, ik_client_right)

    return left_q, right_q, P_CoM[-1]

def get_kick_right_trajectory_raise_foot(p_start, duration, ik_client_left, ik_client_right):
    com_start_pose  = p_start
    com_end_pose    = com_start_pose

    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(com_start_pose, com_end_pose, duration=duration, time_step=SERVO_SAMPLE_TIME)

    initial_r_foot_pos  = np.array([0, -Y_BODY_TO_FEET, 0])
    initial_l_foot_pos  = np.array([0,  Y_BODY_TO_FEET, 0])

    #final_r_foot_pos    = initial_r_foot_pos
    final_r_foot_pos = np.array([-kickLength,  -Y_BODY_TO_FEET_RIGHT*1.3, stepHeight])
    
    r_leg_abs_pos, T = trajectory_planner.get_polynomial_trajectory_multi_dof(initial_r_foot_pos, final_r_foot_pos, duration=duration, time_step=SERVO_SAMPLE_TIME)
    l_leg_abs_pos = np.full((len(T), 3), initial_l_foot_pos)
    
    r_leg_relative_pos  = r_leg_abs_pos - P_CoM
    l_leg_relative_pos  = l_leg_abs_pos - P_CoM

    r_leg_pose = np.concatenate((r_leg_relative_pos, np.full((len(T), 3), [0,0,0])), axis=1)
    l_leg_pose = np.concatenate((l_leg_relative_pos, np.full((len(T), 3), [0,0,0])), axis=1)
    
    left_q  = calculate_ik(l_leg_pose, ik_client_left)
    right_q = calculate_ik(r_leg_pose, ik_client_right)

    return left_q, right_q, P_CoM[-1]

def get_kick_right_trajectory_do_kick(com_start, duration,  ik_client_left, ik_client_right):
    
    com_start_pose = com_start + [0,0,0]
    com_end_pose = [0, Y_BODY_TO_FEET_RIGHT, Z_ROBOT_WALK, 0, 0, 0]

    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(com_start_pose, com_end_pose, duration=duration, time_step=SERVO_SAMPLE_TIME)

    initial_r_foot_pos  = np.array([-kickLength, -Y_BODY_TO_FEET, stepHeight])
    initial_l_foot_pos  = np.array([0,  Y_BODY_TO_FEET, 0])

    final_r_foot_pos = np.array([kickLength,  -Y_BODY_TO_FEET_RIGHT, stepHeight + 0.02])
    #final_l_foot_pos    = initial_l_foot_pos

    r_foot_pose, _ = trajectory_planner.get_polynomial_trajectory_multi_dof(initial_r_foot_pos, final_r_foot_pos, duration=duration, time_step=SERVO_SAMPLE_TIME)
    l_foot_pose = np.full((len(T), 3), initial_l_foot_pos)

    r_leg_pose = r_foot_pose - P_CoM
    l_leg_pose = l_foot_pose - P_CoM

    r_leg_relative_pose = np.concatenate((r_leg_pose, np.full((len(T), 3), [0,0,0])), axis=1)
    l_leg_relative_pose = np.concatenate((l_leg_pose, np.full((len(T), 3), [0,0,0])), axis=1)

    left_q  = calculate_ik(l_leg_relative_pose, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pose, ik_client_right)

    return left_q, right_q, P_CoM[-1]

def get_kick_right_trajectory_final_stop(p_start, duration, ik_client_left, ik_client_right):
    com_start_pose  = [p_start[0], p_start[1], p_start[2]]
    com_end_pose    = [0, 0, Z_ROBOT_WALK - 0.03]

    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(com_start_pose, com_end_pose, duration=duration, time_step=SERVO_SAMPLE_TIME)

    initial_r_foot_pos  = np.array([kickLength,  -Y_BODY_TO_FEET_RIGHT, stepHeight])
    final_r_foot_pos    = np.array([0, -Y_BODY_TO_FEET_RIGHT, 0])

    r_foot_pose, _ = trajectory_planner.get_polynomial_trajectory_multi_dof(initial_r_foot_pos, final_r_foot_pos, duration=duration, time_step=SERVO_SAMPLE_TIME)
    l_foot_pose    = np.full((len(T), 3), [0, Y_BODY_TO_FEET, 0])

    left_leg_relative_pos   =   l_foot_pose - P_CoM
    right_leg_relative_pos  =   r_foot_pose - P_CoM

    r_leg_pose = np.concatenate((right_leg_relative_pos, np.full((len(T), 3), [0,0,0])), axis=1)
    l_leg_pose = np.concatenate((left_leg_relative_pos,  np.full((len(T), 3), [0,0,0])), axis=1)

    left_q  = calculate_ik(l_leg_pose, ik_client_left)
    right_q = calculate_ik(r_leg_pose, ik_client_right)

    return left_q, right_q, P_CoM[-1]

def getFootSwingTraj(initial_foot_position, final_foot_position, swing_height, timeVector):
    # Time
    t_0 = timeVector[0]
    t_f = timeVector[-1]
    delta_t = t_f - t_0

    # Initial and final components
    x_0, y_0, z_0 = initial_foot_position
    x_f, y_f, z_f = final_foot_position

    # Parametric X function (linear)
    m_x = (x_f - x_0)/delta_t
    x = lambda t: x_0 + m_x*t

    # Parametric Y function (linear)
    m_y = (y_f - y_0)/delta_t
    y = lambda t: y_0 + m_y*t

    # Parametric Z function (quadratic)
    h = delta_t/2
    k = swing_height
    a = -k/(h**2)
    z = lambda t: a*((t-h)**2) + k

    swingFootTrajectory = np.zeros((len(timeVector),3))

    for i, t in enumerate(timeVector):
        aux = np.array([[x(t), y(t), z(t)]])
        swingFootTrajectory[i] = aux
    
    return swingFootTrajectory

def main(args = None):
    rospy.init_node('step_test_node')
    
    # trajectory_dir_left =  rospy.get_param("~trajectory_dir_left")
    trajectory_dir_right = rospy.get_param("~trajectory_dir_right")

    # if not os.path.isdir(trajectory_dir_left):
    #     raise Exception(f"File directory not found: {trajectory_dir_left}")
    
    if not os.path.isdir(trajectory_dir_right):
        raise Exception(f"File directory not found: {trajectory_dir_right}")
    
    right_leg_client    = rospy.ServiceProxy('/control/ik_leg_right', CalculateIK)
    left_leg_client     = rospy.ServiceProxy('/control/ik_leg_left', CalculateIK)

    # RIGHT SIDE
    #left_q, right_q, last_p_com = get_kick_standup_trajectory(duration=1, ik_client_left=left_leg_client, ik_client_right=right_leg_client)
    #np.savez(os.path.join(trajectory_dir_right, "standup"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)
    #print("\n Done with standup\n")

    left_q, right_q, last_p_com = get_kick_right_trajectory_start_pose(duration=1, stepHeight=Z_ROBOT_WALK, ik_client_left=left_leg_client, ik_client_right=right_leg_client)
    np.savez(os.path.join(trajectory_dir_right, "kick_right_start_pose"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)
    print("\n Done with kick_right_start_pose\n")

    left_q, right_q, last_p_com = get_kick_right_trajectory_raise_foot(last_p_com, duration=1, ik_client_left=left_leg_client, ik_client_right=right_leg_client)
    np.savez(os.path.join(trajectory_dir_right, "kick_right_raise_foot"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)
    print("\n Done with kick_right_raise_foot")

    left_q, right_q, last_p_com = get_kick_right_trajectory_do_kick(last_p_com, duration=1,  ik_client_left=left_leg_client, ik_client_right=right_leg_client)
    np.savez(os.path.join(trajectory_dir_right, "kick_right_do_kick"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)
    print("\n Done with kick_right_do_kick")

    left_q, right_q, _ = get_kick_right_trajectory_final_stop(last_p_com, duration=1, ik_client_left=left_leg_client, ik_client_right=right_leg_client)
    np.savez(os.path.join(trajectory_dir_right, "kick_right_final_stop"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)
    print("\n Done with kick_right_final_stop")



if __name__ == "__main__":
    main()
