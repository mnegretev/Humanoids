#!/usr/bin/env python
import numpy as np
from scipy import signal, interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import os

#ROS
import rospy
from std_msgs.msg import String, Float32MultiArray
from ctrl_msgs.srv import CalculateIK, CalculateIKRequest
from trajectory_planner import trajectory_planner

# Y_BODY_TO_FEET  = 0.0555 # [m]
Y_BODY_TO_FEET  = 0.075
# Z_ROBOT_WALK    = 0.55 # m
Z_ROBOT_WALK    = 0.50
Z_ROBOT_STATIC= 0.57 # m

stepHeight = 0.1
STEP_LENGTH = 0.1 # [m]
ROBOT_VEL_X = 0.1 # [m]

com_x_offset = 0.02
com_y_offset = 0.025

# Tiempo de muestreo para resolver la ecuación diferencial del LIPM (debe ser pequeño)
LIPM_SAMPLE_TIME = 0.0001 # [s]

# Tiempo de muestreo maximo para escribir a los servomotores
SERVO_SAMPLE_TIME = 0.025 # [s]

def calculate_ik(P, service_client):
    joint_values = np.zeros((len(P),6))
    for i, vector in enumerate(P):
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



def get_twist_trajectory_start_pose(duration, stepHeight, ik_client_left, ik_client_right):
    com_start   = [0 + com_x_offset, 0, Z_ROBOT_STATIC]
    com_end     =   [0 + com_x_offset, -(Y_BODY_TO_FEET + com_y_offset), Z_ROBOT_WALK]

    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(com_start, com_end, duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    left_leg_relative_pos   =   [0, Y_BODY_TO_FEET, 0]  - P_CoM
    right_leg_relative_pos  =   [0, -Y_BODY_TO_FEET, 0] - P_CoM

    r_leg_pose = np.concatenate((right_leg_relative_pos, np.full((len(T), 3), [0,0,0])), axis=1)
    l_leg_pose = np.concatenate((left_leg_relative_pos,  np.full((len(T), 3), [0,0,0])), axis=1)

    left_q  = calculate_ik(l_leg_pose, ik_client_left)
    right_q = calculate_ik(r_leg_pose, ik_client_right)

    return left_q, right_q, P_CoM[-1]

def get_twist_trajectory_left_first_step(p_start, duration, twist_angle, ik_client_left, ik_client_right):
    com_start_pose  = p_start + [0,0,0]
    com_end_pose    = com_start_pose

    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(com_start_pose, com_end_pose, duration=duration, time_step=SERVO_SAMPLE_TIME)

    initial_r_foot_pos  = np.array([0, -Y_BODY_TO_FEET, 0])
    initial_l_foot_pos  = np.array([0,  Y_BODY_TO_FEET, 0])
    
    initial_l_foot_orientation  = np.array([0,0,0])
    final_l_foot_orientation    = np.array([0,0,math.pi/6])
    
    O, T = trajectory_planner.get_polynomial_trajectory_multi_dof(initial_l_foot_orientation, final_l_foot_orientation, duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    #final_r_foot_pos    = initial_r_foot_pos
    final_l_foot_pos = np.array([0,  Y_BODY_TO_FEET*2.5, 0])
    
    r_leg_abs_pos = np.full((len(T), 3), initial_r_foot_pos)
    l_leg_abs_pos = getFootSwingTraj(initial_l_foot_pos, final_l_foot_pos, stepHeight, T)
    #print(r_leg_abs_pos)

    r_leg_relative_pos  = r_leg_abs_pos - P_CoM
    l_leg_relative_pos  = l_leg_abs_pos - P_CoM

    r_leg_pose = np.concatenate((r_leg_relative_pos, np.full((len(T), 3), [0,0,0])), axis=1)
    l_leg_pose = np.concatenate((l_leg_relative_pos, O), axis=1)

    left_q  = calculate_ik(l_leg_pose, ik_client_left)
    right_q = calculate_ik(r_leg_pose, ik_client_right)

    return left_q, right_q, P_CoM[-1]

def get_twist_trajectory_move_com_left(com_start, duration,  ik_client_left, ik_client_right):
    
    com_start_pose = np.concatenate((com_start, [[0,0,0]]), axis=None)
    com_end_pose = [0 + com_x_offset, Y_BODY_TO_FEET*2.5, Z_ROBOT_WALK, 0, 0, 0]

    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(com_start_pose, com_end_pose, duration=duration, time_step=SERVO_SAMPLE_TIME)

    r_foot_pose = np.full((len(T), 6), [0, -Y_BODY_TO_FEET,  0, 0, 0, 0])
    l_foot_pose = np.full((len(T), 6), [0, Y_BODY_TO_FEET*2.5, 0, 0, 0, math.pi/6])

    r_leg_relative_pose = r_foot_pose - P_CoM
    l_leg_relative_pose = l_foot_pose - P_CoM

    left_q  = calculate_ik(l_leg_relative_pose, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pose, ik_client_right)

    return left_q, right_q, P_CoM[-1]

def get_twist_trajectory_right_third_step(p_start, duration, ik_client_left, ik_client_right):
    com_start_pose = p_start
    com_end_pose    = com_start_pose

    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(com_start_pose, com_end_pose, duration=duration, time_step=SERVO_SAMPLE_TIME)

    initial_r_foot_pos  = np.array([0, -Y_BODY_TO_FEET, 0])
    initial_l_foot_pos  = np.array([0,  Y_BODY_TO_FEET*2.5, 0])
    
    initial_l_foot_orientation  = np.array([0,0,math.pi/6])
    final_l_foot_orientation    = np.array([0,0,0])
    
    O, T = trajectory_planner.get_polynomial_trajectory_multi_dof(initial_l_foot_orientation, final_l_foot_orientation, duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    final_r_foot_pos    = np.array([0,  Y_BODY_TO_FEET, 0])
    #final_l_foot_pos    = inital_l_foos_pos
    
    r_leg_abs_pos = getFootSwingTraj(initial_r_foot_pos, final_r_foot_pos, stepHeight, T)
    l_leg_abs_pos = np.full((len(T), 3), initial_l_foot_pos)
    #print(r_leg_abs_pos)

    r_leg_pose = np.concatenate((r_leg_abs_pos, np.full((len(T), 3), [0,0,0])), axis=1)
    l_leg_pose = np.concatenate((l_leg_abs_pos, O), axis=1)

    r_leg_relative_pos  = r_leg_pose - P_CoM
    l_leg_relative_pos  = l_leg_pose - P_CoM

    left_q  = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

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
        print(aux)
    
    return swingFootTrajectory

def main(args = None):
    rospy.init_node('step_test_node')
    
    trajectory_dir = rospy.get_param("~trajectory_dir")
    if not os.path.isdir(trajectory_dir):
        raise Exception(f"File directory not found: {trajectory_dir}")
    
    right_leg_client    = rospy.ServiceProxy('/control/ik_leg_right', CalculateIK)
    left_leg_client     = rospy.ServiceProxy('/control/ik_leg_left', CalculateIK)

    left_q, right_q, last_p_com = get_twist_trajectory_start_pose(duration=1, stepHeight=Z_ROBOT_WALK, ik_client_left=left_leg_client, ik_client_right=right_leg_client)
    np.savez(os.path.join(trajectory_dir, "twist_right_start_pose"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)

    left_q, right_q, last_p_com = get_twist_trajectory_left_first_step(last_p_com, duration=2, twist_angle=math.pi/4, ik_client_left=left_leg_client, ik_client_right=right_leg_client)
    np.savez(os.path.join(trajectory_dir, "twist_left_first_step"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)

    left_q, right_q, last_p_com = get_twist_trajectory_move_com_left(last_p_com, duration=1, ik_client_left=left_leg_client, ik_client_right=right_leg_client)
    np.savez(os.path.join(trajectory_dir, "twist_move_com_left"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)

    left_q, right_q, last_p_com = get_twist_trajectory_right_third_step(last_p_com, duration=2, ik_client_left=left_leg_client, ik_client_right=right_leg_client)
    np.savez(os.path.join(trajectory_dir, "twist_right_third_step"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)


if __name__ == "__main__":
    main()