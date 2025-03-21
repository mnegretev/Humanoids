#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
        print(f"Executing kick {req.iterations}")
        executeTrajectories(first_left_q,  first_right_q,  rate2, pub_legs_goal)
        executeTrajectories(second_left_q, second_right_q, rate, pub_legs_goal)
        executeTrajectories(third_left_q[-2:],  third_right_q[-2:],  rate, pub_legs_goal)
        succes=LateralResponse()
        succes.succes=True
        return succes
    except Exception as e:
        print(f"cant go lateral {e}")
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

def calculate_cartesian_right_start_pose_kick(y_body_to_feet_percent, ik_client_left, ik_client_right):
    p_start = [0 + com_x_offset, 0, Z_ROBOT_STATIC]
    p_end = [0 + com_x_offset, -(y_body_to_feet_percent*Y_BODY_TO_FEET + com_y_offset), Z_ROBOT_WALK]

    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_end, time_step=SERVO_SAMPLE_TIME)
    
    left_leg_relative_pos   = [0,  Y_BODY_TO_FEET, 0] - P_CoM
    right_leg_relative_pos  = [0, -Y_BODY_TO_FEET, 0] - P_CoM

    left_q  = calculate_ik(left_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(right_leg_relative_pos, ik_client_right)

    return left_q, right_q, P_CoM[-1]

def calculate_cartesian_left_raise_foot_pose(p_start, ik_client_left, ik_client_right):
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


G = 9.81 # [m/s^2]

Y_BODY_TO_FEET  = 0.056 #Mínimo valor =0.056 #Máximo valor =0.125#= 0.09
Z_ROBOT_WALK    = 0.50
Z_ROBOT_STATIC  = 0.576 #Máximo valor = 0.576 # m

stepHeight  = 0.1
STEP_LENGTH = 0.1 # [m]
ROBOT_VEL_X = 0.1 # [m]

com_x_offset = 0.02 #original=0.02
com_y_offset = 0.001 # 

# Tiempo de muestreo para resolver la ecuación diferencial del LIPM (debe ser pequeño)
LIPM_SAMPLE_TIME = 0.0001 # [s]

# Tiempo de muestreo maximo para escribir a los servomotores
SERVO_SAMPLE_TIME = 0.025 # [s]


def calculate_cartesian_right_start_pose(y_body_to_feet_percent, ik_client_left, ik_client_right):
    p_start = [0 + com_x_offset, 0, Z_ROBOT_STATIC]
    p_end = [0 + com_x_offset, -y_body_to_feet_percent*Y_BODY_TO_FEET, Z_ROBOT_WALK]

    
    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_end, time_step=SERVO_SAMPLE_TIME)
    
    left_leg_relative_pos = [0, Y_BODY_TO_FEET, 0] - P_CoM
    right_leg_relative_pos =  [0, -Y_BODY_TO_FEET, 0] - P_CoM

    left_q  = calculate_ik(left_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(right_leg_relative_pos, ik_client_right)

    return left_q, right_q, P_CoM[-1]

def calculate_cartesian_left_first_step_pose(p_start, p_end, ik_client_left, ik_client_right):
    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_end, time_step=SERVO_SAMPLE_TIME)
    
    initial_r_foot_pos  = np.array([0, -Y_BODY_TO_FEET, 0])
    initial_l_foot_pos  = np.array([0,  Y_BODY_TO_FEET, 0])

    #final_r_foot_pos    = initial_r_foot_pos
    final_l_foot_pos    = np.array([0, Y_BODY_TO_FEET*2, 0])

    r_leg_abs_pos = np.full((len(T), 3), initial_r_foot_pos)
    l_leg_abs_pos = getFootSwingTraj(initial_l_foot_pos, final_l_foot_pos, stepHeight, T)

    r_leg_relative_pos  = r_leg_abs_pos - P_CoM
    l_leg_relative_pos  = l_leg_abs_pos - P_CoM

    left_q = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, l_leg_abs_pos

def calculate_cartesian_right_second_step(p_start, p_end, ik_client_left, ik_client_right):
    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_end, time_step=SERVO_SAMPLE_TIME)

    initial_r_foot_pos = np.array([0, -Y_BODY_TO_FEET, 0])
    initial_l_foot_pos = np.array([0,  Y_BODY_TO_FEET, 0])

    final_l_foot_pos   = np.array([0,  Y_BODY_TO_FEET, 0])

    r_leg_abs_pos = getFootSwingTraj(initial_l_foot_pos, final_l_foot_pos, stepHeight, T)


def findInitialConditions(STEP_LENGTH, ROBOT_VEL_X, y_0, zModel, G):
    #Desired midstance and state
    s = math.sqrt(zModel/G)
    x_mid = 0
    # using relationship between final body state and initial body state,
    # we can find time it will take to re
    #Corresponding orbital energy is
    E = -G/(2*zModel) * (x_mid**2) + 0.5*(ROBOT_VEL_X**2)
    x_0 = -STEP_LENGTH/2
    #Finding dy0 from midstance energy level
    # using relationship between final body state and initial body state,
    # we can find time it will take to re
    dx0 = math.sqrt(2*(E + G/(2*zModel) * (x_0**2)))
    # using relationship between final body state and initial body state,
    # we can find time it will take to reach midstance given final velocity
    # (dy = ROBOT_VEL_Y) and final position (which is y = 0 at midstance)
    singlesupport_t = 2*math.asinh( STEP_LENGTH/2/(s*ROBOT_VEL_X) ) * s
    # print("singlesupportTIme is ", singlesupport_t)

    tf = singlesupport_t/2

    dy0 = -y_0/s * math.sinh(tf/s) / math.cosh(tf/s)

    return [dy0, x_0, dx0, singlesupport_t]

def getFootSwingTraj(initial_foot_position, final_foot_position, swing_height, timeVector):
    x_0 = initial_foot_position[0]
    x_1 = final_foot_position[0]

    y_0 = initial_foot_position[1]
    y_1 = final_foot_position[1]
    
    h = x_0 + (x_1 - x_0)/2
    k = swing_height
    a = -k/((x_0-h)**2)
    m_x = (x_1 - x_0)/(timeVector[-1] - timeVector[0])
    x_t = lambda t: x_0 + m_x*t
    m_y = (y_1 - y_0)/(timeVector[-1] - timeVector[0])
    y_t = lambda t: y_0 + m_y*t
    z = lambda x: a*((x-h)**2) + k

    swingFootTrajectory = np.array([[0, 0, 0]])

    for i in timeVector:
        aux = np.array([[x_t(i), y_t(i), z(x_t(i))]])
        swingFootTrajectory = np.concatenate((swingFootTrajectory,aux), axis=0)
    swingFootTrajectory = np.delete(swingFootTrajectory, 0, axis=0)
    return swingFootTrajectory

def executeTrajectories(left_foot_q, right_foot_q, rate: rospy.Rate, legs_publisher: rospy.Publisher):
    for left, right in zip(left_foot_q, right_foot_q):
        legs_msg = Float32MultiArray()
        legs_msg.data = [*left] + [*right]
        print(legs_msg.data)
        legs_publisher.publish(legs_msg)
        rate.sleep()

def handle_execute_lateral(req):
    print(f"Executing lateral service{req.iterations}")
    try:
        for i in range (0,req.iterations):
            executeTrajectories(first_right_q_lateral,  first_left_q_lateral,  rate, pub_legs_goal)
            executeTrajectories(second_right_q_lateral, second_left_q_lateral, rate, pub_legs_goal)
        executeTrajectories(first_right_q_lateral,  first_left_q_lateral,  rate, pub_legs_goal)
        succes=LateralResponse()
        succes.succes=True
        return succes
    except Exception as e:
        print(f"cant go lateral {e}")
        succes = LateralResponse()
        succes.succes=False

        return succes

def main(args = None):
    global first_left_q_lateral, first_right_q_lateral, rate, pub_legs_goal, second_left_q_lateral, second_right_q_lateral
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

    first_left_q, first_right_q, last_p_com = calculate_cartesian_right_start_pose_kick(1.0, left_leg_client, right_leg_client)
    second_left_q, second_right_q, last_p_com , final_foot_pos = calculate_cartesian_left_raise_foot_pose(last_p_com, left_leg_client, right_leg_client)
    third_left_q, third_right_q, last_p_com = calculate_cartesian_do_kick(last_p_com, final_foot_pos, left_leg_client, right_leg_client)

    #time.sleep(5)

    zero_msg = Float32MultiArray()
    zero_msg.data = [0.0 for i in range(12)]
    pub_legs_goal.publish(zero_msg)
    service_execute     = rospy.Service("execute_lateral_service", Lateral, handle_execute_lateral)
    #rospy.get_param("/gait/")
    arms_goal_pose      = rospy.Publisher("/hardware/arms_goal_pose", Float32MultiArray , queue_size=1)
    pub_legs_goal       = rospy.Publisher("/hardware/legs_goal_pose", Float32MultiArray, queue_size=1)
    rate = rospy.Rate(40)

    first_left_q_lateral, first_right_q_lateral, last_p_com = calculate_cartesian_right_start_pose(0.8, left_leg_client, right_leg_client)

    p_com_opposite = [last_p_com[0], 0, last_p_com[2]]

    second_left_q_lateral, second_right_q_lateral, left_leg_final_pos = calculate_cartesian_left_first_step_pose(last_p_com, p_com_opposite, left_leg_client, right_leg_client)

    arms_msg = Float32MultiArray()
    arms_msg.data = [0.0, 0.3, 0.0, 0.0, -0.3, 0.0]
    arms_goal_pose.publish(arms_msg)
    rospy.spin()
    


if __name__ == "__main__":
    main()
