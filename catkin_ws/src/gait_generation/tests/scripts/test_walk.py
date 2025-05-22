#!/usr/bin/python3
import numpy as np
from scipy import signal, interpolate
#import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import os
import time

#ROS
import rospy
from std_msgs.msg import String, Float32MultiArray
from ctrl_msgs.srv import CalculateIK, CalculateIKRequest, CalculateDK, CalculateDKRequest
from trajectory_planner import trajectory_planner
from manip_msgs.srv import *

G = 9.81 # [m/s^2]

Y_BODY_TO_FEET  = None 
Z_ROBOT_WALK    = None
Z_ROBOT_STATIC  = None
STEP_HEIGHT     = None
STEP_LENGTH     = None
ROBOT_VEL_X     = None
COM_X_OFFSET    = None
COM_Y_OFFSET    = None
LIPM_SAMPLING   = None
SERVO_RATE      = None

def calculate_ik(P, service_client):
    failed_counts = 0
    joint_values = np.zeros((len(P),6))
    for i, vector in enumerate(P):
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
            if (failed_counts > len(joint_values)*0.9):
                raise ValueError(f"{failed_counts} failed calculations. Aborting")
            print(f"Could not calculate inverse kinematics for pose {vector}")
            joint_values[i] = joint_values[i-1]
    return joint_values

def calculate_cartesian_right_start_pose(duration, y_body_to_feet_percent, ik_client_left, ik_client_right):
    p_start = [0 + COM_X_OFFSET, 0, Z_ROBOT_STATIC]
    p_end = [0 + COM_X_OFFSET, -(y_body_to_feet_percent*Y_BODY_TO_FEET) + COM_Y_OFFSET, Z_ROBOT_WALK]

    
    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_end, duration=duration, time_step=1/SERVO_RATE)
    
    left_leg_relative_pos = [0, Y_BODY_TO_FEET, 0] - P_CoM
    right_leg_relative_pos =  [0, -Y_BODY_TO_FEET, 0] - P_CoM

    left_q = calculate_ik(left_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(right_leg_relative_pos, ik_client_right)

    return left_q, right_q, P_CoM[-1],

def calculate_cartesian_right_end_pose(duration, y_body_to_feet_percent, ik_client_left, ik_client_right):
    p_end = [0 + COM_X_OFFSET, 0, Z_ROBOT_STATIC]
    p_start = [0 + COM_X_OFFSET, -(y_body_to_feet_percent*Y_BODY_TO_FEET) + COM_Y_OFFSET, Z_ROBOT_WALK]

    
    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_end, duration=duration, time_step=1/SERVO_RATE)
    
    left_leg_relative_pos = [0, Y_BODY_TO_FEET, 0] - P_CoM
    right_leg_relative_pos =  [0, -Y_BODY_TO_FEET, 0] - P_CoM

    left_q = calculate_ik(left_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(right_leg_relative_pos, ik_client_right)

    return left_q, right_q, P_CoM[-1],

def calculate_cartesian_left_half_step_pose(duration, p_start, p_end, ik_client_left, ik_client_right):
    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_end, duration=duration, time_step=1/SERVO_RATE)
    
    initial_r_foot_pos  = np.array([0, -Y_BODY_TO_FEET, 0])
    initial_l_foot_pos  = np.array([0,  Y_BODY_TO_FEET, 0])

    #final_r_foot_pos    = initial_r_foot_pos
    final_l_foot_pos    = np.array([STEP_LENGTH/2, Y_BODY_TO_FEET, 0])

    r_leg_abs_pos = np.full((len(T), 3), initial_r_foot_pos)
    l_leg_abs_pos = getFootSwingTraj(initial_l_foot_pos, final_l_foot_pos, STEP_HEIGHT, T)

    r_leg_relative_pos  = r_leg_abs_pos - P_CoM
    l_leg_relative_pos  = l_leg_abs_pos - P_CoM

    left_q = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, l_leg_abs_pos

def calculate_cartesian_right_step_pose(initial_l_foot_pos, ik_client_left, ik_client_right):
    global final_posr_right
    #Left foot is the support foot
    y_0 = -Y_BODY_TO_FEET

    [dy0, x_0, dx0, single_support_time] = findInitialConditions(STEP_LENGTH, ROBOT_VEL_X, y_0, Z_ROBOT_WALK, G)

    state0 = [x_0, dx0, y_0, dy0]
    steptimeVector = np.linspace(0, single_support_time, math.floor(single_support_time*LIPM_SAMPLING))

    nSteps = len(steptimeVector)
    states = np.array([state0])
    #Solve differential equation (sample time must be as small as possible)
    for i in range(0, nSteps-1):
        dx_next = states[i][1] + ((states[i][0])*G/Z_ROBOT_WALK) / LIPM_SAMPLING
        x_next  = states[i][0] + states[i][1]                    / LIPM_SAMPLING
        dy_next = states[i][3] + ((states[i][2])*G/Z_ROBOT_WALK) / LIPM_SAMPLING
        y_next  = states[i][2] + states[i][3]                    / LIPM_SAMPLING
        states = np.concatenate((states, [[x_next, dx_next, y_next, dy_next]]), axis=0)
    
    body_position = zip(np.add(states[:,0] + COM_X_OFFSET, initial_l_foot_pos[0]), np.add(states[:,2],initial_l_foot_pos[1]), [Z_ROBOT_WALK for i in states])

    P_CoM = np.array([list(i) for i in list(body_position)])

    initial_r_foot_pos  = [0, -Y_BODY_TO_FEET, 0]
    final_r_foot_pos    = [STEP_LENGTH, -Y_BODY_TO_FEET,0]
    final_posr_right = final_r_foot_pos
    l_foot_abs_pos = np.full((len(steptimeVector), 3), initial_l_foot_pos)
    r_foot_abs_pos = getFootSwingTraj(initial_r_foot_pos, final_r_foot_pos, STEP_HEIGHT, steptimeVector)

    l_leg_relative_pos  = l_foot_abs_pos - P_CoM
    r_leg_relative_pos  = r_foot_abs_pos - P_CoM

    l_leg_relative_pos = l_leg_relative_pos[::int(LIPM_SAMPLING/SERVO_RATE)]
    r_leg_relative_pos = r_leg_relative_pos[::int(LIPM_SAMPLING/SERVO_RATE)]

    left_q = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, r_foot_abs_pos

def calculate_cartesian_left_step_pose(initial_r_foot_pos, ik_client_left, ik_client_right):
    #Left foot is the support foot
    global final_posl_left
    y_0 = Y_BODY_TO_FEET

    [dy0, x_0, dx0, single_support_time] = findInitialConditions(STEP_LENGTH, ROBOT_VEL_X, y_0, Z_ROBOT_WALK, G)

    state0 = [x_0, dx0, y_0, dy0]
    steptimeVector = np.linspace(0, single_support_time, math.floor(single_support_time*LIPM_SAMPLING))

    nSteps = len(steptimeVector)
    states = np.array([state0])
    #Solve differential equation (sample time must be as small as possible)
    for i in range(0, nSteps-1):
        dx_next = states[i][1] + 1/LIPM_SAMPLING * ((states[i][0])*G/Z_ROBOT_WALK)
        x_next  = states[i][0] + 1/LIPM_SAMPLING * states[i][1]
        dy_next = states[i][3] + 1/LIPM_SAMPLING * ((states[i][2])*G/Z_ROBOT_WALK)
        y_next  = states[i][2] + 1/LIPM_SAMPLING * states[i][3]
        states = np.concatenate((states, [[x_next, dx_next, y_next, dy_next]]), axis=0)
    
    body_position = zip(np.add(states[:,0] + COM_X_OFFSET, initial_r_foot_pos[0]), np.add(states[:,2],initial_r_foot_pos[1]), [Z_ROBOT_WALK for i in states])

    P_CoM = np.array([list(i) for i in list(body_position)])

    initial_l_foot_pos  = [0, Y_BODY_TO_FEET, 0]
    final_l_foot_pos    = [STEP_LENGTH, Y_BODY_TO_FEET, 0]
    final_posl_left= final_l_foot_pos
    l_foot_abs_pos = getFootSwingTraj(initial_l_foot_pos, final_l_foot_pos, STEP_HEIGHT, steptimeVector)
    r_foot_abs_pos = np.full((len(steptimeVector), 3), initial_r_foot_pos)

    l_leg_relative_pos  = l_foot_abs_pos - P_CoM
    r_leg_relative_pos  = r_foot_abs_pos - P_CoM

    l_leg_relative_pos = l_leg_relative_pos[::int(LIPM_SAMPLING/SERVO_RATE)]
    r_leg_relative_pos = r_leg_relative_pos[::int(LIPM_SAMPLING/SERVO_RATE)]

    left_q  = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, l_foot_abs_pos

def calculate_cartesian_right_end_step_pose(initial_l_foot_pos, ik_client_left, ik_client_right):
    #Left foot is the support foot
    y_0 = -Y_BODY_TO_FEET

    [dy0, x_0, dx0, single_support_time] = findInitialConditions(STEP_LENGTH, ROBOT_VEL_X, y_0, Z_ROBOT_WALK, G)

    state0 = [x_0, dx0, y_0, dy0]
    steptimeVector = np.linspace(0, single_support_time, math.floor(single_support_time*LIPM_SAMPLING))

    nSteps = len(steptimeVector)
    states = np.array([state0])
    #Solve differential equation (sample time must be as small as possible)
    for i in range(0, nSteps-1):
        dx_next = states[i][1] + 1/LIPM_SAMPLING * ((states[i][0])*G/Z_ROBOT_WALK)
        x_next  = states[i][0] + 1/LIPM_SAMPLING * states[i][1]
        dy_next = states[i][3] + 1/LIPM_SAMPLING * ((states[i][2])*G/Z_ROBOT_WALK)
        y_next  = states[i][2] + 1/LIPM_SAMPLING * states[i][3]
        states = np.concatenate((states, [[x_next, dx_next, y_next, dy_next]]), axis=0)
    
    body_position = zip(np.add(states[:,0] + COM_X_OFFSET, initial_l_foot_pos[0]), np.add(states[:,2],initial_l_foot_pos[1]), [Z_ROBOT_WALK for i in states])

    P_CoM = np.array([list(i) for i in list(body_position)])

    initial_r_foot_pos  = [0, -Y_BODY_TO_FEET, 0]
    final_r_foot_pos    = [STEP_LENGTH, -Y_BODY_TO_FEET,0]
    l_foot_abs_pos = np.full((len(steptimeVector), 3), initial_l_foot_pos)
    r_foot_abs_pos = getFootSwingTraj(initial_r_foot_pos, final_r_foot_pos, STEP_HEIGHT, steptimeVector)

    l_leg_relative_pos  = l_foot_abs_pos - P_CoM
    r_leg_relative_pos  = r_foot_abs_pos - P_CoM

    l_leg_relative_pos = l_leg_relative_pos[::int(LIPM_SAMPLING/SERVO_RATE)]
    r_leg_relative_pos = r_leg_relative_pos[::int(LIPM_SAMPLING/SERVO_RATE)]

    left_q = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, r_foot_abs_pos

def calculate_cartesian_left_end_step_pose(initial_r_foot_pos, ik_client_left, ik_client_right):
    #Left foot is the support foot
    global final_posl_left
    y_0 = Y_BODY_TO_FEET

    [dy0, x_0, dx0, single_support_time] = findInitialConditions(STEP_LENGTH, ROBOT_VEL_X, y_0, Z_ROBOT_WALK, G)

    state0 = [x_0, dx0, y_0, dy0]
    steptimeVector = np.linspace(0, single_support_time, math.floor(single_support_time*LIPM_SAMPLING))

    nSteps = len(steptimeVector)
    states = np.array([state0])
    #Solve differential equation (sample time must be as small as possible)
    for i in range(0, nSteps-1):
        dx_next = states[i][1] + 1/LIPM_SAMPLING * ((states[i][0])*G/Z_ROBOT_WALK)
        x_next  = states[i][0] + 1/LIPM_SAMPLING * states[i][1]
        dy_next = states[i][3] + 1/LIPM_SAMPLING * ((states[i][2])*G/Z_ROBOT_WALK)
        y_next  = states[i][2] + 1/LIPM_SAMPLING * states[i][3]
        states = np.concatenate((states, [[x_next, dx_next, y_next, dy_next]]), axis=0)
    
    body_position = zip(np.add(states[:,0] + COM_X_OFFSET,initial_r_foot_pos[0]), np.add(states[:,2],initial_r_foot_pos[1]), [Z_ROBOT_WALK for i in states])

    P_CoM = np.array([list(i) for i in list(body_position)])

    initial_l_foot_pos  = [0, Y_BODY_TO_FEET, 0]
    final_l_foot_pos    = [STEP_LENGTH, Y_BODY_TO_FEET,0]
    final_posl_left= final_l_foot_pos
    l_foot_abs_pos = getFootSwingTraj(initial_l_foot_pos, final_l_foot_pos, STEP_HEIGHT, steptimeVector)
    r_foot_abs_pos = np.full((len(steptimeVector), 3), initial_r_foot_pos)

    l_leg_relative_pos  = l_foot_abs_pos - P_CoM
    r_leg_relative_pos  = r_foot_abs_pos - P_CoM

    l_leg_relative_pos = l_leg_relative_pos[::int(LIPM_SAMPLING/SERVO_RATE)]
    r_leg_relative_pos = r_leg_relative_pos[::int(LIPM_SAMPLING/SERVO_RATE)]

    left_q  = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, l_foot_abs_pos

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
    h = x_0 + (x_1 - x_0)/2
    k = swing_height
    a = -k/((x_0-h)**2)
    m = (x_1 - x_0)/(timeVector[-1]- timeVector[0])
    x_t = lambda t: x_0 + m*t
    z = lambda x: a*((x-h)**2) + k

    swingFootTrajectory = np.array([[0, 0, 0]])

    for i in timeVector:
        aux = np.array([[x_t(i), initial_foot_position[1], z(x_t(i))]])
        swingFootTrajectory = np.concatenate((swingFootTrajectory,aux), axis=0)
    swingFootTrajectory = np.delete(swingFootTrajectory, 0, axis=0)
    return swingFootTrajectory

def executeTrajectories(left_foot_q, right_foot_q, rate: rospy.Rate, legs_publisher: rospy.Publisher):
    for left, right in zip(left_foot_q, right_foot_q):
        legs_msg = Float32MultiArray()
        legs_msg.data = [*left] + [*right]
        legs_publisher.publish(legs_msg)
        rate.sleep()

def main(args = None):
    global Y_BODY_TO_FEET, Z_ROBOT_WALK, Z_ROBOT_STATIC, STEP_HEIGHT, STEP_LENGTH, ROBOT_VEL_X, COM_X_OFFSET, COM_Y_OFFSET, LIPM_SAMPLING, SERVO_RATE
    rospy.init_node('step_test_node')

    right_leg_client    = rospy.ServiceProxy('/control/ik_leg_right', CalculateIK)
    left_leg_client     = rospy.ServiceProxy('/control/ik_leg_left', CalculateIK)
    
    rospy.wait_for_service("/control/ik_leg_right")
    rospy.wait_for_service("/control/ik_leg_left")

    legs_goal_pub       = rospy.Publisher('/hardware/legs_goal_pose', Float32MultiArray, queue_size=1)
    arms_goal_pose      = rospy.Publisher("/hardware/arms_goal_pose", Float32MultiArray , queue_size=1)


    Y_BODY_TO_FEET  = rospy.get_param("/walk/y_body_to_feet")
    Z_ROBOT_WALK    = rospy.get_param("/walk/z_robot_walk")
    Z_ROBOT_STATIC  = rospy.get_param("/walk/z_robot_static")
    STEP_LENGTH     = rospy.get_param("/walk/step_height")
    STEP_HEIGHT     = rospy.get_param("/walk/step_length")
    ROBOT_VEL_X     = rospy.get_param("/walk/robot_vel_x")
    COM_X_OFFSET    = rospy.get_param("/walk/com_x_offset")
    COM_Y_OFFSET    = rospy.get_param("/walk/com_y_offset")
    LIPM_SAMPLING   = rospy.get_param("/walk/lipm_sampling")
    SERVO_RATE      = rospy.get_param("/walk/servo_rate")


    
    print(f"Y_BODY_TO_FEET is:\t\t {Y_BODY_TO_FEET}")
    print(f"Z_ROBOT_WALK  is:\t\t {Z_ROBOT_WALK}")
    print(f"Z_ROBOT_STATIC is:\t {Z_ROBOT_STATIC}")
    print(f"STEP_LENGTH   is:\t {STEP_LENGTH}")
    print(f"STEP_HEIGHT   is:\t {STEP_HEIGHT}")
    print(f"ROBOT_VEL_X   is:\t {ROBOT_VEL_X}")
    print(f"COM_X_OFFSET  is:\t {COM_X_OFFSET}")
    print(f"COM_Y_OFFSET  is:\t {COM_Y_OFFSET}")
    print(f"LIPM_SAMPLING is:\t {LIPM_SAMPLING}")
    print(f"SERVO_RATE    is:\t {SERVO_RATE}")

    time.sleep(5)

    first_left_q, first_right_q, last_p_com = calculate_cartesian_right_start_pose(2, 1.0, left_leg_client, right_leg_client)
    print("\n Done with start_pose\n")

    initial_halfstep_pos = last_p_com
    final_halfstep_pos = [STEP_LENGTH/2 + COM_X_OFFSET, 0, Z_ROBOT_WALK]

    second_left_q, second_right_q, final_l_foot_pos = calculate_cartesian_left_half_step_pose(0.5, initial_halfstep_pos, final_halfstep_pos, left_leg_client, right_leg_client)
    print("\n Done with first_halfstep \n")

    third_left_q, third_right_q, final_r_foot_pos = calculate_cartesian_right_step_pose(final_l_foot_pos[-1], left_leg_client, right_leg_client)
    print("\n Done with right_full_step\n")

    fourth_left_q, fourth_right_q, final_l_foot_pos = calculate_cartesian_left_step_pose(final_r_foot_pos[-1], left_leg_client, right_leg_client)
    print("\n Done with left_full_step\n")

    fourth_a_left_q, fourth_a_right_q, final_r_foot_pos = calculate_cartesian_right_end_step_pose(final_l_foot_pos[-1], left_leg_client, right_leg_client)
    print("\n Done with right_stop_step\n")

    third_a_left_q, right_a_right_q, final_l_foot_pos = calculate_cartesian_left_end_step_pose(final_r_foot_pos[-1], left_leg_client, right_leg_client)
    print("\n Done with left_stop step\n")
  
    final_left_q, final_q, last_p_com = calculate_cartesian_right_end_pose(1, 0.0, left_leg_client, right_leg_client)
    print("\n Done with start_pose\n")

    arms_msg = Float32MultiArray()
    arms_msg.data = [0.0, 0.3, 0.0, 0.0, -0.3, 0.0]
    arms_goal_pose.publish(arms_msg)

    zero_msg = Float32MultiArray()
    zero_msg.data = [0.0 for i in range(12)]
    legs_goal_pub.publish(zero_msg)
    rate = rospy.Rate(SERVO_RATE)
    
    executeTrajectories(first_left_q, first_right_q, rate, legs_goal_pub)
    #time.sleep(1)
    executeTrajectories(second_left_q, second_right_q, rate, legs_goal_pub)
    while not rospy.is_shutdown():
        executeTrajectories(third_left_q, third_right_q, rate,legs_goal_pub)
        executeTrajectories(fourth_left_q, fourth_right_q, rate, legs_goal_pub)
    executeTrajectories(fourth_a_left_q, fourth_a_right_q, rate,legs_goal_pub)

    return 0



if __name__ == "__main__":
    exit(main())
