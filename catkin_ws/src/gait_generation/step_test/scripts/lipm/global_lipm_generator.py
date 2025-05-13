#!/usr/bin/env python
import numpy as np
from scipy import signal, interpolate
#import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import os

#ROS
import rospy
from std_msgs.msg import String, Float32MultiArray
from ctrl_msgs.srv import CalculateIK, CalculateIKRequest, CalculateDK, CalculateDKRequest
from trajectory_planner import trajectory_planner
from manip_msgs.srv import *

G = 9.81 # [m/s^2]

# Y_BODY_TO_FEET  = 0.0555 # [m]
Y_BODY_TO_FEET  = 0.55 #Mínimo valor =0.056 #Máximo valor =0.125#= 0.09
# Z_ROBOT_WALK  = 0.55 # m
Z_ROBOT_WALK    = 0.3
Z_ROBOT_STATIC  = 0.35 #Máximo valor = 0.576 # m

stepHeight = 0.04
STEP_LENGTH = 0.03 # [m]
ROBOT_VEL_X = 0.1 # [m]

com_x_offset = 0.02 #original=0.02

# Tiempo de muestreo para resolver la ecuación diferencial del LIPM (debe ser pequeño)
LIPM_SAMPLE_TIME = 0.0001 # [s]

# Tiempo de muestreo maximo para escribir a los servomotores
SERVO_SAMPLE_TIME = 0.05 # [s]

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

def calculate_Fk(q, service_client):
    cartesian_traj = np.zeros((len(q),6))
    for i, vector in enumerate(q):
        req=vector
        response = service_client(req)
        x=response.x
        y=response.y
        z=response.z
        roll=response.roll
        pitch=response.pitch
        yaw=response.yaw
        cartesian_traj[i]=[x,y,z,roll,pitch,yaw]
    return cartesian_traj

def calculate_cartesian_right_start_pose(duration, y_body_to_feet_percent, ik_client_left, ik_client_right):
    p_start = [0 + com_x_offset, 0, Z_ROBOT_STATIC]
    p_end = [0 + com_x_offset, -y_body_to_feet_percent*Y_BODY_TO_FEET, Z_ROBOT_WALK]

    
    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_end, duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    left_leg_relative_pos = [0, Y_BODY_TO_FEET, 0] - P_CoM
    right_leg_relative_pos =  [0, -Y_BODY_TO_FEET, 0] - P_CoM

    left_q = calculate_ik(left_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(right_leg_relative_pos, ik_client_right)

    return left_q, right_q, P_CoM[-1],

def calculate_cartesian_right_end_pose(duration, y_body_to_feet_percent, ik_client_left, ik_client_right):
    p_end = [0 + com_x_offset, 0, Z_ROBOT_STATIC]
    p_start = [0 + com_x_offset, -y_body_to_feet_percent*Y_BODY_TO_FEET, Z_ROBOT_WALK]

    
    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_end, duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    left_leg_relative_pos = [0, Y_BODY_TO_FEET, 0] - P_CoM
    right_leg_relative_pos =  [0, -Y_BODY_TO_FEET, 0] - P_CoM

    left_q = calculate_ik(left_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(right_leg_relative_pos, ik_client_right)

    return left_q, right_q, P_CoM[-1],

def calculate_cartesian_left_half_step_pose(duration, p_start, p_end, ik_client_left, ik_client_right):
    P_CoM, T = trajectory_planner.get_polynomial_trajectory_multi_dof(p_start, p_end, duration=duration, time_step=SERVO_SAMPLE_TIME)
    
    initial_r_foot_pos  = np.array([0, -Y_BODY_TO_FEET, 0])
    initial_l_foot_pos  = np.array([0,  Y_BODY_TO_FEET, 0])

    #final_r_foot_pos    = initial_r_foot_pos
    final_l_foot_pos    = np.array([STEP_LENGTH/2, Y_BODY_TO_FEET, 0])

    r_leg_abs_pos = np.full((len(T), 3), initial_r_foot_pos)
    l_leg_abs_pos = getFootSwingTraj(initial_l_foot_pos, final_l_foot_pos, stepHeight, T)

    r_leg_relative_pos  = r_leg_abs_pos - P_CoM
    l_leg_relative_pos  = l_leg_abs_pos - P_CoM

    left_q = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, l_leg_abs_pos

def calculate_cartesian_right_step_pose(initial_l_foot_pos, ik_client_left, ik_client_right):
    global final_posr_right
    #Left foot is the support foot
    y_0 = -Y_BODY_TO_FEET

    [dy0, x_0, dx0, single_support_time] = findInitialConditions(STEP_LENGTH/2, ROBOT_VEL_X, y_0, Z_ROBOT_WALK, G)

    state0 = [x_0, dx0, y_0, dy0]
    tFinal = single_support_time
    steptimeVector = np.linspace(0, tFinal, math.floor(tFinal/LIPM_SAMPLE_TIME))

    nSteps = len(steptimeVector)
    states = np.array([state0])
    #Solve differential equation (sample time must be as small as possible)
    for i in range(0, nSteps-1):
        dx_next = states[i][1] + LIPM_SAMPLE_TIME * ((states[i][0])*G/Z_ROBOT_WALK)
        x_next  = states[i][0] + LIPM_SAMPLE_TIME * states[i][1]
        dy_next = states[i][3] + LIPM_SAMPLE_TIME * ((states[i][2])*G/Z_ROBOT_WALK)
        y_next  = states[i][2] + LIPM_SAMPLE_TIME * states[i][3]
        states = np.concatenate((states, [[x_next, dx_next, y_next, dy_next]]), axis=0)
    
    body_position = zip(np.add(states[:,0] + com_x_offset, initial_l_foot_pos[0]), np.add(states[:,2],initial_l_foot_pos[1]), [Z_ROBOT_WALK for i in states])

    P_CoM = np.array([list(i) for i in list(body_position)])

    initial_r_foot_pos  = [0, -Y_BODY_TO_FEET, 0]
    final_r_foot_pos    = [STEP_LENGTH, -Y_BODY_TO_FEET,0]
    final_posr_right = final_r_foot_pos
    l_foot_abs_pos = np.full((len(steptimeVector), 3), initial_l_foot_pos)
    r_foot_abs_pos = getFootSwingTraj(initial_r_foot_pos, final_r_foot_pos, stepHeight, steptimeVector)

    l_leg_relative_pos  = l_foot_abs_pos - P_CoM
    r_leg_relative_pos  = r_foot_abs_pos - P_CoM

    l_leg_relative_pos = l_leg_relative_pos[::int(SERVO_SAMPLE_TIME/LIPM_SAMPLE_TIME)]
    r_leg_relative_pos = r_leg_relative_pos[::int(SERVO_SAMPLE_TIME/LIPM_SAMPLE_TIME)]

    left_q = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, r_foot_abs_pos

def calculate_cartesian_left_step_pose(initial_r_foot_pos, ik_client_left, ik_client_right):
    #Left foot is the support foot
    global final_posl_left
    y_0 = Y_BODY_TO_FEET

    [dy0, x_0, dx0, single_support_time] = findInitialConditions(STEP_LENGTH/2, ROBOT_VEL_X, y_0, Z_ROBOT_WALK, G)

    state0 = [x_0, dx0, y_0, dy0]
    tFinal = single_support_time
    steptimeVector = np.linspace(0, tFinal, math.floor(tFinal/LIPM_SAMPLE_TIME))

    nSteps = len(steptimeVector)
    states = np.array([state0])
    #Solve differential equation (sample time must be as small as possible)
    for i in range(0, nSteps-1):
        dx_next = states[i][1] + LIPM_SAMPLE_TIME * ((states[i][0])*G/Z_ROBOT_WALK)
        x_next  = states[i][0] + LIPM_SAMPLE_TIME * states[i][1]
        dy_next = states[i][3] + LIPM_SAMPLE_TIME * ((states[i][2])*G/Z_ROBOT_WALK)
        y_next  = states[i][2] + LIPM_SAMPLE_TIME * states[i][3]
        states = np.concatenate((states, [[x_next, dx_next, y_next, dy_next]]), axis=0)
    
    body_position = zip(np.add(states[:,0] + com_x_offset,initial_r_foot_pos[0]), np.add(states[:,2],initial_r_foot_pos[1]), [Z_ROBOT_WALK for i in states])

    P_CoM = np.array([list(i) for i in list(body_position)])

    initial_l_foot_pos  = [0, Y_BODY_TO_FEET, 0]
    final_l_foot_pos    = [STEP_LENGTH, Y_BODY_TO_FEET,0]
    final_posl_left= final_l_foot_pos
    l_foot_abs_pos = getFootSwingTraj(initial_l_foot_pos, final_l_foot_pos, stepHeight, steptimeVector)
    r_foot_abs_pos = np.full((len(steptimeVector), 3), initial_r_foot_pos)

    l_leg_relative_pos  = l_foot_abs_pos - P_CoM
    r_leg_relative_pos  = r_foot_abs_pos - P_CoM

    l_leg_relative_pos = l_leg_relative_pos[::int(SERVO_SAMPLE_TIME/LIPM_SAMPLE_TIME)]
    r_leg_relative_pos = r_leg_relative_pos[::int(SERVO_SAMPLE_TIME/LIPM_SAMPLE_TIME)]

    left_q  = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, l_foot_abs_pos

def calculate_cartesian_right_end_step_pose(initial_l_foot_pos, ik_client_left, ik_client_right):
    #Left foot is the support foot
    y_0 = -Y_BODY_TO_FEET

    [dy0, x_0, dx0, single_support_time] = findInitialConditions(STEP_LENGTH/2, ROBOT_VEL_X, y_0, Z_ROBOT_WALK, G)

    state0 = [x_0, dx0, y_0, dy0]
    tFinal = single_support_time
    steptimeVector = np.linspace(0, tFinal, math.floor(tFinal/LIPM_SAMPLE_TIME))

    nSteps = len(steptimeVector)
    states = np.array([state0])
    #Solve differential equation (sample time must be as small as possible)
    for i in range(0, nSteps-1):
        dx_next = states[i][1] + LIPM_SAMPLE_TIME * ((states[i][0])*G/Z_ROBOT_WALK)
        x_next  = states[i][0] + LIPM_SAMPLE_TIME * states[i][1]
        dy_next = states[i][3] + LIPM_SAMPLE_TIME * ((states[i][2])*G/Z_ROBOT_WALK)
        y_next  = states[i][2] + LIPM_SAMPLE_TIME * states[i][3]
        states = np.concatenate((states, [[x_next, dx_next, y_next, dy_next]]), axis=0)
    
    body_position = zip(np.add(states[:,0] + com_x_offset, initial_l_foot_pos[0]), np.add(states[:,2],initial_l_foot_pos[1]), [Z_ROBOT_WALK for i in states])

    P_CoM = np.array([list(i) for i in list(body_position)])

    initial_r_foot_pos  = [0, -Y_BODY_TO_FEET, 0]
    final_r_foot_pos    = [STEP_LENGTH, -Y_BODY_TO_FEET,0]
    l_foot_abs_pos = np.full((len(steptimeVector), 3), initial_l_foot_pos)
    r_foot_abs_pos = getFootSwingTraj(initial_r_foot_pos, final_r_foot_pos, stepHeight, steptimeVector)

    l_leg_relative_pos  = l_foot_abs_pos - P_CoM
    r_leg_relative_pos  = r_foot_abs_pos - P_CoM

    l_leg_relative_pos = l_leg_relative_pos[::int(SERVO_SAMPLE_TIME/LIPM_SAMPLE_TIME)]
    r_leg_relative_pos = r_leg_relative_pos[::int(SERVO_SAMPLE_TIME/LIPM_SAMPLE_TIME)]

    left_q = calculate_ik(l_leg_relative_pos, ik_client_left)
    right_q = calculate_ik(r_leg_relative_pos, ik_client_right)

    return left_q, right_q, r_foot_abs_pos

def calculate_cartesian_left_end_step_pose(initial_r_foot_pos, ik_client_left, ik_client_right):
    #Left foot is the support foot
    global final_posl_left
    y_0 = Y_BODY_TO_FEET

    [dy0, x_0, dx0, single_support_time] = findInitialConditions(STEP_LENGTH/2, ROBOT_VEL_X, y_0, Z_ROBOT_WALK, G)

    state0 = [x_0, dx0, y_0, dy0]
    tFinal = single_support_time
    steptimeVector = np.linspace(0, tFinal, math.floor(tFinal/LIPM_SAMPLE_TIME))

    nSteps = len(steptimeVector)
    states = np.array([state0])
    #Solve differential equation (sample time must be as small as possible)
    for i in range(0, nSteps-1):
        dx_next = states[i][1] + LIPM_SAMPLE_TIME * ((states[i][0])*G/Z_ROBOT_WALK)
        x_next  = states[i][0] + LIPM_SAMPLE_TIME * states[i][1]
        dy_next = states[i][3] + LIPM_SAMPLE_TIME * ((states[i][2])*G/Z_ROBOT_WALK)
        y_next  = states[i][2] + LIPM_SAMPLE_TIME * states[i][3]
        states = np.concatenate((states, [[x_next, dx_next, y_next, dy_next]]), axis=0)
    
    body_position = zip(np.add(states[:,0] + com_x_offset,initial_r_foot_pos[0]), np.add(states[:,2],initial_r_foot_pos[1]), [Z_ROBOT_WALK for i in states])

    P_CoM = np.array([list(i) for i in list(body_position)])

    initial_l_foot_pos  = [0, Y_BODY_TO_FEET, 0]
    final_l_foot_pos    = [STEP_LENGTH, Y_BODY_TO_FEET,0]
    final_posl_left= final_l_foot_pos
    l_foot_abs_pos = getFootSwingTraj(initial_l_foot_pos, final_l_foot_pos, stepHeight, steptimeVector)
    r_foot_abs_pos = np.full((len(steptimeVector), 3), initial_r_foot_pos)

    l_leg_relative_pos  = l_foot_abs_pos - P_CoM
    r_leg_relative_pos  = r_foot_abs_pos - P_CoM

    l_leg_relative_pos = l_leg_relative_pos[::int(SERVO_SAMPLE_TIME/LIPM_SAMPLE_TIME)]
    r_leg_relative_pos = r_leg_relative_pos[::int(SERVO_SAMPLE_TIME/LIPM_SAMPLE_TIME)]

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

def main(args = None):
    rospy.init_node('step_test_node')

    trajectory_dir      = rospy.get_param("~trajectory_dir")
    right_leg_client    = rospy.ServiceProxy('/control/ik_leg_right', CalculateIK)
    left_leg_client     = rospy.ServiceProxy('/control/ik_leg_left', CalculateIK)
    right_Dleg_client   = rospy.ServiceProxy('/control/fk_leg_right_pose', CalculateDK)
    left_Dleg_client    = rospy.ServiceProxy('/control/fk_leg_left_pose', CalculateDK)

    left_q, right_q, last_p_com = calculate_cartesian_right_start_pose(1, 0.0, left_leg_client, right_leg_client)
    np.savez(os.path.join(trajectory_dir, "right_start_pose"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)

    initial_halfstep_pos = last_p_com
    final_halfstep_pos = [STEP_LENGTH/4 + com_x_offset, 0, Z_ROBOT_WALK]

    left_q, right_q, final_l_foot_pos = calculate_cartesian_left_half_step_pose(0.5, initial_halfstep_pos, final_halfstep_pos, left_leg_client, right_leg_client)
    np.savez(os.path.join(trajectory_dir, "left_first_halfstep_pose"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)
    # cartesian_l_pos = calculate_Fk(left_q, left_Dleg_client)
    # np.savetxt(os.path.join(trajectory_dir, "csv/left_first_half_step_pose.csv"), cartesian_l_pos)
    
    left_q, right_q, final_r_foot_pos = calculate_cartesian_right_step_pose(final_l_foot_pos[-1], left_leg_client, right_leg_client)
    np.savez(os.path.join(trajectory_dir, "right_full_step_pose"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)
    # cartesian_r_pos = calculate_Fk(right_q, right_Dleg_client)
    # np.savetxt(os.path.join(trajectory_dir,"csv/right_full_step_pose.csv"), cartesian_r_pos)

    left_q, right_q, final_l_foot_pos = calculate_cartesian_left_step_pose(final_r_foot_pos[-1], left_leg_client, right_leg_client)
    np.savez(os.path.join(trajectory_dir, "left_full_step_pose"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)
    # cartesian_l_pos = calculate_Fk(left_q, left_Dleg_client)
    # np.savetxt(os.path.join(trajectory_dir,"csv/left_full_step_pose.csv"), cartesian_l_pos)

    left_q, right_q, last_p_com = calculate_cartesian_right_end_pose(1, 0.0, left_leg_client, right_leg_client)
    np.savez(os.path.join(trajectory_dir, "right_end_pose"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)

    left_q, right_q, final_r_foot_pos = calculate_cartesian_right_end_step_pose(final_l_foot_pos[-1], left_leg_client, right_leg_client)
    np.savez(os.path.join(trajectory_dir, "right_end_step_pose"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)
    # cartesian_r_pos = calculate_Fk(right_q, right_Dleg_client)
    # np.savetxt(os.path.join(trajectory_dir,"csv/right_end_step_pose.csv"), cartesian_r_pos)

    left_q, right_q, final_l_foot_pos = calculate_cartesian_left_end_step_pose(final_r_foot_pos[-1], left_leg_client, right_leg_client)
    np.savez(os.path.join(trajectory_dir, "left_end_step_pose"), right=right_q, left=left_q, timestep=SERVO_SAMPLE_TIME)
    # cartesian_l_pos = calculate_Fk(left_q, left_Dleg_client)
    # np.savetxt(os.path.join(trajectory_dir,"csv/left_end_step_pose.csv"), cartesian_l_pos)
    
if __name__ == "__main__":
    main()
