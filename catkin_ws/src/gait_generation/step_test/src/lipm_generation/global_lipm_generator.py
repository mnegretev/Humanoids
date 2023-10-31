import numpy as np
from scipy import signal, interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

G = 9.81 # [m/s^2]

X_BODY_TO_FEET  = 0.12 # [m]
Z_ROBOT_WALK    = 0.68 # m
Z_ROBOT_STATIC= 0.75 # m

# stepHeight = 0.2
STEP_LENGTH = 0.2 # [m]
ROBOT_VEL_Y = 0.1 # [m]

# Tiempo de muestreo
SAMPLE_TIME = 0.0001 # [s]

class Step:
    stepmode = ["DoubleSupport",
                "LeftSupport",
                "RightSupport"
                ]
    
    def __init__(self, mode=None):
        self.com_rel_position = []
        
        if mode in Step.stepmode:
            self.mode = mode
        else:
            self.mode = None

        self.timevec = []
        self.footleft = []
        self.footright = []
        self.jointsleft = []
        self.jointsright = []
        self.transmatleft = []
        self.transmatright = []


def main():

    steps = []

    x_0 = X_BODY_TO_FEET
    [dx0, y_0, dy0, single_support_time] = findInitialConditions(STEP_LENGTH,
                                                                 ROBOT_VEL_Y,
                                                                 x_0,
                                                                 Z_ROBOT_WALK,
                                                                 G)

    #Initial conditions vector that guarantees symetric trajectories in LIPM
    state0 = [x_0, dx0, y_0, dy0]
    body_position = np.array([[0,0,0]])
    time_vector = np.array([])# [m]

    #################################################################
    ## PART 1: GO TO START POSE
    #################################################################
    #First, lower the body of the robot and move its COM to the right foot
    #This is a harcoded value that can be modified later

    starting_body_points = np.array([
                     [0,0.67*X_BODY_TO_FEET],
                     [0,0],
                     [Z_ROBOT_STATIC, Z_ROBOT_WALK]])

    timepoints = [0,1]

    time_vector = np.linspace(timepoints[0],timepoints[1], math.floor(1/SAMPLE_TIME))

    m_x = (starting_body_points[0][1] - starting_body_points[0][0])/(timepoints[1]-timepoints[0])
    m_z = (starting_body_points[2][1] - starting_body_points[2][0])/(timepoints[1]-timepoints[0])

    q_x = interpolate.interp1d(time_vector, time_vector*m_x*SAMPLE_TIME, fill_value="extrapolate")
    q_y = 0
    q_z = interpolate.interp1d(time_vector, 
                               starting_body_points[2][0] + time_vector*m_z*SAMPLE_TIME,
                               fill_value="extrapolate")

    for i in np.arange(0,len(time_vector)):
        aux = np.array([[q_x(i), q_y, q_z(i)]])
        body_position = np.concatenate((body_position,aux), axis=0)
    body_position = np.delete(body_position, 0, axis=0)

    rel_position = body_position - [-X_BODY_TO_FEET, 0, 0]

    foot_right = np.array([[-X_BODY_TO_FEET,0,0]])
    foot_left = np.array([[X_BODY_TO_FEET,0,0]])
    
    for i in np.arange(0, len(time_vector)-1):
        foot_right = np.concatenate((foot_right, np.array([[-x_0, 0, 0]])), axis=0)
        foot_left = np.concatenate((foot_left, np.array([[x_0, 0 , 0]])), axis=0)

    # print(f"Relative position COM matrix shape: {rel_position.shape}")
    # print(rel_position)
    print(f"Right foot matrix shape: {foot_right.shape}")
    print(foot_right)
    print(f"Left foot matrix shape: {foot_left.shape}")
    print(foot_left)

    start_pose_step = Step(mode="DoubleSupport")
    start_pose_step.timevec = time_vector
    start_pose_step.footleft = foot_left
    start_pose_step.footright = foot_right

    steps.append(start_pose_step)

    #################################################################
    ## PART 2: MAKE A HALF STEP
    #################################################################
    # Store left and right foothold position (left:odd, right:even)

    #TODO COMENZAR A PARTIR DE AQUI CON EL SIGUIENTE STEP, EL ANTERIOR YA ESTA

    final_halfstep_position = np.array([[0],[0.1],[Z_ROBOT_WALK]])
    starting_body_points = np.concatenate((starting_body_points, final_halfstep_position), axis=1)

    m_x = (starting_body_points[0][2] - starting_body_points[0][1])/(timepoints[1]-timepoints[0])
    m_y = (starting_body_points[1][2] - starting_body_points[1][1])/(timepoints[1]-timepoints[0])

    q_x = interpolate.interp1d( time_vector, 
                                starting_body_points[0][1] + time_vector*m_x*SAMPLE_TIME,
                                fill_value="extrapolate")
    q_z = Z_ROBOT_WALK
    q_y = interpolate.interp1d(time_vector, 
                               starting_body_points[1][1] + time_vector*m_y*SAMPLE_TIME,
                               fill_value="extrapolate")

    for i in np.arange(0,len(time_vector)):
        aux = np.array([[q_x(i), q_y(i), q_z]])
        body_position = np.concatenate((body_position,aux), axis=0)
    body_position = np.delete(body_position, 0, axis=0)

    foot_position = np.array([
                            [-X_BODY_TO_FEET, X_BODY_TO_FEET],
                            [0,0],
                            [0,0]
                            ])

    print(body_position.shape)
    print(body_position)

    start_pose_step = Step(mode="RightSupport")

    exit()

    #################################################################
    ## PART 3: MAKE CONSECUTIVE STEPS
    #################################################################

    footHold_x0 = -0.12
    footHold_y0 = 0.2

    foot_position = np.concatenate((foot_position, [[footHold_x0],[footHold_y0],[0]]), axis=1)

    #foot_swing = 

    state0 = [0.12, dx0, -0.1, dy0]

    for i in range(1,2  ):
        tInitial = time_vector[-1]
        tFinal = tInitial + single_support_time
        steptimeVector = np.linspace(tInitial, tFinal, math.floor((tFinal-tInitial)/SAMPLE_TIME))

        #Simulation loop
        nSteps = len(steptimeVector)
        states = np.array([state0])

        for i in range(0, nSteps-1):
            dx_next = states[i][1] + SAMPLE_TIME*((states[i][0])*G/Z_ROBOT_WALK)
            x_next = states[i][0] + SAMPLE_TIME*states[i][1]
            dy_next = states[i][3] + SAMPLE_TIME*((states[i][2])*G/Z_ROBOT_WALK)
            y_next = states[i][2] + SAMPLE_TIME*states[i][3]
            states = np.concatenate((states, [[x_next, dx_next, y_next, dy_next]]), axis=0)

        aux = zip(np.add(states[:,0],footHold_x0), np.add(states[:,2],footHold_y0), [Z_ROBOT_WALK for i in states])
        body_position = np.concatenate((body_position,[list(i) for i in list(aux)]), axis=0)

        [state0, footHold_x0, footHold_y0] = changeLeg(states[-1], body_position)

        foot_position = np.concatenate((foot_position, [[footHold_x0],[footHold_y0],[0]]), axis=1)

    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(body_position[:,0],body_position[:,1],body_position[:,2], "o")
    ax.plot(foot_position[0,:],foot_position[1,:], foot_position[2,:], "o")
    
    plt.show()

def findInitialConditions(STEP_LENGTH, ROBOT_VEL_Y, x_0, zModel, G):
    #Desired midstance and state
    s = math.sqrt(zModel/G)
    y_mid = 0
    # using relationship between final body state and initial body state,
    # we can find time it will take to re
    #Corresponding orbital energy is
    E = -G/(2*zModel) * (y_mid**2) + 0.5*(ROBOT_VEL_Y**2)
    y_0 = -STEP_LENGTH/2
    #Finding dy0 from midstance energy level
    # using relationship between final body state and initial body state,
    # we can find time it will take to re
    dy0 = math.sqrt(2*(E + G/(2*zModel) * (y_0**2)))
    # using relationship between final body state and initial body state,
    # we can find time it will take to reach midstance given final velocity
    # (dy = ROBOT_VEL_Y) and final position (which is y = 0 at midstance)
    tsinglesupport = 2*math.asinh( STEP_LENGTH/2/(s*ROBOT_VEL_Y) ) * s
    print("singlesupportTIme is ", tsinglesupport)

    tf = tsinglesupport/2

    dx0 = -x_0/s * math.sinh(tf/s) / math.cosh(tf/s)

    return [dx0, y_0, dy0, tsinglesupport]

def changeLeg(finalState, body_position):
    xf  =   finalState[0]
    dxf =   finalState[1]
    yf  =   finalState[2]
    dyf =   finalState[3]

    #Use simple control law (mirroring)
    x_0  = -xf

    y_0  = -yf 
    dx0 = dxf
    dy0 = dyf

    newInitialState = [x_0, dx0, y_0, dy0]

    newFoothold_x = body_position[-1,0] - x_0
    newFoothold_y = body_position[-1,1] - y_0

    return [newInitialState, newFoothold_x, newFoothold_y]

def getFootSwingTraj(footpos0, footpos1, swing_height, initial_state, t0, tf, Ts):
    pass

def cubicPolyTraj():
    pass

main()
