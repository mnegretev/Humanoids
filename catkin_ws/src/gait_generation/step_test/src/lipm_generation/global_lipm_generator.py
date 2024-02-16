import numpy as np
from scipy import signal, interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

G = 9.81 # [m/s^2]

Y_BODY_TO_FEET  = 0.12 # [m]
Z_ROBOT_WALK    = 0.68 # m
Z_ROBOT_STATIC= 0.75 # m

stepHeight = 0.2
STEP_LENGTH = 0.2 # [m]
ROBOT_VEL_X = 0.1 # [m]

# Tiempo de muestreo
SAMPLE_TIME = 0.0001 # [s]

class Step:
    stepmode = ["DoubleSupport",
                "LeftSupport",
                "RightSupport"
                ]
    
    def __init__(self, mode=None):
        self.com_rel_position = []
        
        assert mode in Step.stepmode
        self.mode = mode
        self.timevec = []
        self.footleft = []
        self.footright = []
        self.jointsleft = []
        self.jointsright = []
        self.transmatleft = []
        self.transmatright = []


def main():
    steps = []
    body_position = np.array([[0,0,0]])
    time_vector = np.array([])# [m]

    #################################################################
    ## PART 1: GO TO START POSE
    #################################################################
    #First, lower the body of the robot and move its COM to the right foot
    #This is a harcoded value that can be modified later

    starting_body_points = np.array([
                     [0,0],
                     [0,-0.67*Y_BODY_TO_FEET],
                     [Z_ROBOT_STATIC, Z_ROBOT_WALK]]
                    )

    timepoints = [0,1]

    time_vector = np.linspace(timepoints[0],timepoints[1], math.floor(1/SAMPLE_TIME))

    #Applying linear interpolation to generate first trajectory
    m_y = (starting_body_points[1][1] - starting_body_points[1][0])/(timepoints[1]-timepoints[0])
    m_z = (starting_body_points[2][1] - starting_body_points[2][0])/(timepoints[1]-timepoints[0])
    q_x = 0
    q_y = interpolate.interp1d(time_vector, 
                               time_vector*m_y*SAMPLE_TIME,
                               fill_value="extrapolate")
    q_z = interpolate.interp1d(time_vector, 
                               starting_body_points[2][0] + time_vector*m_z*SAMPLE_TIME,
                               fill_value="extrapolate")
    
    #Generating point array for trajectory
    for i in np.arange(0,len(time_vector)):
        aux = np.array([[q_x, q_y(i), q_z(i)]])
        body_position = np.concatenate((body_position,aux), axis=0)
    body_position = np.delete(body_position, 0, axis=0)

    rel_position = body_position - [0, -Y_BODY_TO_FEET, 0]

    foot_right = np.array([[0,-Y_BODY_TO_FEET,0]])
    foot_left = np.array([[0, Y_BODY_TO_FEET,0]])
    
    for i in np.arange(0, len(time_vector)-1):
        foot_right = np.concatenate((foot_right, np.array([[0, -Y_BODY_TO_FEET, 0]])), axis=0)
        foot_left = np.concatenate((foot_left, np.array([[0, Y_BODY_TO_FEET, 0]])), axis=0)
    
    print(f"Absolute COM shape is: {rel_position.shape}")
    print(rel_position)
    print(f"Right foot matrix shape: {foot_right.shape}")
    print(foot_right)
    print(f"Left foot matrix shape: {foot_left.shape}")
    print(foot_left)

    start_pose_step = Step(mode="DoubleSupport")
    start_pose_step.timevec = time_vector
    start_pose_step.footleft = foot_left
    start_pose_step.footright = foot_right

    steps.append(start_pose_step)
    
    ax = plt.figure().add_subplot(projection='3d')
    ax.axes.set_xlim3d(left=0, right=0.4) 
    ax.axes.set_ylim3d(bottom=-0.2, top=0.2) 
    ax.axes.set_zlim3d(bottom=0, top=0.8) 

    #################################################################
    ## PART 2: MAKE A HALF STEP
    #################################################################
    # Store left and right foothold position (left:odd, right:even)

    final_halfstep_position = np.array([[0.05],[0],[Z_ROBOT_WALK]])
    starting_body_points = np.concatenate((starting_body_points, final_halfstep_position), axis=1)

    print(starting_body_points)

    m_x = (starting_body_points[0][2] - starting_body_points[0][1])/(timepoints[1]-timepoints[0])
    m_y = (starting_body_points[1][2] - starting_body_points[1][1])/(timepoints[1]-timepoints[0])

    q_x = interpolate.interp1d( time_vector, 
                                starting_body_points[0][1] + time_vector*m_x*SAMPLE_TIME,
                                fill_value="extrapolate")
    q_z = Z_ROBOT_WALK
    q_y = interpolate.interp1d(time_vector, 
                               starting_body_points[1][1] + time_vector*m_y*SAMPLE_TIME,
                               fill_value="extrapolate")

    body_position = np.array([[0,0,0]])
    for i in np.arange(0,len(time_vector)):
        aux = np.array([[q_x(i), q_y(i), q_z]])
        body_position = np.concatenate((body_position,aux), axis=0)
    body_position = np.delete(body_position, 0, axis=0)

    initial_right_foot_pos  = np.array([0, -Y_BODY_TO_FEET, 0])
    initial_left_foot_pos   = np.array([0, Y_BODY_TO_FEET, 0])
    final_right_foot_pos    = np.array([0, -Y_BODY_TO_FEET, 0])
    final_left_foot_pos     = np.array([STEP_LENGTH/2, Y_BODY_TO_FEET, 0])

    first_step = Step(mode="RightSupport")
    first_step.timevec = np.linspace(timepoints[0],timepoints[1], math.floor(1/SAMPLE_TIME))
    
    first_step.footleft = getFootSwingTraj(initial_left_foot_pos, final_left_foot_pos, stepHeight, first_step.timevec)
    first_step.footright = np.full((len(first_step.timevec), 3), initial_right_foot_pos)

    print(first_step.footleft)
    print(first_step.footleft.shape)
    print(first_step.footright)
    print(first_step.footleft.shape)
    print(body_position)
    print(body_position.shape)

    ax.plot(body_position[:,0],body_position[:,1],body_position[:,2], "o")
    ax.plot([initial_left_foot_pos[0]],
            [initial_left_foot_pos[1]],
            [initial_left_foot_pos[2]], "o")

    ax.plot([initial_right_foot_pos[0]],
            [initial_right_foot_pos[1]],
            [initial_right_foot_pos[2]], "o")

    ax.plot([final_left_foot_pos[0]],
            [final_left_foot_pos[1]],
            [final_left_foot_pos[2]], "o")

    ax.plot([final_right_foot_pos[0]],
            [final_right_foot_pos[1]],
            [final_right_foot_pos[2]], "o")

    ax.plot(first_step.footleft[:,0],
            first_step.footleft[:,1],
            first_step.footleft[:,2], "o")

    #################################################################
    ## PART 3: MAKE CONSECUTIVE STEPS
    #################################################################

    #TODO COMENZAR A PARTIR DE AQUI CON EL SIGUIENTE STEP, EL ANTERIOR YA ESTA
    initial_right_foot_pos  = final_right_foot_pos
    initial_left_foot_pos   = final_left_foot_pos

    final_right_foot_pos    = np.array([STEP_LENGTH, -Y_BODY_TO_FEET, 0])

    y_0 = -Y_BODY_TO_FEET
    [dy0, x_0, dx0, single_support_time] = findInitialConditions(STEP_LENGTH,
                                                                 ROBOT_VEL_X,
                                                                 y_0,
                                                                 Z_ROBOT_WALK,
                                                                 G)

    #Initial conditions vector that guarantees symetric trajectories in LIPM

    # TODO: REVISAR EL VECTOR DE ESTADOS Y EL OUTPUT EN Y, EN X y Z estan bien

    state0 = [x_0, dx0, y_0, dy0]
    print(state0)
    tFinal = single_support_time
    print(single_support_time)
    steptimeVector = np.linspace(0, tFinal, math.floor(tFinal/SAMPLE_TIME))
    #Simulation loop
    nSteps = len(steptimeVector)
    states = np.array([state0])
    for i in range(0, nSteps-1):
        dx_next = states[i][1] + SAMPLE_TIME*((states[i][0])*G/Z_ROBOT_WALK)
        x_next = states[i][0] + SAMPLE_TIME*states[i][1]
        dy_next = states[i][3] + SAMPLE_TIME*((states[i][2])*G/Z_ROBOT_WALK)
        y_next = states[i][2] + SAMPLE_TIME*states[i][3]
        states = np.concatenate((states, [[x_next, dx_next, y_next, dy_next]]), axis=0)
    aux = zip(np.add(states[:,0],initial_left_foot_pos[0]), np.add(states[:,2],initial_left_foot_pos[0]), [Z_ROBOT_WALK for i in states])
    
    body_position = np.concatenate((body_position,[list(i) for i in list(aux)]), axis=0)
    [state0, initial_left_foot_pos[0], initial_left_foot_pos[1]] = changeLeg(states[-1], body_position)
    # foot_position = np.concatenate((foot_position, [[initial_left_foot_pos[0]],[initial_left_foot_pos[1]],[0]]), axis=1)
    
    print(body_position)

    ax.plot(body_position[:,0],body_position[:,1],body_position[:,2], "o")
    # ax.plot(foot_position[0,:],foot_position[1,:], foot_position[2,:], "o")
    
    plt.show()

    exit()

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
    print("singlesupportTIme is ", singlesupport_t)

    tf = singlesupport_t/2

    dy0 = -y_0/s * math.sinh(tf/s) / math.cosh(tf/s)

    return [dy0, x_0, dx0, singlesupport_t]

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

def getFootSwingTraj(initial_foot_position, final_foot_position, swing_height, timeVector):
    
    x_0 = initial_foot_position[0]
    x_1 = final_foot_position[0]

    h = x_0 + (x_1 - x_0)/2
    print(f"h = {h}")
    k = swing_height
    print(f"k = {k}")
    a = -k/((-h)**2)
    print(f"a = {a}")
    x_t = lambda t: x_0 + (x_1 - x_0) * t
    z = lambda x: a*((x-h)**2) + k
    
    print(z(x_0))
    print(z(x_1))

    swingFootTrajectory = np.array([[0, 0, 0]])

    for i in timeVector:
        aux = np.array([[x_t(i), initial_foot_position[1], z(x_t(i))]])
        swingFootTrajectory = np.concatenate((swingFootTrajectory,aux), axis=0)
    swingFootTrajectory = np.delete(swingFootTrajectory, 0, axis=0)

    return swingFootTrajectory

def cubicPolyTraj():
    pass
    
if __name__ == "__main__":
    exit(main())