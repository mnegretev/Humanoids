#!/usr/bin/env python
import math
import sys
import rospy
import numpy
import tf
import tf.transformations as tft
from std_msgs.msg import Float64MultiArray
from manip_msgs.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

prompt = ""

def get_polynomial_trajectory(q_initial, q_final, qp_initial=0, qp_final=0, qpp_initial=0, qpp_final=0, duration=1.0, time_step=0.05):
    #
    # This function calculates a polynomial trajectory for a single variable.
    # It is intended to be called N times for a N-DoF trajectory. 
    #
    t = duration
    A = [[   t**5,    t**4,   t**3, t**2, t, 1],
         [ 5*t**4,  4*t**3, 3*t**2,  2*t, 1, 0],
         [20*t**3, 12*t**2,    6*t,    2, 0, 0],
         [      0,       0,      0,    0, 0, 1],
         [      0,       0,      0,    0, 1, 0],
         [      0,       0,      0,    2, 0, 0]]
    A = numpy.asarray(A)
    B = [[q_final  ],
         [qp_final ],
         [qpp_final],
         [q_initial  ],
         [qp_initial ],
         [qpp_initial]]
    B = numpy.asarray(B)
    X = numpy.dot(numpy.linalg.inv(A),B)
    a5, a4, a3, a2, a1, a0 = X[0,0], X[1,0], X[2,0], X[3,0], X[4,0], X[5,0]
    T = numpy.arange(0, t, time_step)
    Q = numpy.zeros(len(T))
    for i in range(len(T)):
        Q[i] = a5*T[i]**5 + a4*T[i]**4 + a3*T[i]**3 + a2*T[i]**2 + a1*T[i] + a0

    return T, Q
    
def get_polynomial_trajectory_multi_dof(Q_start, Q_end, Qp_start=[], Qp_end=[], Qpp_start=[], Qpp_end=[], duration=1.0, time_step=0.05):
    Q = []
    T = []
    if len(Qp_start) == 0:
        Qp_start = numpy.zeros(len(Q_start))
    if len(Qpp_start) == 0:
        Qpp_start = numpy.zeros(len(Q_start))
    if len(Qp_end) == 0:
        Qp_end = numpy.zeros(len(Q_end))
    if len(Qpp_end) == 0:
        Qpp_end = numpy.zeros(len(Q_end))
    for i in range(len(Q_start)):
        T, Qi = get_polynomial_trajectory(Q_start[i], Q_end[i], Qp_start[i], Qp_end[i], Qpp_start[i], Qpp_end[i], duration, time_step)
        Q.append(Qi)
    Q = numpy.asarray(Q)
    Q = Q.transpose()
    return Q,T


def get_trajectory_time(p1, p2, speed_factor):
    p1 = numpy.asarray(p1)
    p2 = numpy.asarray(p2)
    m = max(numpy.absolute(p1 - p2))
    return m/speed_factor + 0.5


def callback_polynomial_trajectory(req):
    print(prompt+"Calculating polynomial trajectory")
    t  = req.duration if req.duration > 0 else get_trajectory_time(p1, p2, 0.25)
    Q, T = get_polynomial_trajectory_multi_dof(req.p1, req.p2, req.v1, req.v2, req.a1, req.a2, t, req.time_step)
    trj = JointTrajectory()
    trj.header.stamp = rospy.Time.now()
    for i in range(len(Q)):
        p = JointTrajectoryPoint()
        p.positions = Q[i]
        p.time_from_start = rospy.Duration.from_sec(T[i])
        trj.points.append(p)
    resp = GetPolynomialTrajectoryResponse()
    resp.trajectory = trj
    return resp
        
    

def main():
    global joint_names, max_iterations, joints, transforms, prompt
    print("INITIALIZING TRAJECTORY PLANNER NODE BY MARCOSOFT...")
    rospy.init_node("trajectory_planner")
    prompt = rospy.get_name().upper() + ".->"
    rospy.Service("/manipulation/polynomial_trajectory", GetPolynomialTrajectory, callback_polynomial_trajectory)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()


