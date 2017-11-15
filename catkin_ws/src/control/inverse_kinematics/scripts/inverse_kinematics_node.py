#!/usr/bin/env python

"""
Created on Tue Sep 26 20:54:16 2017

@author: Allen
"""
import rospy
from std_msgs.msg import Header
import numpy as np
import math as m
from scipy.optimize import fsolve
from sensor_msgs.msg import JointState
from Tz import Tz

Tz = Tz()

""" ----------Datos----------"""

esc = 1/10

degree = np.pi/180
b_11 = 0#180*degree
b_21 = 0#330*degree
b_31 = 0#60*degree
b_41 = 0#40*degree
b_51 = 0#45*degree
b_61 = 0#180*degree

b_12 = b_11;
b_22 = 30*degree
b_32 = 300*degree
b_42 = 315*degree
b_52 = 315*degree
b_62 = 0*degree

b_7 = 75*degree
b_8 = 270*degree

#x_B = 0
y_B = esc * 38.5
z_B = esc * 234.37

th_B = 0
fi_B = 0
psi_B = 0

x_left_foot = 0.0
y_left_foot = 0.0
z_left_foot = -0.4
th_left_foot = 0
fi_left_foot = 0
psi_left_foot = 0

x_2 = esc * 44.54
y_2 = 0
z_2 = 0
th_2 = 0
fi_2 = 0
psi_2 = 0

#th_11 = 0
#th_12= 0
#th_21= 0
#th_22= 0
#th_31= 0
#th_32= 0
#th_41= 0
#th_42= 0
#th_51= 0
#th_52= 0
#th_61= 0
#th_62= 0


y_B_j11 = 0.055#esc * 44.54
z_B_j11 = -0.12
z_j11_j21 = -0.0125#esc * 54.58
x_j21_j31 = -0.02#esc * 29.08
x_j31_j41 = 0.102#esc * 74.60
x_j41_j51 = 0.102#esc * 57.29
x_j51_j61 = -0.024#esc * 54.97
x_j61_left_foot = -0.05#esc * 23.06

def CinematicaInv(x,*CoM):
    x_B,y_B = CoM

    T_0_B = 1#Tz.T_Roll(th_B + 0*degree)*Tz.T_Pitch(fi_B)*Tz.T_Yaw(psi_B)*Tz.Tras_x(x_B)*Tz.Tras_y(y_B)*Tz.Tras_z(z_B)
    """Joints"""
    T_B_j11 = Tz.T_Yaw(x[0] + 0*degree)*Tz.Tras_y(y_B_j11)*Tz.Tras_z(z_B_j11)# """Yaw -- > T_Yaw"""HIP
    T_j11_j21 = Tz.T_Roll(x[1] + 0*degree) * Tz.Tras_z(z_j11_j21)# """Roll -- > T_Roll""" HIP
    T_j21_j31 = Tz.T_Pitch(x[2] + 0*degree) * Tz.Tras_x(x_j21_j31)#"""Pitch -- > T_Pitch"""HIP
    T_j31_j41 = Tz.T_Pitch(x[3] + 0*degree) * Tz.Tras_x(x_j31_j41)#"""Pitch -- > T_Pitch"""KNEE
    T_j41_j51 = Tz.T_Pitch(x[4] - 0*degree) * Tz.Tras_x(x_j41_j51)#"""Pitch -- > T_Pitch"""ANKLE
    T_j51_j61 = Tz.T_Roll(x[5] + 0*degree) * Tz.Tras_x(x_j51_j61)#"""Roll -- > T_Roll"""ANKLE
    T_j61_01  = Tz.T_Yaw(0*degree) * Tz.T_Pitch(0*degree) *Tz.Tras_y(x_j61_left_foot)

    T_01 = Tz.Tras_x(x_left_foot)*Tz.Tras_y(y_left_foot)*Tz.Tras_z(z_left_foot)*Tz.T_Roll(th_left_foot)*Tz.T_Pitch(fi_left_foot)*Tz.T_Yaw(psi_left_foot)
    
    PosI1 = T_0_B * T_B_j11 * T_j11_j21 * T_j21_j31 * T_j31_j41 * T_j41_j51 * T_j51_j61 * T_j61_01
    PosI2 = T_01
    
    #print(PosI1)
    
    out = [PosI1[0,3] - PosI2[0,3]]
    out.append(PosI1[1,3] - PosI2[1,3])
    out.append(PosI1[2,3] - PosI2[2,3])
    out.append(PosI1[0,0] - PosI2[0,0])
    out.append(PosI1[1,1] - PosI2[1,1])
    out.append(PosI1[2,2] - PosI2[2,2])
    
    #print(out)
    
    return out

def inverse_kinematics_joints():
    pub = rospy.Publisher('joint_states',JointState, queue_size=10)
    rospy.init_node('inverse_kinematics')
    rate = rospy.Rate(10) # 10hz

    msg = JointState()
    msg.name = ["left_joint_leg_yaw","left_joint_leg_pitch","left_joint_leg_roll",
                "left_joint_knee_pitch","left_joint_ankle_pitch","left_joint_ankle_roll",
                "right_joint_leg_yaw","right_joint_leg_pitch","right_joint_leg_roll",
                "right_joint_knee_pitch","right_joint_ankle_pitch","right_joint_ankle_roll"]
    #print (msg)
    x_B = 0.0
    y_B = 0.0
    while not rospy.is_shutdown():
        x_B = 0.0#x_B + 0.1
        y_B = 0.0#x_B
        CoM = (x_B,y_B)
        thetas = fsolve(CinematicaInv,[0,0,0,0,0,0],args = CoM,maxfev = 30)
        msg.header.stamp = rospy.Time.now()
        msg.position = [thetas[0],thetas[1],thetas[2],thetas[3],thetas[4],thetas[5],0,0,0,0,0,0]
        #msg.position = [0,thetas[1],0,0,0,0,0,0,0,0,0,0]
        pub.publish(msg)
        print(thetas)
        
        if(x_B > 1.0):
            x_B = 0
        rate.sleep()

if __name__ == '__main__':
    try:
        inverse_kinematics_joints()
    except rospy.ROSInterruptException:
        pass