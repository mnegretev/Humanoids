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

"""----------Datos----------"""

esc = 1/10

degree = np.pi/180.0

def limites(ang):
    if (ang < -np.pi):
        ang = ang + np.pi
        if (ang > -np.pi):
            ang = ang
    if (ang > np.pi):
        ang = ang - np.pi
        if(ang < np.pi):
            ang = ang
    return ang
""" Denavit - Hartenberg """
def T_a_x(x):
    Tx = np.matrix([[1,0,0,x],
                   [0,1,0,0],
                   [0,0,1,0],
                   [0,0,0,1]])
    return Tx

def T_d_z(z):
    Tz = np.matrix([[1,0,0,0],
                   [0,1,0,0],
                   [0,0,1,z],
                   [0,0,0,1]])
    return Tz

def T_x_alfa(theta_x):
    T_th_x = np.matrix([[ 1 ,      0          ,        0         ,   0],
                         [0 , np.cos(theta_x) , -np.sin(theta_x) ,   0],
                         [0 , np.sin(theta_x) , np.cos(theta_x)  ,   0],
                         [0 ,      0          ,        0         ,   1]])
    return T_th_x

def T_z_th(theta_z):
    T_th_z = np.matrix([[ np.cos(theta_z)  ,   -np.sin(theta_z) ,   0  ,   0],
                        [ np.sin(theta_z)  ,    np.cos(theta_z) ,   0  ,   0],
                        [        0         ,           0        ,   1  ,   0],
                        [        0         ,           0        ,   0  ,   1]])
    return T_th_z

def DH(th_i,d_i,a_i,alfa_i):
    A = T_z_th(th_i) * T_d_z(d_i) * T_a_x(a_i) * T_x_alfa(alfa_i)
    return A


def CinematicaInv(x,*tray):#,*dato):
    tray_LF,aaa = tray
    A_0_T = Tz.Tras_x(0.1)*Tz.Tras_y(0.0)*Tz.Tras_z(0.52)*Tz.T_Roll(0.0)*Tz.T_Pitch(0.0)*Tz.T_Yaw(0.0)#TRUNK
    A_1_2 = DH(x[0],-0.12,0.055,0.0)#YAW <origin xyz="0.055 0.00 -0.12"/>
    A_2_3 = DH(x[1],-0.0125,0.0,-90*degree)#ROLL <origin rpy="-1.570795 0 0" xyz="0 0 -0.0125"/>
    A_3_4 = DH(x[2] + 90*degree ,-0.01,0.0,90*degree)#PITCH <origin rpy="1.570795 -0.5236 1.570795" xyz="0 0 -0.01"/>
    A_4_5 = DH(x[3] - 60*degree ,0.0,2*0.102,0.0)#PITCH <origin rpy="0 0 -1.0472" xyz="0.102 0 0"/>
    A_5_6 = DH(x[4],0.0,2*0.102,0.0)#PITCH <origin rpy="0 0 0" xyz="0.102 0 0"/>
    A_6_7 = DH(x[5] - 90.0*degree,0.0,0.0,30*degree)#ROLL <origin rpy="-1.570795 0 0.5236" xyz="0 0 0"/>
    #A_7_F = Tz.Tras_x(0.0)*Tz.Tras_y(0.0)*Tz.Tras_z(0.0)*Tz.T_Roll(0.0)*Tz.T_Pitch(0.0)*Tz.T_Yaw(0.0)#FOOT

    # A_0_T = Tz.Tras_x(0.0)*Tz.Tras_y(0.0)*Tz.Tras_z(0.52)*Tz.T_Roll(0.0)*Tz.T_Pitch(0.0)*Tz.T_Yaw(0.0)#TRUNK
    # A_1_2 = DH(x[0],0.0,0.0,0.0)#YAW <origin xyz="0.055 0.00 -0.12"/>
    # A_2_3 = DH(x[1],0.0,0.0,0.0)#ROLL <origin rpy="-1.570795 0 0" xyz="0 0 -0.0125"/>
    # A_3_4 = DH(x[2],0.0,0.0,0.0)#PITCH <origin rpy="1.570795 -0.5236 1.570795" xyz="0 0 -0.01"/>
    # A_4_5 = DH(x[3],0.0,0.0,0.0)#PITCH <origin rpy="0 0 -1.0472" xyz="0.102 0 0"/>
    # A_5_6 = DH(x[4],0.0,0.0,0.0)#PITCH <origin rpy="0 0 0" xyz="0.102 0 0"/>
    # A_6_7 = DH(x[5],0.0,0.0,0.0)#ROLL <origin rpy="-1.570795 0 0.5236" xyz="0 0 0"/>

    Trans_I = A_0_T * A_1_2 * A_2_3 * A_3_4 * A_4_5 * A_5_6 * A_6_7
    #print(Trans_I)
    Trans_D = Tz.Tras_x(0.1)*Tz.Tras_y(0.0)*Tz.Tras_z(tray_LF)*Tz.T_Roll(90.0*degree)*Tz.T_Pitch(0.0)*Tz.T_Yaw(0.0*degree)#FOOT <origin rpy="1.570795 0 -1.570795" xyz="0.02 0 0"/>
    
    out = [Trans_I[0,3] - Trans_D[0,3]]
    out.append(Trans_I[1,3] - Trans_D[1,3])
    out.append(Trans_I[2,3] - Trans_D[2,3])
    out.append(Trans_I[0,0] - Trans_D[0,0])
    out.append(Trans_I[1,1] - Trans_D[1,1])
    out.append(Trans_I[2,2] - Trans_D[2,2])    
    #print(out)
    
    return out

def inverse_kinematics_joints():
    pub = rospy.Publisher('joint_states',JointState, queue_size=10)
    rospy.init_node('inverse_kinematics')
    rate = rospy.Rate(10) # 10hz

    msg = JointState()
    msg.name = ["left_joint_leg_yaw","left_joint_leg_pitch",
                "left_joint_leg_roll","left_joint_knee_pitch",
                "left_joint_ankle_pitch","left_joint_ankle_roll",
                "right_joint_leg_yaw","right_joint_leg_pitch",
                "right_joint_leg_roll","right_joint_knee_pitch",
                "right_joint_ankle_pitch","right_joint_ankle_roll"]
    #print (msg)
    tray = 0.0
    while not rospy.is_shutdown():
        for i in range(1,10):
            tray_l = np.linspace(0.4,0.52,10)
            tray = (tray_l[i],0.2)
            thetas = fsolve(CinematicaInv,[0,0,0,0,0,0], args = tray,maxfev = 10)
            #thetas = fsolve(CinematicaInv,[0,0,0],args = tray, maxfev = 50)
            msg.header.stamp = rospy.Time.now()
            thetas[0] = limites(thetas[0])
            thetas[1] = limites(thetas[1])
            thetas[2] = limites(thetas[2])
            thetas[3] = limites(thetas[3])
            thetas[4] = limites(thetas[4])
            thetas[5] = limites(thetas[5])
            msg.position = [thetas[0],thetas[1],thetas[2],thetas[3],thetas[4],thetas[5],0,0,0,0,0,0]
            pub.publish(msg)
            print(thetas,'trayectoria: ',tray_l[i])
            rate.sleep()

if __name__ == '__main__':
    try:
        inverse_kinematics_joints()
    except rospy.ROSInterruptException:
        pass