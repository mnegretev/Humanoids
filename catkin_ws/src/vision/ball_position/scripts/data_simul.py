#!/usr/bin/env python
import rospy
from random import gauss
from std_msgs.msg import Float32MultiArray
    
counter = 0

def data_simul():
    global counter
    pub = rospy.Publisher("/vision/ball_position/ball_position", Float32MultiArray, queue_size=10)
    rospy.init_node('data_simul', anonymous=True)
    rate = rospy.Rate(30)

    positions = Float32MultiArray() 
    list = []
    
    g  = 9.81
    Mg = 0.15
    dt  = 0.033333
    r = 0.04
    t = 0
    
    px = -1.2 
    py = -1.2
    velx = 2#1.9#2.66
    vely = 2#1.9#2.66

    px0 = -1.2
    py0 = -1.2
    velx0 = 1.9
    vely0 = 1.9

    print "Starting data_simul with vy:", vely, "m/s"
    while not rospy.is_shutdown():
        counter += 1

        '''velx = velx - Mg * g * dt
        vely = vely - Mg * g * dt
        px   = px + velx * dt
        py   = py + vely * dt'''
        py_ = py
        px = px0 + velx0*t - 0.5*Mg*g*t**2
        py = py0 + vely0*t - 0.5*Mg*g*t**2
        vely = (py - py_)/dt

        list.append(round(px, 3))
        list.append(round(py, 3))
        list.append(round(gauss(px, r),3))
        list.append(round(gauss(py, r),3))

        #print list,"\tvely:", vely


        positions.data = list
        pub.publish(positions)
        rate.sleep()

        list = []
        
        t += dt

        if vely < 0.0: break

if __name__ == '__main__':
    data_simul()
    print "Data sent:", counter
