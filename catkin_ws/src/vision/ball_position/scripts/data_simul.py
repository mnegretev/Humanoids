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
    r = 0.06
    
    px = -1.2 
    py = -1.2
    velx = 1.9#2.66
    vely = 1.9#2.66


    print "Starting data_simul with vy:", vely, "m/s"
    while not rospy.is_shutdown():
        counter += 1
        velx = velx - Mg * g * dt
        vely = vely - Mg * g * dt
        px   = px + velx * dt
        py   = py + vely * dt



        list.append(round(px, 3))
        list.append(round(py, 3))
        list.append(round(gauss(px, r),3))
        list.append(round(gauss(py, r),3))

        #print list,"\tvely:", vely


        positions.data = list
        pub.publish(positions)
        rate.sleep()

        list = []

        if vely <= 0.01:
            break

if __name__ == '__main__':
    data_simul()
    print "Data sent:", counter
