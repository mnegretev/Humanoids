#!/usr/bin/env python
import os
import json
import rospy

from std_msgs.msg import Float32MultiArray

t0 = 0
first_sample = True

def callback(data):
    global first_sample, t0
    ti = rospy.get_time()
    if (first_sample == True):
        t0 = ti
        first_sample = False
        
    positions_list = list(data.data)
    positions_list.append(ti-t0)
    t0 = ti

    with open('positions_data.txt', 'a') as filehandle:
        json.dump(positions_list, filehandle)
        filehandle.write("\n")

def catch_ball_positions():
    rospy.init_node('catch_ball_positions', anonymous=True)
    rospy.Subscriber("/vision/ball_kinematics/ball_kinematics", Float32MultiArray, callback)
    
    if os.path.exists("positions_data.txt"):
        os.remove("positions_data.txt")

    rospy.spin();

if __name__ == '__main__':
    catch_ball_positions()
