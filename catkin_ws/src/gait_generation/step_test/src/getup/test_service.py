#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import os
from ctrl_msgs.srv import Getup

def main():
    path = rospy.get_param("path", "/home/k1000/Humanoids/catkin_ws/src/gait_generation/step_test/src/getup/poses")
    rospy.wait_for_service('getup')
    try:
        getup=rospy.ServiceProxy('getup', Getup)
        succes= getup(path)
        print(succes)
    except rospy.ServiceException as e:
        print("Something go wrong:" %e)


if __name__ == '__main__':
    main()
