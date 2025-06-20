#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import os
from ctrl_msgs.srv import Getup

def main():
    rospy.wait_for_service('getup')
    try:
        getup=rospy.ServiceProxy('getup', Getup)
        succes= getup(path)
        print(succes)
    except rospy.ServiceException as e:
        print("Something go wrong:" %e)


if __name__ == '__main__':
    main()
