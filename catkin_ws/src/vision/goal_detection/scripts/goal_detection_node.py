#!/usr/bin/env python
import rospy
import numpy
import cv2
import ros_numpy
import math
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from cv_bridge import CvBridge   
from matplotlib import pyplot as plt

def callback_image (msg):
    print ("imagen recibida")
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    gradients_laplacian = cv2.Laplacian(cv_image, -1)

    canny_output = cv2.Canny(cv_image, 80, 150)
    cv2.imshow('Canny', canny_output)

    cv2.imshow("imagen",cv_image)
    cv2.waitKey(10)


def main ():
    rospy.init_node("goal_detection_node")  
    rospy.Subscriber("/hardware/camera/image", Image, callback_image)
    rospy.spin()

if __name__=="__main__":
    main()
