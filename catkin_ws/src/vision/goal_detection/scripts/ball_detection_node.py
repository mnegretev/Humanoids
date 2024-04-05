#!/usr/bin/env python
import rospy
import numpy
import cv2
import ros_numpy
import math
import random
import argparse
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from cv_bridge import CvBridge   
from matplotlib import pyplot as plt

def callback_image (msg):

    bridge = CvBridge()     
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')#source file
    hls_image= cv2.cvtColor(cv_image, cv2.COLOR_BGR2HLS)
    # Define thresholds (adjust based on your image data)
    lower_saturation = 80  # Minimum saturation to remove noise
    lower_lightness = 120  # Minimum lightness to avoid shadows
    upper_lightness = 255  # Maximum lightness to avoid overexposure

    # Threshold the S and L channels
    mask_s = cv2.inRange(hls_image, (0, lower_saturation, 0), (255, 255, 255))
    mask_l = cv2.inRange(hls_image, (0, 0, lower_lightness), (255, 255, upper_lightness))

    # Combine the masks using bitwise AND to keep only pixels meeting both conditions
    mask = cv2.bitwise_and(mask_s, mask_l)
   
    # Filter the original image based on the Hue range for the soccer ball
    hue_mask = cv2.inRange(hls_image, (50, 0, 0), (100, 255, 255)) #50 and 100 are for green color MUST CHANGE FOR FIFA BALL

    # Combine the mask from HSV filtering with the mask from S & L thresholding
    # using bitwise AND to refine the result
    refined_mask = cv2.bitwise_and(mask, hue_mask)

    blur=cv2.GaussianBlur(refined_mask,(7,7),0)
    
    ball_detected=bool
    return ball_detected

def main ():
    #if ball_detected=True :
        
    global pub_head_goal,centroid_pub
    pub_head_goal = rospy.Publisher("/hardware/head_goal_pose", Float32MultiArray, queue_size=1)
    centroid_pub = rospy.Publisher("/centroid_publisher", Point32, queue_size=1)
    rospy.init_node("ball_detection_node")  
    rospy.Subscriber("/hardware/camera/image", Image, callback_image)
    rospy.spin()

if __name__=="__main__":
    main()