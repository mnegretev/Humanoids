#!/usr/bin/env python
import rospy
import numpy as np
import cv2
#import ros_numpy
import math
import random
import argparse
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from cv_bridge import CvBridge   
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point32

def calculate_distance(center1, center2):
    return math.sqrt((center1[0] - center2[0])**2 + (center1[1] - center2[1])**2)
    
center=None
def callback_image (msg):
    global center, prev_center

    centroid_msg = Point32()  # Create a Point32 message object
    bridge = CvBridge()     
    #For simul: desired_encoding='bgr8', for real: desired_encoding='rgb8'
    if rospy.get_param("rgb8"):
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')#source file
    else:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #Para espacio HSV
    #REAL                   SIMUL
    # h_min = 25          h_min = 0    
    # h_max = 44          h_max = 70
    # s_min = 65          s_min = 215
    # s_max = 160         s_max = 255
    # v_min = 70          v_min = 0
    # v_max = 255         v_max = 255
    if rospy.get_param("use_hsv"):
        hsv_image= cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
        h_min = rospy.get_param("hsv_h_min")
        h_max = rospy.get_param("hsv_h_max")
        s_min = rospy.get_param("hsv_s_min")
        s_max = rospy.get_param("hsv_s_max")
        v_min = rospy.get_param("hsv_v_min")
        v_max = rospy.get_param("hsv_v_max")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv_image, lower, upper)
        masked_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    else:
    #Para HLS
        hls_image= cv2.cvtColor(cv_image, cv2.COLOR_BGR2HLS)    
        h_min = rospy.get_param("hls_h_min")
        h_max = rospy.get_param("hls_h_max")
        l_min = rospy.get_param("hls_l_min")
        l_max = rospy.get_param("hls_l_max")
        s_min = rospy.get_param("hls_s_min")
        s_max = rospy.get_param("hls_s_max") 
        lower = np.array([h_min, l_min, s_min])
        upper = np.array([h_max, l_max, s_max])
        mask = cv2.inRange(hls_image, lower, upper)
        masked_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
    blur=cv2.GaussianBlur(gray,(7,7),0)
    minDist =500
    param1 = 50
    param2 = 50 #smaller value-> more false circles
    minRadius = 5
    maxRadius = 50
    ball_detected=False
    distance_threshold = 5
    # docstring of HoughCircles: HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) -> circles
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1.5, minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)
    if circles is not None:
        for i in circles[0,:]:
            #centers.append(center) 
            # draw the outer circle
            cv2.circle(cv_image,(int (i[0]),int (i[1])), int (i[2]),(0,255,0),2)
            # draw the center of the circle
            cv2.circle(cv_image,(int (i[0]),int (i[1])),2,(0,0,255),3)
            if center is None:
                prev_center= center = (int (i[0]),int (i[1]))
            else:
                prev_center = center
                center = (int (i[0]), int (i[1]))
            #print("center: ",center)           
            #print("previous center: ",prev_center)
        if prev_center is not None:
            #print("*********",prev_center)
            
            distance = calculate_distance(prev_center, center)        
            #print("Distance = ",distance)
            if distance < distance_threshold:
                ball_detected=True
                #print("Ball detected.")
                # Publish the center coordinates as a Point32 message
                centroid_msg.x = center[0]
                centroid_msg.y = center[1]
                centroid_pub.publish(centroid_msg)
            if distance > distance_threshold:
                ball_detected=False
                
            
     
    msg = bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
    ball_image_pub.publish(msg)
    msg2=bridge.cv2_to_imgmsg(blur, encoding='8UC1')
    hue_image_pub.publish(msg2)
   
    #cv2.imshow('Original Image', cv_image) #hls image
    #cv2.imshow("blur image",blur) #Blur image
    #cv2.imshow("image mask",masked_image) #Masked image
    cv2.waitKey(10)
   
def main ():
    global centroid_pub, ball_image_pub,hue_image_pub
    rospy.init_node("ball_detection_node")  
    rospy.Subscriber("/hardware/camera/image", Image, callback_image)
    centroid_pub = rospy.Publisher("/centroid_publisher", Point32, queue_size=1)
    ball_image_pub = rospy.Publisher("/ball_image", Image, queue_size=1)
    hue_image_pub = rospy.Publisher("/hue_image", Image, queue_size=1)
    rospy.spin()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
