#!/usr/bin/env python
import rospy
import numpy
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
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')#source file
    hls_image= cv2.cvtColor(cv_image, cv2.COLOR_BGR2HLS)

    # Define thresholds (adjust based on your image data)
    # lower_saturation = 60  # Minimum saturation to remove noise
    # lower_lightness = 100  # Minimum lightness to avoid shadows
    # upper_lightness = 255  # Maximum lightness to avoid overexposure
    lower_saturation = 70  # Minimum saturation to remove noise
    lower_lightness = 90  # Minimum lightness to avoid shadows
    upper_lightness = 255  # Maximum lightness to avoid overexposure
    # Threshold the S and L channels
    mask_s = cv2.inRange(hls_image, (0, lower_saturation, 0), (255, 255, 255))
    mask_l = cv2.inRange(hls_image, (0, 0, lower_lightness), (255, 255, upper_lightness))

    # Combine the masks using bitwise AND to keep only pixels meeting both conditions
    mask = cv2.bitwise_and(mask_s, mask_l)
   
    # Filter the original image based on the Hue range for the soccer ball
    # hue_mask = cv2.inRange(hls_image, (0, 205, 0), (250, 250, 250)) #50 and 100 are for green color MUST CHANGE FOR FIFA BALL
    hue_mask = cv2.inRange(cv_image, (45, 0, 25), (100, 250, 250)) #50 and 100 are for green color MUST CHANGE FOR FIFA BALL
    # Combine the mask from HSV filtering with the mask from S & L thresholding
    # using bitwise AND to refine the result
    refined_mask = cv2.bitwise_and(mask, hue_mask)
    blur=cv2.GaussianBlur(refined_mask,(7,7),0)

    minDist =500
    param1 = 50
    param2 = 30 #smaller value-> more false circles
    minRadius = 5
    maxRadius = 30
    ball_detected=False
    distance_threshold = 5 
    # docstring of HoughCircles: HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) -> circles
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1.5, minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)
    if circles is not None:
        for i in circles[0,:]:
            #centers.append(center) 
            # draw the outer circle
            cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(cv_image,(i[0],i[1]),2,(0,0,255),3)
            if center is None:
                prev_center= center = (i[0], i[1])
            else:
                prev_center = center
                center = (i[0], i[1])
            #print("center: ",center)           
            #print("previous center: ",prev_center)
        if prev_center is not None:
            #print("*********",prev_center)
            distance = calculate_distance(prev_center, center)        
            print("Distance = ",distance)
            if distance < distance_threshold:
                ball_detected=True
                #print("Ball detected.")
                # Publish the center coordinates as a Point32 message
                centroid_msg.x = center[0]
                centroid_msg.y = center[1]
                print("previous center: ",prev_center)
                print("center: ",center)
            if distance > distance_threshold:
                ball_detected=False
                centroid_msg.x = 0
                centroid_msg.y = 0


    centroid_pub.publish(centroid_msg) 
    msg = bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
    ball_image_pub.publish(msg)
    msg2=bridge.cv2_to_imgmsg(blur,encoding='8UC1')
    hue_image_pub.publish(msg2)
#     #print(ball_detected)
      # **This line publishes the center**


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