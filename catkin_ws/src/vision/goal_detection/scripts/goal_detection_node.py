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

def callback_image (msg):
    # Print ("imagen recibida")
    bridge = CvBridge() 
    # Read de original image 
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')#source file
    # Convert to hls space color: brightness > color intensity
    hls_image= cv2.cvtColor(cv_image, cv2.COLOR_BGR2HLS)
    Lchannel=hls_image[:,:,2]#To access the entire lightness channel
    mask = cv2.inRange(Lchannel, 0,0)#Create the mask
    image_mask=cv2.bitwise_and(cv_image,cv_image,mask=mask)#Apply mask to original image
    #0 ----> Hue 
    #2 ----> Saturation
    #Bitwise_and-------> intersection
    #Bitwise_or--------> non intersection and intersection
    #Bitwise_xor-------> non intersection regions
    #--------------------------FIND EDGES*------------------------------------#
    # Detect lines using Probabilistic Hough Line Transform
    # Blur the image for better edge detection
    blur=cv2.GaussianBlur(image_mask,(7,7),0)
    # Canny Edge Detection
    edges = cv2.Canny(blur,120, 150) # Canny Edge Detection
    cdstP = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR) 
    lines = cv2.HoughLinesP(edges, 1, numpy.pi/180, threshold=50, minLineLength=10, maxLineGap=60)
    #for the method above https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
    # Draw lines
    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]#retrieves cordinates
            cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv2.LINE_AA)#draws a line onto the image, connecting the endpoints

    #-------------------------FIND CONTOURS**------------------------------#
    #apply binary thresholding
    gray = cv2.cvtColor(cdstP, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 50, 150, cv2.THRESH_BINARY)#0 is black 255 is white
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #RETR_TREE ----> retrieves all contours with hierarchy 
    #RETR_LIST ----> retrieves all contours without hierarchy
    #RETR_EXTERNAL --->retrieves only external contours
    #CHAIN_APPROX_NONE---> stores all the contour points
    #CHAIN_APPROX_SIMPLE---> compresses h,v,d segments only their points
    #contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.INTERSECT_FULL, method=cv2.CHAIN_APPROX_NONE)
    #-----------draw contours on the original image--------------
    contour_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(image=contour_image, contours=contours, contourIdx=-1, color=(0, 0, 250), thickness=2, lineType=cv2.LINE_AA)
    #print("Number of contours found:", len(contours)) 
    target_contour=None

    for contour in contours:
        M=cv2.moments(contour)
        hu_moments = cv2.HuMoments(M)        
        if 0.21<hu_moments[0]<0.24 and 0.01<hu_moments[1]<0.018 and hu_moments[5]>0 and hu_moments[6]>0.000001:
            i+=1
            real_contours=[i]
            target_contour=contour           
            #shorter x longer y /// longer x y shorter y
            break
    global cx,cy
    if target_contour is not None:
        #M_t = cv2.moments(target_contour)
        #hu_moments = cv2.HuMoments(M_t) 
        cx=int(M["m10"]/M["m00"])
        cy=int(M["m01"]/M["m00"])        
        print("Centroid:", (cx,cy)) 
        
        print("Target contour:", len(target_contour)) 
        print("Number of contours found:", len(real_contours))
        print("Hu moments:",hu_moments)
        cv2.drawContours(image=contour_image, contours=[target_contour], contourIdx=-1, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_AA)
        error_x = cx-320
        error_y = cy - 240
        goal_pan = -1/10000.0 * error_x
        goal_tilt = 1/7000.0 * error_y

        if  goal_pan > 1.5:
             goal_pan = 1.5

        if goal_pan < -1.5:
           goal_pan = -1.5

        if goal_tilt > 1:
           goal_tilt = 1

        if goal_tilt < -1:
           goal_tilt = -1
        contour_image=cv2.circle(contour_image, (cx,cy), radius=0, color=(0, 255, 0), thickness=-1)
    # head_goal_pose = Float32MultiArray()
    # head_goal_pose.data = [goal_pan, goal_tilt]
    # print(head_goal_pose.data)
    # pub_head_goal.publish(head_goal_pose)

        centroid_pub_msg = Point32()
        centroid_pub_msg.x=cx
        centroid_pub_msg.y=cy
        centroid_pub.publish(centroid_pub_msg)
        # Convert Hu moments to a list for easier comparison

    #print(f"Area of contour: {area,hu_moments}")
    # Detect lines using Hough Line Transform
    #---------------------------Head movement----------------------------------#
    #pub_contours.publish(bridge.cv2_to_imgmsg(contour_image, encoding = "rgb8"))
    # see the results
    #cv2.imshow('None approximation', image_copy)
    #cv2.waitKey(10)
    #cv2.imwrite('contours_none_image1.jpg', image_copy)
    # Display images
    #Transform img to message with cv_bridge

    #cv2.imshow("imagen",cv_image) #source file
    msg = bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
    original_cv_image_pub.publish(msg)

    cv2.imshow('HLS image', hls_image) #hls image
    cv2.imshow("blur image",blur) #Blur image
    cv2.imshow("image mask",image_mask) #Masked image
    cv2.imshow('Canny Edge Detection', edges)#*Canny edge detection
    cv2.imshow("Contours", contour_image)#contours found
    cv2.imshow("Detected Lines-Probabilistic Line Transform", cdstP)
    cv2.waitKey(10)


def main ():
    global pub_head_goal,centroid_pub, original_cv_image_pub
    pub_head_goal = rospy.Publisher("/hardware/head_goal_pose", Float32MultiArray, queue_size=1)
    centroid_pub = rospy.Publisher("/centroid_publisher", Point32, queue_size=1)
    original_cv_image_pub = rospy.Publisher("/original_image", Image, queue_size=1)
    rospy.init_node("goal_detection_node")  
    rospy.Subscriber("/hardware/camera/image", Image, callback_image)
    rospy.spin()

if __name__=="__main__":
    main()
