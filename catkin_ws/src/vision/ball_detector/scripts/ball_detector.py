#!/usr/bin/env python3

import cv2
import numpy as np
import math
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

HSV_LOWER_LIMIT = np.array([40, 100, 50])
HSV_UPPER_LIMIT = np.array([80, 255, 255])

def get_color_segmentation(bgr_image, hsv_lower_limit, hsv_upper_limit):
    img_hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    img_bin = cv2.inRange(img_hsv, hsv_lower_limit, hsv_upper_limit)
    # Erode and then dilate
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    img_without_noise = cv2.morphologyEx(img_bin, cv2.MORPH_OPEN, kernel)
    return img_without_noise

def get_centroid_px_from_binary_img(binary_img):
    nonzero_elements = cv2.findNonZero(binary_img)
    centroid = cv2.mean(nonzero_elements)
    return (int(centroid[0]),int(centroid[1]))

def callback_calculate_new_camera_frame(msg):
    bridge = CvBridge()
    distorted_image = bridge.imgmsg_to_cv2(msg)
    # Get color segmentation
    binary_image = get_color_segmentation(bgr_image = distorted_image,
                                          hsv_lower_limit = HSV_LOWER_LIMIT,
                                          hsv_upper_limit = HSV_UPPER_LIMIT)
    # Get the ball distorted centroid in pixels
    distorted_centroid = get_centroid_px_from_binary_img(binary_img = binary_image)
    
    # == BEGIN: DEBUGGING SECTION ==
    distorted_image = cv2.circle(distorted_image,
                                 (distorted_centroid[0],
                                 distorted_centroid[1]),
                                 radius = 5,
                                 color = (0, 0, 255),
                                 thickness = -1)
    cv2.imshow("distorted_image", distorted_image)
    cv2.imshow("binary_image", binary_image)
    cv2.waitKey(1)
    # == END: DEBUGGING SECTION ==

def main():
    # Check: http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
    rospy.init_node("ball_detector")
    rospy.Subscriber("/hardware/camera/image", Image, callback_calculate_new_camera_frame)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass