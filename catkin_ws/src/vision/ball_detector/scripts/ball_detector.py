#!/usr/bin/env python3

import cv2
import numpy as np
import math
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

HSV_LOWER_LIMIT = np.array([40, 100, 50])
HSV_UPPER_LIMIT = np.array([80, 255, 255])
CAMERA_MATRIX = np.array([[531.16719459, 0,686.90394518],
                          [0, 532.5711697, 364.00099154],
                          [0, 0, 1]],
                          dtype=np.float32)
DIST = np.array([[-0.31429497,
                  0.09157624,
                  -0.00064995,
                  0.00094649,
                  -0.01083083]],
                  dtype=np.float32)

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

def get_undistorted_px_by_service(distorted_point):
    # To Do: Create service to use MATLAB's function
    # to undistort a pixel
    return distorted_point

def get_undistorted_image(distorted_image, camera_matrix, distortion_coef):
    return cv2.undistort(distorted_image, camera_matrix, distortion_coef, None)

def callback_calculate_new_camera_frame(msg):
    bridge = CvBridge()
    distorted_image = bridge.imgmsg_to_cv2(msg)
    # Get color segmentation
    binary_image = get_color_segmentation(bgr_image = distorted_image,
                                          hsv_lower_limit = HSV_LOWER_LIMIT,
                                          hsv_upper_limit = HSV_UPPER_LIMIT)
    # Get the ball distorted centroid in pixels
    distorted_centroid = get_centroid_px_from_binary_img(binary_img = binary_image)
    # Get the ball undistorted centroid in pixels
    x_undistorted, y_undistorted = get_undistorted_px_by_service(distorted_point = distorted_centroid)
    # == BEGIN: DEBUGGING SECTION ==
    undistorted_image = get_undistorted_image(distorted_image = distorted_image,
                                              camera_matrix   = CAMERA_MATRIX,
                                              distortion_coef = DIST)
    undistorted_image = cv2.circle(undistorted_image,
                                 (x_undistorted,
                                 y_undistorted),
                                 radius = 5,
                                 color = (0, 0, 255),
                                 thickness = -1)
    distorted_image = cv2.circle(distorted_image,
                                 (distorted_centroid[0],
                                 distorted_centroid[1]),
                                 radius = 5,
                                 color = (0, 0, 255),
                                 thickness = -1)

    cv2.imshow("distorted_image", distorted_image)
    cv2.imshow("binary_image", binary_image)
    cv2.imshow("undistorted_image", undistorted_image)
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