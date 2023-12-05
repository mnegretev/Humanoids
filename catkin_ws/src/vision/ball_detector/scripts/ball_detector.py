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

def get_roll_pitch_yaw(centroid_x,
                       centroid_y,
                       img_x_center,
                       img_y_center,
                       width_resolution,
                       height_resolution,
                       hfov_rad,
                       vfov_rad):
  # https://themetalmuncher.github.io/fov-calc/
  # hfov = 150 degrees, vfov = 130 degrees
  k_hfov = hfov_rad / width_resolution
  k_vfov = vfov_rad / height_resolution
  return [0, # roll
          (centroid_y - img_y_center) * k_vfov, # pitch
          (img_x_center - centroid_x) * k_hfov] # yaw

def get_quaternion_from_euler(roll, pitch, yaw):
  # https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

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
    # Get the angles roll, pitch and yaw that indicate the orientation of the new frame
    roll, pitch, yaw = get_roll_pitch_yaw(centroid_x = x_undistorted,
                                          centroid_y = y_undistorted,
                                          img_x_center = 640,
                                          img_y_center = 360,
                                          width_resolution = 1550,
                                          height_resolution = 1290,
                                          hfov_rad = 2.6180,
                                          vfov_rad = 2.2689)
    # Get the anglesr roll, pitch and yaw as a quaternion q
    q = get_quaternion_from_euler(roll = roll, pitch = pitch, yaw = yaw)

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