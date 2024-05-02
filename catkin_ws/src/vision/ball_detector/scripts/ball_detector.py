#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from object_identification.hsv_color_segmentation import HsvColorSegmentation

HSV_LOWER_LIMIT = np.array([40, 100, 50])
HSV_UPPER_LIMIT = np.array([80, 255, 255])

def callback_get_obj_centroid(msg):
    bgr_img = CvBridge().imgmsg_to_cv2(msg)
    seg = HsvColorSegmentation(HSV_LOWER_LIMIT, HSV_UPPER_LIMIT)
    centroid = seg.get_obj_centroid(bgr_img)
    centroid_point = Point()
    centroid_point.x = centroid[0]
    centroid_point.y = centroid[1]
    centroid_pub.publish(centroid_point)

def main():
    global centroid_pub # centroid publisher
    rospy.init_node("ball_detector")
    rospy.Subscriber("/hardware/camera/image", Image, callback_get_obj_centroid, queue_size=1)
    centroid_pub = rospy.Publisher("/vision/dist_centoid_px", Point, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
