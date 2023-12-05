#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def callback_get_ball_position(msg):
    bridge = CvBridge()
    distorted_image = bridge.imgmsg_to_cv2(msg)
    cv2.imshow("distorted_image", distorted_image)
    cv2.waitKey(1)

def main():
    # Check: http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
    rospy.init_node("ball_detector")
    rospy.Subscriber("/hardware/camera/image", Image, callback_get_ball_position)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass