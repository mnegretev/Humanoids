#!/usr/bin/env python3

# The node NODE_NAME continously publishes images
# that come from CAMERA to the topic TOPIC_NAME 

# Sources:
# https://www.youtube.com/watch?v=2l913YwWYe4

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os 

PUBL_NAME  = "camera_video_publisher"
TOPIC_NAME = "/hardware/camera/image"
CAMERA     = 0
RATE       = 30.0

def configure_camera():
    # Configuración de la cámara usando v4l2-ctl
    os.system("v4l2-ctl -d /dev/video{} -c focus_auto=0".format(CAMERA))
    os.system("v4l2-ctl -d /dev/video{} -c white_balance_temperature_auto=0".format(CAMERA))
    os.system("v4l2-ctl -d /dev/video{} -c exposure_auto=1".format(CAMERA))  
    os.system("v4l2-ctl -d /dev/video{} -c exposure_absolute=50".format(CAMERA)) 
    os.system("v4l2-ctl -d /dev/video{} -c white_balance_temperature=45000".format(CAMERA))


def main():
    # Check: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
    pub_img = rospy.Publisher(TOPIC_NAME, Image, queue_size = 10)
    # Check: http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
    rospy.init_node(PUBL_NAME)

    configure_camera()

    video_capture = cv2.VideoCapture(CAMERA)
    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    video_capture.set(cv2.CAP_PROP_FPS, 30)

    bridge = CvBridge()
    rate = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        ret, frame = video_capture.read()
        if not ret:
            rospy.logerr("Couldn't get image from camera")
            break
        # Check: https://github.com/whats-in-a-name/CarND-Capstone/commit/de9ad68f4e5f1f983dd79254a71a51894946ac11
        pub_img.publish(bridge.cv2_to_imgmsg(frame, encoding = "rgb8"))
        # rate.sleep()        

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
