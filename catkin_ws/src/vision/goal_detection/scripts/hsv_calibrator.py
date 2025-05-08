#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def nothing(x):
    pass

class HSVCalibrator:
    def __init__(self):
        rospy.init_node('hsv_calibrator', anonymous=True)

        # Configurar interfaz
        cv2.namedWindow('HSV Calibrator')
        cv2.createTrackbar('H Min', 'HSV Calibrator', 0, 179, nothing)
        cv2.createTrackbar('H Max', 'HSV Calibrator', 179, 179, nothing)
        cv2.createTrackbar('S Min', 'HSV Calibrator', 0, 255, nothing)
        cv2.createTrackbar('S Max', 'HSV Calibrator', 255, 255, nothing)
        cv2.createTrackbar('V Min', 'HSV Calibrator', 0, 255, nothing)
        cv2.createTrackbar('V Max', 'HSV Calibrator', 255, 255, nothing)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/hardware/camera/image', Image, self.image_callback)
        self.frame = None

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error en cv_bridge: {e}")

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.frame is None:
                rate.sleep()
                continue

            frame = self.frame.copy()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            h_min = cv2.getTrackbarPos('H Min', 'HSV Calibrator')
            h_max = cv2.getTrackbarPos('H Max', 'HSV Calibrator')
            s_min = cv2.getTrackbarPos('S Min', 'HSV Calibrator')
            s_max = cv2.getTrackbarPos('S Max', 'HSV Calibrator')
            v_min = cv2.getTrackbarPos('V Min', 'HSV Calibrator')
            v_max = cv2.getTrackbarPos('V Max', 'HSV Calibrator')

            lower_bound = np.array([h_min, s_min, v_min])
            upper_bound = np.array([h_max, s_max, v_max])

            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            cv2.imshow('Original', frame)
            cv2.imshow('Mask', mask)
            cv2.imshow('Filtered', result)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    calibrator = HSVCalibrator()
    calibrator.run()