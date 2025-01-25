#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge   
from sensor_msgs.msg import *
from std_msgs.msg import *
import math
h_min = 0
h_max = 0
s_min = 0
s_max = 0
v_min = 0
v_max = 0
def nothing(x):
    pass

def callback_image (msg):
    global h_min, h_max, s_min, s_max, v_min, v_max
    bridge = CvBridge()     
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')#source file
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Crear los límites inferiores y superiores
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    # Crear una máscara basada en los valores de los trackbars
    mask = cv2.inRange(hsv, lower, upper)

    # Aplicar la máscara a la imagen original
    result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

      # Leer los valores de los trackbars
    h_min = cv2.getTrackbarPos('H_min', 'Control HSV')
    h_max = cv2.getTrackbarPos('H_max', 'Control HSV')
    s_min = cv2.getTrackbarPos('S_min', 'Control HSV')
    s_max = cv2.getTrackbarPos('S_max', 'Control HSV')
    v_min = cv2.getTrackbarPos('V_min', 'Control HSV')
    v_max = cv2.getTrackbarPos('V_max', 'Control HSV')

    # Show results
    cv2.imshow('Original', cv_image)
    cv2.imshow('Máscara', mask)
    cv2.imshow('Resultado', result)
    cv2.waitKey(10)


   
def main ():
    rospy.init_node("hsv_calibrator_node")  
    rospy.Subscriber("/hardware/camera/image", Image, callback_image)

    # Crear una ventana con deslizadores
    cv2.namedWindow('Control HSV')
    cv2.createTrackbar ('H_min', 'Control HSV', 0, 179, nothing)
    cv2.createTrackbar ('H_max', 'Control HSV', 179, 179, nothing)
    cv2.createTrackbar ('S_min', 'Control HSV', 0, 255, nothing)
    cv2.createTrackbar ('S_max', 'Control HSV', 255, 255, nothing)
    cv2.createTrackbar ('V_min', 'Control HSV', 0, 255, nothing)
    cv2.createTrackbar ('V_max', 'Control HSV', 255, 255, nothing)

    # Convierte el fotograma a HSV
  
    
    #ball_image_pub = rospy.Publisher("/ball_image", Image, queue_size=1)
    #hue_image_pub = rospy.Publisher("/hue_image", Image, queue_size=1)
    rospy.spin()



if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
