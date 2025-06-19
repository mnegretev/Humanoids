#!/usr/bin/env python3

import rospy
#import ros_numpy
import torch
#from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def yolo_callback(msg):
    global bridge, model, class_image_pub
    img = bridge.imgmsg_to_cv2(msg,"rgb8")
    det_result = model(img)
    #print(det_result)
    #print(dir(det_result))
    det_annotated = det_result.render()[0]
    #print(det_annotated)
    class_image_pub.publish(bridge.cv2_to_imgmsg(det_annotated,"rgb8"))


def main():
    global bridge, model, class_image_pub
    model_path = rospy.get_param("/yolo_model_path")
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
    #model = torch.hub.load('.', 'custom', path=model_path, source='local') 
    class_image_pub = rospy.Publisher("/vision/yolo/image", Image, queue_size=5)
    bridge = CvBridge()
    rospy.Subscriber("/hardware/camera/image", Image, yolo_callback)
    rospy.init_node('Yolo_sim')
    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
