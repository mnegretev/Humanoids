#import yolov5
import rospy
import ros_numpy
import torch
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def yolo_callback(msg):
    global bridge, model, class_image_pub
    img = bridge.imgmsg_to_cv2(msg)
    det_result = model(img)
    det_annotated = det_result[0].plot(show=False)
    class_image_pub.publish(bridge.cv2_to_imgmsg())



def main():
    global bridge, model, class_image_pub
    model_path = rospy.get_param("/yolo_model_path")
    model = YOLO(model_path)
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
