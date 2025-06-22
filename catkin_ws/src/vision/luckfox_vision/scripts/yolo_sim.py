#!/usr/bin/env python3

import rospy
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from socket import socket, AF_INET, SOCK_DGRAM
from warnings import filterwarnings

filterwarnings("ignore")
UDP_PORT = 5000

def yolo_callback(msg):
    global bridge, model, class_image_pub, udp_server, udp_ip
    img = bridge.imgmsg_to_cv2(msg,"rgb8")
    det_result = model(img)
    print(det_result.pred)
    det_annotated = det_result.render()[0]
    for det in det_result.xyxy[0]:
        detarr = det.detach().cpu().numpy()
        #print(detarr)
        if len(detarr)>0:
            vis_str = "{}@({:d},{:d},{:d},{:d}){:.2f}".format(det_result.names[detarr[5]], int(detarr[0]), int(detarr[1]), int(detarr[2]), int(detarr[3]), detarr[4])
            print(vis_str)
            udp_server.sendto(vis_str.encode('utf-8'), (udp_ip, UDP_PORT))
    class_image_pub.publish(bridge.cv2_to_imgmsg(det_annotated,"rgb8"))


def main():
    global bridge, model, class_image_pub, udp_ip, udp_server
    model_path = rospy.get_param("/yolo_model_path")
    udp_server = socket(AF_INET, SOCK_DGRAM)
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
    #model = torch.hub.load('.', 'custom', path=model_path, source='local')
    rospy.init_node('Yolo_sim')
    udp_ip = rospy.get_param("~ip_server") 
    class_image_pub = rospy.Publisher("/vision/yolo/image", Image, queue_size=5)
    bridge = CvBridge()
    rospy.Subscriber("/hardware/camera/image", Image, yolo_callback)
    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
