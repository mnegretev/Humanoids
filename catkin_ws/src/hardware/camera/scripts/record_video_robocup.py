#!/usr/bin/python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RecordNode:
    def __init__(self, topic, topic2, topic3):
        if rospy.get_param("~record_video"):
            self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.write  = cv2.VideoWriter('/home/humanoid/video.mp4',self.fourcc, 30.0, (640*3,480))
        self.image_pub = rospy.Publisher("/vision/triple_camera", Image, queue_size=1)
        self.topic = topic
        self.topic2 = topic2
        self.topic3 = topic3
        self.bridge = CvBridge()

    def saveFrame(self):
        img     = rospy.wait_for_message(self.topic, Image, timeout=10)
        cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        img2    = rospy.wait_for_message(self.topic2, Image, timeout=10)
        cv_image2 = self.bridge.imgmsg_to_cv2(img2, desired_encoding='8UC1')
        cv_image2 = cv2.cvtColor(cv_image2, cv2.COLOR_GRAY2RGB)
        img3    = rospy.wait_for_message(self.topic3, Image, timeout=10)
        cv_image3 = self.bridge.imgmsg_to_cv2(img3, desired_encoding='rgb8')
        im_h = cv2.hconcat([cv_image, cv_image3])
        im_h = cv2.hconcat([im_h, cv_image2])
        if rospy.get_param("~record_video", default=True):
            self.write.write(im_h)
        msg = self.bridge.cv2_to_imgmsg(im_h, encoding='rgb8')
        self.image_pub.publish(msg)


    def closeVideo(self):
        if rospy.get_param("~record_video"):
            self.write.release()

def main():
    rospy.init_node("video_writer")  
    record_node = RecordNode("/hardware/camera/image", "/hue_image", "/ball_image")
    while not rospy.is_shutdown():               
        record_node.saveFrame()
    record_node.closeVideo()
  
  
if __name__ == "__main__": 
    try:
        main()
    except rospy.ROSInterruptException:
        pass
