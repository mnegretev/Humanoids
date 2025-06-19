#!/usr/bin/env python

import rospy
from vision_msgs.srv import ProcessObject, ProcessObjectRequest
from vision_msgs.msg import VisionObject
from sensor_msgs.msg import Image
from std_msgs.msg import Header

def call_service():
    rospy.init_node('test_process_object')

    rospy.wait_for_service('/intercept_plane_service')
    try:
        process_object = rospy.ServiceProxy('/intercept_plane_service', ProcessObject)

        # Crear objeto simulado
        obj = VisionObject()
        obj.x = 320
        obj.y = 180
        obj.header.frame_id = "camera_link"

        img = Image()
        img.header = obj.header

        req = ProcessObjectRequest()
        req.object = obj
        #req.img = img

        res = process_object(req)

        rospy.loginfo("Respuesta del servicio:")
        rospy.loginfo("Posición estimada: x=%.2f, y=%.2f, z=%.2f",
                      res.object.pose.position.x,
                      res.object.pose.position.y,
                      res.object.pose.position.z)
    except rospy.ServiceException as e:
        rospy.logerr("Error al llamar al servicio: %s", str(e))

if __name__ == "__main__":
    call_service()
