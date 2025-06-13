#!/usr/bin/env python

import rospy
import math
#import tf2_ros
import tf

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from vision_msgs.srv import ProcessObject, ProcessObjectResponse
from vision_msgs.msg import VisionObject

image_width = 1280
image_height = 720
center_x = image_width / 2
center_y = image_height / 2
HFOV = math.radians(90.0)
VFOV = math.radians(60.0)

def callback_calculate_position(req):
    #global tf_buffer

    x_B = req.object.x
    y_B = req.object.y


    theta = (VFOV / image_height) * (center_y - y_B)
    phi = (HFOV / image_width) * (center_x - x_B)

    ux = math.cos(theta) * math.cos(phi)
    uy = math.cos(theta) * math.sin(phi)
    uz = math.sin(theta)
    print("wrt_camera", ux, uy, uz)

    # try:
    #     trans = tf_buffer.lookup_transform('left_foot_plane_link', 'camera_optical', rospy.Time(0), rospy.Duration(1.0))
    # except Exception as e:
    #     rospy.logwarn(f"Error al obtener la transformación tf: {e}")
    #     return ProcessObjectResponse()

    p = PointStamped()
    p.header.frame_id = 'camera_optical'
    p.header.stamp = rospy.Time(0)
    p.point.x, p.point.y, p.point.z = ux, uy, uz
    listener.waitForTransform('left_foot_plane_link','camera_optical', rospy.Time(), rospy.Duration(10.0))
    p = listener.transformPoint('left_foot_plane_link', p)
    #p = tf_buffer.transform(p, 'left_foot_plane_link')
    print("wrt_foot", p)
    camera = PointStamped()
    camera.header.frame_id = 'camera_optical'
    camera.header.stamp = rospy.Time(0)
    camera = listener.transformPoint('left_foot_plane_link', camera)

    print("camera", camera)

    lx, ly, lz = p.point.x - camera.point.x, p.point.y - camera.point.y, p.point.z - camera.point.z
    mag = math.sqrt(lx * lx + ly * ly + lz * lz)
    lx, ly, lz = lx / mag, ly / mag, lz / mag
    print("l", lx, ly, lz)
    d = (-camera.point.z / (lz))
    print("d", d)
    px, py, pz = camera.point.x + d * lx, camera.point.y + d * ly, camera.point.z + d * lz
    ball = PointStamped()
    ball.header.frame_id = 'left_foot_plane_link'
    ball.header.stamp = rospy.Time(0)
    ball.point.x, ball.point.y, ball.point.z = px, py, pz 
    print(px, py, pz)
    #r = trans.transform.translation
    #print(r)
    #r_x, r_y, r_z = r.x, r.y, r.z


    #tan_theta = math.tan(theta)
    #est_x = r_x - r_z * tan_theta * math.cos(phi)
    #est_y = r_y - r_z * tan_theta * math.sin(phi)


    res = ProcessObjectResponse()
    res.object = req.object
    res.object.pose.position.x = px
    res.object.pose.position.y = py
    res.object.pose.position.z = pz
    res.img = req.img

    pub.publish(ball)

    return res

def ball_pose_estimator():
    #global tf_buffer
    global listener
    global pub
    rospy.init_node('ball_pose_estimator')

    #tf_buffer = tf2_ros.Buffer()
    listener = tf.TransformListener()

    s = rospy.Service('process_object', ProcessObject, callback_calculate_position)
    rospy.loginfo("Servicio 'process_object' listo.")
    pub = rospy.Publisher("ball_position", PointStamped, queue_size= 1)
    rospy.spin()

if __name__ == "__main__":

    ball_pose_estimator()