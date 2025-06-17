#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose, TransformStamped, PointStamped, Point, Quaternion
from tf2_geometry_msgs import do_transform_point
from tf import TransformListener
from vision_msgs.msg import VisionObject
from vision_msgs.srv import ProcessObject, ProcessObjectResponse
import tf2_ros
import tf.transformations as tft
import numpy as np

PI = 3.14159265
HFOV = 1.7157
VFOV = 1.7157
HRES = 640
VRES = 640
BALL_RADIUS = 0.06207

def broadcaster_frame_object(frame, child_frame, pose):   # Emite la transformacion en el frame base_link,
    #br = tf2_ros.TransformBroadcaster()
    br =  tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.frame_id = frame
    t.child_frame_id = child_frame 
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation.x = pose.orientation.x
    t.transform.rotation.y = pose.orientation.y
    t.transform.rotation.z = pose.orientation.z
    t.transform.rotation.w = pose.orientation.w
    br.sendTransform(t)

def get_direction_vector(cv,cu):
    theta = (VFOV/VRES)*(cv-(VRES/2))
    phi = (HFOV/HRES)*(cu-(HRES/2))

    Rot_mtx = tft.euler_matrix(0,theta,phi,'sxyz')
    #print(Rot_mtx)
    dir_vect = Rot_mtx @ np.array([1.0,0.0,0.0,0.0])

    direc = PointStamped()
    direc.header.frame_id = 'camera_optical'
    direc.point.x = dir_vect[0]
    direc.point.y = dir_vect[1]
    direc.point.z = dir_vect[2]

    return direc

def process_ball(msg):
    global tf_listener, tf_buf, scaling_constant, cam_range
    u = msg.object.x
    v = msg.object.y
    w = msg.object.width
    h = msg.object.height
    cu = (u+w)/2
    cv = (v+h)/2

    #d = scaling_constant * (w+h)/2

    oc = PointStamped()
    oc.header.frame_id = 'camera_optical'
    oc.point = Point(x=0,y=0,z=0)

    cam_pos = tf_buf.transform(oc,'left_foot_link').point
    ball_dir = tf_buf.transform(get_direction_vector(cv,cu),'left_foot_link').point

    #print(cam_pos)
    cam_pos = np.array([cam_pos.x,cam_pos.y,cam_pos.z])
    ball_dir = np.array([ball_dir.x,ball_dir.y,ball_dir.z])

    line_vect = (cam_pos-ball_dir)/np.linalg.norm(cam_pos-ball_dir)

    L = (BALL_RADIUS-cam_pos[2])/line_vect[2]
    xb = cam_pos[0] + L*line_vect[0]
    yb = cam_pos[1] + L*line_vect[1]
    zb = BALL_RADIUS

    obj_pose = Pose()
    obj_pose.position.x = xb
    obj_pose.position.y = yb
    obj_pose.position.z = zb
    obj_pose.orientation = Quaternion(x=0,y=0,z=0,w=1)

    print(obj_pose)

    broadcaster_frame_object('left_foot_link','ball_frame',obj_pose)

    resp = ProcessObjectResponse()
    
    resp.object.header = msg.object.header
    resp.object.id = msg.object.id
    resp.object.confidence = msg.object.confidence
    resp.object.pose = obj_pose
    resp.object.x = msg.object.x
    resp.object.y = msg.object.y
    
    return resp

def calculate_distance(width, height):
    print(width, height)
    d1 = exp_w/(width*exp_d)
    d2 = exp_h/(height*exp_d)
    return (d1+d2)/2
    
def calculate_angle(x,y):
    #320 camera center
    print(x,y)
    try:
        x1 = x-320
        theta = (x1*exp_a)/640
        y1 = y-320
        phi = (y1*exp_b)/640
        return theta, phi
    except Exception as e:
        print(f"No se pudo calcular el angulo {e}")

def process_goal(req):
    print(f"Client send: {req.object.id}")
    d=calculate_distance(req.object.width, req.object.height)
    (theta,phi) = calculate_angle(req.object.x+(req.object.width/2), req.object.y+(req.object.height/2))
    x = d*np.sin(phi) * np.cos(theta)
    y = d*np.sin(theta) * np.sin(phi)
    z = d* np.cos(phi)
    camera_pos = PointStamped()
    camera_pos.header.frame_id = "/camera_optical"
    camera_pos.header.stamp = rospy.Time(0)
    camera_pos.point.x,camera_pos.point.y,camera_pos.point.z = x,y,z 
    camera_pos = listener.transformPoint("/left_foot_link", camera_pos)
    res = ProcessObjectResponse()
    res.object.id = req.object.id
    res.object.confidence = req.object.confidence
    res.object.x = req.object.x
    res.object.y = req.object.y
    res.object.width = req.object.width
    res.object.height = req.object.height
    res.object.pose.position = camera_pos.point
    return res



def callback_process_object(msg):
    if msg.object.header == "ball":
        return process_ball(msg)
    elif msg.object.header == "goal":
        return process_goal(msg)
    else:
        return ProcessObjectResponse(VisionObject())


def main():
    global tf_listener, tf_buf, scaling_constant, cam_range, listener, exp_w, exp_h, exp_d, exp_a, exp_b
    rospy.sleep(0.01)
    print('Starting Process Object Service')
    cam_range = rospy.get_param('/cam_angle_range',PI)
    exp_w = rospy.get_param("~width", 400)
    exp_h = rospy.get_param("~height", 400)
    exp_d = rospy.get_param("~distance", 1.0)
    exp_a = rospy.get_param("~alpha", (np.pi)/4)
    exp_b = rospy.get_param("~beta",(3*np.pi)/4)
    scaling_constant = rospy.get_param('/cam_scaling_constant',1)
    rospy.init_node('process_obj_srv')

    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    listener = TransformListener()
    listener.waitForTransform("/left_foot_link", "/camera_optical", rospy.Time(), rospy.Duration(4.0))
    
    rospy.Service('/intercept_plane_service' ,ProcessObject ,callback_process_object)
    loop = rospy.Rate(1)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass