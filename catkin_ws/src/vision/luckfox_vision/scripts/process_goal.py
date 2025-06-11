#!/usr/bin/env python

import rospy
import tf
import numpy

from vision_msgs import ProcessObject, ProcessObjectResponse
from geometry_msgs.msg import PointStamped

def calculate_distance(width, height):
    d1 = exp_w/(width*exp_d)
    d2 = exp_h/(height*exp_d)
    return (d1+d2)/2
    
def calculate_angle(x,y):
    #320 camera center
    if x ==320:
        theta = 0
    else:
        x1 = x-320
        theta = (640*exp_a)/x1
    if y ==320:
        phi = 0
    else:
        y1 = y-320
        phi = (640*exp_b)/y1
    return(theta, phi)

def handle_goal_keep(req):
    print(f"Client send: {req.object.id}")
    d=calculate_distance(req.object.width, req.object.height)
    (theta,phi) = calculate_angle(req.object.x+(req.object.width/2), req.object.y+(req.object.height/2))
    x = d*numpy.sin(phi) * numpy.cos(theta)
    y = d*numpy.sin(theta) * numpy.sin(phi)
    z = d* numpy.cos(phi)
    camera_pos = PointStamped()
    camera_pos.header.frame_id = "/camera_optical"
    camera_pos.header.stamp = rospy.Time(0)
    camera_pos.point.x,camera_pos.point.y,camera_pos.point.z = x,y,z 
    camera_pos = listener.transformPoint("/right_foot_plane_link", camera_pos)
    res = ProcessObjectResponse()
    res.object = req.object
    res.pose.position = camera_pos
    return res



def main():
    global listener, exp_w, exp_h, exp_d, exp_a, exp_b
    rospy.init_node("goal_keep_service")
    exp_w = rospy.get_param("~width")
    exp_h = rospy.get_param("~height")
    exp_d = rospy.get_param("~distance")
    exp_a = rospy.get_param("~alpha")
    exp_b = rospy.get_param("~beta")
    listener = tf.TransformListener()
    listener.waitForTransform("/right_foot_plane_link", "/camera_optical", rospy.Time(), rospy.Duration(4.0))
    s = rospy.Service("ProcessGoal", ProcessObject, handle_goal_keep)
    print("Process Goal Service UP")
    rospy.spin()

if __name__ == "__main__":
    main()