#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from kalman_filter import EKF

# === Begin: To be removed later ===
import time
import csv

def write_pose_and_velocity(x_pos, y_pos, # x and y positions. They are NOT calculated with KF
                            kf_x_pos, kf_y_pos, # x and y positions. They are calculated with KF
                            kf_x_vel, kf_y_vel, # x and y velocities. They are calculated with KF
                            file_path):

    with open(file_path, 'a', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([time.time(),
                             x_pos,
                             y_pos,
                             kf_x_pos,
                             kf_y_pos,
                             kf_x_vel,
                             kf_y_vel])
    print(type(kf_x_pos))

# === End: To be removed lated ===

def main():

    # Check:
    # http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
    # https://answers.ros.org/question/335584/how-to-transpose-and-rotate-frames-and-get-new-coordinates/
    # http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html

    rospy.init_node("ball_position_estimator")
    print("Node ball_position_estimator is working")
    pub_pos = rospy.Publisher("vision/ball_detector/ball_position", Point, queue_size = 10)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(30.0)

    while not rospy.is_shutdown():

        try:
            # Transform from left_foot_plane_link to new_camera_optical
            t_foot_to_new_camera = tfBuffer.lookup_transform('left_foot_plane_link',
                                                             'new_camera_optical',
                                                             rospy.Time())
        except Exception as e:
            continue
        
        try:
            # Transform from left_foot_plane_link to camera_optical
            t_foot_to_camera = tfBuffer.lookup_transform('left_foot_plane_link',
                                                         'camera_optical',
                                                         rospy.Time())
        except Exception as e:
            continue

        # Point with respect to the frame new_camera
        p_new_cam = PoseStamped()
        p_new_cam.header.frame_id = "new_camera_optical"
        p_new_cam.pose.position.x = 0.5
        p_new_cam.pose.position.y = 0
        p_new_cam.pose.position.z = 0
        # Calculate p_new_cam w.r.t the frame left_foot_plane_link
        p_l_foot = tf2_geometry_msgs.do_transform_pose(p_new_cam, t_foot_to_new_camera)
        Qx = float(p_l_foot.pose.position.x)
        Qy = float(p_l_foot.pose.position.y)
        Qz = float(p_l_foot.pose.position.z)
        # Get the origin of camera_optical (the same as new_camera_optical) w.r.t left_foot_plane_link
        Px =  float(t_foot_to_camera.transform.translation.x)
        Py =  float(t_foot_to_camera.transform.translation.y)
        Pz =  float(t_foot_to_camera.transform.translation.z)
        # Get the vector that points to the center of the ball
        PQ = [Px - Qx, Py - Qy, Pz - Qz]
        # ball radius
        r = 0.08 / 2
        k = (r - Pz) / PQ[2]
        ball_x = Px + k * PQ[0]
        ball_y = Py + k * PQ[1]
        ball_z = r
        ball_x = round(ball_x, 2)
        ball_y = round(ball_y, 2)
        ball_z = round(ball_z, 2)

        # Applying Kalman Filter and considering constant velocity
        Q = np.identity(4) * 0.1
        R = np.identity(2) * 0.01
        kf = EKF(0.1, Q, R)
        kf_pos_x, kf_pos_y, kf_vel_x, kf_vel_y = kf.estimate(Z = [ball_x, ball_y])

        kf_pos_x = kf_pos_x[0]
        kf_pos_y = kf_pos_y[0]
        kf_vel_x = kf_vel_x[0]
        kf_vel_y = kf_vel_y[0]

        # Publish ball position
        ball_position = Point()
        ball_position.x = ball_x
        ball_position.y = ball_y
        ball_position.z = ball_z

        pub_pos.publish(ball_position)
        
        write_pose_and_velocity(x_pos = ball_x,
                                y_pos = ball_y,
                                kf_x_pos = kf_pos_x,
                                kf_y_pos = kf_pos_y,
                                kf_x_vel = kf_vel_x,
                                kf_y_vel = kf_vel_y,
                                file_path = '/home/andreslopez/data/not_from_raspberry_test_3.csv')

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
