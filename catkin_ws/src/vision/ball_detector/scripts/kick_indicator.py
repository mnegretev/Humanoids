#!/usr/bin/env python3

# == Begin: Temporal ==
import csv
import time
# == End: Temporal ==
import numpy as np
import rospy
from kalman_filter import EKF
from geometry_msgs.msg import Point

# The kick_indicator node uses the measured ball's position
# along with the Kalman Filter to determine the moment when
# the robot must kick the ball.

# ROBOT_FOOT_Y_POS = -0.1
# Y_AXIS_THRESHOLD = 0.1
# KICK_DURATION = 0.1
# DT = 0.1

# ball_reached_threshold = False
# time_to_reach_foot = 0
# last_y_pos = 0

# def kick_ball():
#     rate = rospy.Rate(1/DT)
#     t = 0
#     while t < time_to_reach_foot - KICK_DURATION:
#         t += DT
    
#     rospy.logwarn('== KICK ==')

def callback_receive_measurement(data):

    with open('/home/andreslopez/data/test9.csv', 'a', newline = '') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([time.time(), data.x, data.y])

    # global ball_reached_threshold
    # global time_to_reach_foot
    # global last_y_pos

    # if not ball_reached_threshold:

    #     measured_x_pos = data.x # measured x-position
    #     measured_y_pos = data.y # measured y-position

        # Q = np.identity(4) * 0.1
        # R = np.identity(2) * 0.01
        # kf = EKF(0.1, Q, R)
        # kf_data = kf.estimate(Z = [measured_x_pos, measured_y_pos])
        # kf_pos_x = kf_data[0][0]
        # kf_pos_y = kf_data[1][0]
        # kf_vel_x = kf_data[2][0]
        # kf_vel_y = kf_data[3][0]
        # time_to_reach_foot = (abs(kf_pos_y) + abs(ROBOT_FOOT_Y_POS)) / kf_vel_y
        # vel_y = (measured_y_pos - last_y_pos) / DT
        # time_to_reach_foot = (abs(measured_y_pos) + abs(ROBOT_FOOT_Y_POS)) / vel_y if vel_y != 0 else 0
        # last_y_pos = measured_y_pos

        # ball_reached_threshold = measured_y_pos <= Y_AXIS_THRESHOLD

        # rospy.loginfo(f'Measured x pos: {measured_x_pos} | Measured y pos: {measured_y_pos} | ' + 
        # f'kf_pos_x: {kf_pos_x} | kf_vel_x: {kf_vel_x} | ' +
        # f'kf_pos_y: {kf_pos_y} | kf_vel_y: {kf_vel_y}')
        # rospy.loginfo(f'kf_pos_y: {kf_pos_y} | kf_vel_y : {kf_vel_y} | time_to_reach_foot : {time_to_reach_foot}')
        # rospy.loginfo(f'pos_y: {measured_y_pos} | vel_y : {round(vel_y,2)} | time_to_reach_foot : {round(time_to_reach_foot,2)}')
    

def main():
    # To know how to write publishers and subscribers
    # check: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
    rospy.init_node("kick_indicator")
    rospy.Subscriber("vision/ball_detector/ball_position", Point, callback_receive_measurement)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     if ball_reached_threshold:
    #         break

    # kick_ball()
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass