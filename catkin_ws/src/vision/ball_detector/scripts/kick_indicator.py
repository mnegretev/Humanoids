#!/usr/bin/env python3

# == Begin: Temporal ==
import csv
import time
import matplotlib.pyplot as plt
# == End: Temporal ==
import numpy as np
import rospy
from kalman_filter import EKF
from geometry_msgs.msg import Point

# The kick_indicator node uses the measured ball's position
# along with the Kalman Filter to determine the moment when
# the robot must kick the ball.

KICK_DURATION = 0.1
ROBOT_FOOT_Y_POS = -0.1
# When reached, the time to reach the ball will
# no longer be updated
Y_THRESHOLD = -0.35

# === KALMAN FILTER PARAMS ==
DT = 0.1
Q = np.identity(4) * 0.1
R = np.identity(2) * 0.01
kf = EKF(DT, Q, R)

# == MEASUREMENTS COLLECTION ==
MAX_MEASUREMENTS = 100 # Max number of measurements to be collected
measurements = 0 # Current number of collected measurements
measurements_collected = False # Flag to indicate if the all measurements have been collected

def write_data(data):
    # This function will be used to collect data
    # for analysis purposes
    measured_x_pos = data.x # measured x-position
    measured_y_pos = data.y # measured y-position
    kf.predict()
    kf_data = kf.update(Z = [measured_x_pos, measured_y_pos])
    
    with open('/home/andreslopez/data/test1_feb6.csv', 'a', newline = '') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([time.time(), # time
                             data.x, # measured x position
                             data.y, # measured y position
                             kf_data[0][0], # estimated x position
                             kf_data[1][0], # estimated y position
                             kf_data[2][0], # estimated x velocity
                             kf_data[3][0] # estimated y velocity
                             ])

def prepare_to_kick():
    
    y_pos = kf.x_hat[1][0]
    y_vel = kf.x_hat[3][0]
    time_to_reach_foot = (abs(y_pos) - abs(Y_THRESHOLD)) / abs(y_vel)

    while y_pos < Y_THRESHOLD:
        kf.x = kf.predict()
        y_pos = kf.x_hat[1][0]
        y_vel = 0.0001 if y_vel == 0 else kf.x_hat[3][0]
        # Here, we suppose abs(y_pos) > abs(Y_THRESHOLD)
        if y_pos < Y_THRESHOLD:
            time_to_reach_foot = (abs(y_pos) - abs(Y_THRESHOLD)) / abs(y_vel)
        rospy.loginfo(f'time_to_reach_foot: {time_to_reach_foot}; y_pos: {y_pos} ; y_vel : {y_vel}')

    rate = rospy.Rate(1/DT)
    t = 0
    while t < time_to_reach_foot - KICK_DURATION:
        t += DT
        rospy.loginfo(f'time_to_efectively_kick: {time_to_reach_foot - KICK_DURATION - t}')
        rate.sleep()
    
    rospy.logwarn('== K I C K ==')

def callback_receive_measurement(data):

    global measurements
    global measurements_collected

    if not measurements_collected:
        # data.x is the measured x-position
        # data.y is the measured y-position
        kf.predict() # Prediction stage of the Kalman filter
        kf.update(Z = [data.x, data.y]) # Update stage of the Kalman filter
        measurements += 1
        measurements_collected = measurements >= MAX_MEASUREMENTS
        rospy.loginfo(f'measurements: {measurements}')

def main():
    # To know how to write publishers and subscribers
    # check: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
    rospy.init_node("kick_indicator")
    rospy.Subscriber("vision/ball_detector/ball_position", Point, callback_receive_measurement)
    # rospy.spin()
    while not rospy.is_shutdown():
        if measurements_collected: break
    prepare_to_kick()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass