#!/usr/bin/env python3

# == Begin: Temporal ==
import csv
import time
import matplotlib.pyplot as plt
from pprint import pprint
# == End: Temporal ==
import numpy as np
import rospy
from kalman_filter import EKF
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

# === PLOTTING PARAMS ===
time0 = None

# === Kick params ===

# The kick_indicator node uses the measured ball's position
# along with the Kalman Filter to determine the moment when
# the robot must kick the ball.
has_kicked = False
kick_time = None
KICK_DURATION = 0.1
ROBOT_FOOT_Y_POS = -0.2
# When reached, the time to reach the ball will
# no longer be updated
Y_THRESHOLD = -0.4

# === KALMAN FILTER PARAMS ==
DT = 0.1
Q = np.identity(4) * 0.1
R = np.identity(2) * 0.01
kf = EKF(DT, Q, R)

# == MEASUREMENTS COLLECTION ==
MAX_MEASUREMENTS = 80 # Max number of measurements to be collected
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

def sent_data_to_plotter(
    sampled_x_pos,
    sampled_y_pos,
    measured_x_pos,
    measured_y_pos,
    estimated_x_pos,
    estimated_y_pos,
    estimated_x_vel,
    estimated_y_vel,
    foot_pos = ROBOT_FOOT_Y_POS,
    threshold = Y_THRESHOLD
):
    global time0
    global kick_time
    global has_kicked
    
    estimations_list = []
    estimations_array = Float32MultiArray()

    if not time0:
        time0 = time.time()

    if has_kicked:
        kick_time = time.time() - time0
        has_kicked = False

    estimations_list.append(round(time.time() - time0,4)) # 0
    estimations_list.append(round(foot_pos, 4)) # 1
    estimations_list.append(round(threshold, 4)) # 2
    estimations_list.append(round(sampled_x_pos, 4)) # 3
    estimations_list.append(round(sampled_y_pos, 4)) # 4
    estimations_list.append(round(measured_x_pos, 4)) # 5
    estimations_list.append(round(measured_y_pos, 4)) # 6
    estimations_list.append(round(estimated_x_pos, 4)) # 7
    estimations_list.append(round(estimated_y_pos, 4)) # 8
    estimations_list.append(round(estimated_x_vel, 4)) # 9
    estimations_list.append(round(estimated_y_vel, 4)) # 10
    estimations_list.append(round(kick_time if kick_time else 1000, 4)) # 11

    estimations_array.data = estimations_list
    data_pub.publish(estimations_array)

def prepare_to_kick():
    
    global has_kicked

    rate = rospy.Rate(1/DT)

    y_pos = kf.x_hat[1][0]
    y_vel = kf.x_hat[3][0]
    time_to_reach_foot = (abs(y_pos) - abs(Y_THRESHOLD)) / abs(y_vel)

    while y_pos < Y_THRESHOLD and not rospy.is_shutdown():
        # kf.x = kf.predict()
        y_pos = kf.x_hat[1][0]
        y_vel = 0.0001 if y_vel == 0 else kf.x_hat[3][0]
        # Here, we suppose abs(y_pos) > abs(Y_THRESHOLD)
        if y_pos < Y_THRESHOLD:
            time_to_reach_foot = (abs(y_pos) - abs(ROBOT_FOOT_Y_POS)) / abs(y_vel)
        rate.sleep()

    print(f"{y_pos}, {time_to_reach_foot}\n")

    t = 0
    if t < time_to_reach_foot - KICK_DURATION and not rospy.is_shutdown():
        print(f'time_to_efectively_kick: {time_to_reach_foot - KICK_DURATION - t}')
        while t < time_to_reach_foot - KICK_DURATION:
            t += DT
            rate.sleep()
        has_kicked = True
    else:
        print("COULDN'T KICK")
    
    while not rospy.is_shutdown():
        rate.sleep()

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

        sent_data_to_plotter(
            sampled_x_pos = data.x,
            sampled_y_pos = data.y,
            measured_x_pos = data.x,
            measured_y_pos = data.y,
            estimated_x_pos = kf.x[0][0],
            estimated_y_pos = kf.x[1][0],
            estimated_x_vel = kf.x[2][0],
            estimated_y_vel = kf.x[3][0]
        )
    else:

        kf.x = kf.predict()

        sent_data_to_plotter(
            sampled_x_pos = 1000,
            sampled_y_pos = 1000,
            measured_x_pos = data.x,
            measured_y_pos = data.y,
            estimated_x_pos = kf.x_hat[0][0],
            estimated_y_pos = kf.x_hat[1][0],
            estimated_x_vel = kf.x_hat[2][0],
            estimated_y_vel = kf.x_hat[3][0]
        )


def main():
    global data_pub # data publisher
    # To know how to write publishers and subscribers
    # check: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
    rospy.init_node("kick_indicator")
    rospy.Subscriber("vision/ball_detector/ball_position", Point, callback_receive_measurement)
    data_pub = rospy.Publisher("vision/ball_detector/estimations", Float32MultiArray, queue_size = 10)
    # rospy.spin()
    while not rospy.is_shutdown():
        if measurements_collected: break
    prepare_to_kick()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass