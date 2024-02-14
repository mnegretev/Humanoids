#!/usr/bin/env python3

import csv
import rospy
from geometry_msgs.msg import Point
from pprint import pprint

DATA_PATH = "/home/andres/Andres/humanoid_computer_vision/catkin_ws/src/experiments/scripts/data_plotting/data/test1_feb6.csv"

def read_positions(path):
    # row[1] : measured x-position
    # row[2] : measured y-position

    with open(path, newline = '') as csv_file:
        positions = [[float(row[1]), float(row[2])] for row in csv.reader(csv_file)] 
    
    return positions


def main():
    rospy.init_node("data_generator")
    data_pub = rospy.Publisher("vision/ball_detector/ball_position", Point, queue_size = 10)
    pos_data = read_positions(path = DATA_PATH)
    rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown() and i < len(pos_data):
        print(f"{i} : ", pos_data[i])
        measured_x = pos_data[i][0]
        measured_y = pos_data[i][1]
        i += 1
        ball_pos = Point()
        ball_pos.x = measured_x
        ball_pos.y = measured_y
        data_pub.publish(ball_pos)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass