#!/usr/bin/env python3

import csv
import rospy
import time
from geometry_msgs.msg import Point

PATH = '/home/andreslopez/data/test3_feb15.csv'
time0 = None

def callback_receive_data(data):

    global time0

    if not time0:
        time0 = time.time()

    with open(PATH, 'a', newline = '') as csv_file:
        csv_writer = csv.writer(csv_file)
        print(f'Writing: {[time.time() - time0, data.x, data.y]}')
        csv_writer.writerow([time.time() - time0, data.x, data.y])

def main():
    rospy.init_node("data_collector")
    rospy.Subscriber("vision/ball_detector/ball_position", Point, callback_receive_data)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass