##!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import math

FREQ=10

class Imu():
    def __init__(self):
        self.sub    = rospy.Subscriber("/imu/data_raw", Imu, self.callback)
        self.pub    = rospy.Publisher("/imu/humanoid_state", String, queue_size=1)
        self.rate   = rospy.Rate(5)
        self.ay     = 0
        self.ax     = 0

    def callback(self, msg):
        self.ay     = math.atan2(msg.linear_acceleration.x, math.sqrt( msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2)) * 180 / math.pi
        self.ax     = math.atan2(msg.linear_acceleration.y, msth.sqrt( msg.linear_acceleration.x**2 + msg.linear_acceleration.z**2)) * 180 / math.pi
        gx = gx + gyrX / FREQ;
        gy = gy - gyrY / FREQ;
        gz = gz + gyrZ / FREQ;

        gx = gx * 0.96 + self.ax * 0.04;
        gy = gy * 0.96 + self.ay * 0.04;
        


def main ():
	rospy.init_node("head_goal_ball_position_node") 
	rospy.spin()

if __name__=="__main__":
    main()
