#!/usr/bin/python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler
import math

FREQ=25

class Imu_Node():
    def __init__(self):
        self.sub    = rospy.Subscriber("/imu/data_raw", Imu, self.callback)
        self.pub    = rospy.Publisher("/imu/humanoid_orientation", Vector3, queue_size=1)
        self.pub_hs = rospy.Publisher("/imu/state", String, queue_size=1)
        self.b      = TransformBroadcaster()
        self.pub_msg= Vector3()
        self.rate   = rospy.Rate(5)
        self.gx     = 0
        self.gy     = 0
        self.gz     = 0
        self.translation    = (0.0, 0.0, 0.0)
        self.rotation       = None#(0.0, 0.0, 0.0, 1.0)

    def callback(self, msg):
        ay     = math.atan2(msg.linear_acceleration.x, math.sqrt( msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2)) * 180 / math.pi
        ax     = math.atan2(msg.linear_acceleration.y, math.sqrt( msg.linear_acceleration.x**2 + msg.linear_acceleration.z**2)) * 180 / math.pi
        
        self.gx     = self.gx + msg.angular_velocity.x / FREQ
        self.gy     = self.gy - msg.angular_velocity.y / FREQ
        self.gz     = self.gz + msg.angular_velocity.z / FREQ

        self.gx = self.gx * 0.96 + ax * 0.04
        self.gy = self.gy * 0.96 + ay * 0.04

        self.rotation = quaternion_from_euler(self.gx, self.gy, self.gz)

        self.b.sendTransform(self.translation, self.rotation, rospy.Time.now(), 'imu_link_rotation', 'imu_link')

        self.pub.publish(self.pub_msg)
        
        if self.gy < -40.0:
            self.pub_hs.publish("fall_front")
        elif self.gy > 40.0:
            self.pub_hs.publish("fall_back")
        else:
            self.pub_hs.publish("straight")




def main ():
    rospy.init_node("head_goal_ball_position_node")
    node = Imu_Node()
    rospy.spin()

if __name__=="__main__":
    main()
