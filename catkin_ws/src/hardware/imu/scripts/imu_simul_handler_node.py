#!/usr/bin/python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker
import math

FREQ=25

class Imu_Node():
    def __init__(self):
        self.sub    = rospy.Subscriber("/imu/data_raw", Imu, self.callback)
        self.pub    = rospy.Publisher("/imu/humanoid_orientation", Vector3, queue_size=1)
        self.pub_hs = rospy.Publisher("/imu/state", String, queue_size=1)
        self.pub_acc= rospy.Publisher("/rviz/acceleration", Marker, queue_size=2)
        self.pub_msg    = Vector3()
        self.rate   = rospy.Rate(5)
        self.gx     = 0
        self.gy     = 0
        self.gz     = 0
        self.acc_marker = Marker()
        self.acc_marker.header.frame_id = "imu_link"
        self.acc_marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.acc_marker.type = 0
        self.acc_marker.id = 0

        # Set the scale of the self.acc_marker
        self.acc_marker.scale.x = 0.02
        self.acc_marker.scale.y = 0.02
        self.acc_marker.scale.z = 0.0

        # Set the color
        self.acc_marker.color.r = 1.0
        self.acc_marker.color.g = 0.0
        self.acc_marker.color.b = 0.0
        self.acc_marker.color.a = 0.7
        # Set start position of arrow (with respect to imu link)
        self.acc_marker.pose.position.x = 0
        self.acc_marker.pose.position.y = 0
        self.acc_marker.pose.position.z = 0


    def callback(self, msg):
        #Calculate unit vector for acceleration (displayed in RViz)
        acc_vec = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        norm_vec = 0.02*sum([ x**2 for x in acc_vec ])**0.5
        # Set the pose of the self.acc_marker
        vec_points = [Point(), Point()]
        vec_points[0].x = 0.0
        vec_points[0].y = 0.0
        vec_points[0].z = 0.0
        vec_points[1].x = acc_vec[0] / norm_vec
        vec_points[1].y = acc_vec[1] / norm_vec
        vec_points[1].z = acc_vec[2] / norm_vec
        self.acc_marker.points = vec_points
        self.pub_acc.publish(self.acc_marker)
        
        #Calculate orientation based of linear acc. and angular vel.
        ay     = math.atan2(msg.linear_acceleration.x, math.sqrt( msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2)) * 180 / math.pi
        ax     = math.atan2(msg.linear_acceleration.y, math.sqrt( msg.linear_acceleration.x**2 + msg.linear_acceleration.z**2)) * 180 / math.pi
        
        self.gx     = self.gx + msg.angular_velocity.x / FREQ
        self.gy     = self.gy - msg.angular_velocity.y / FREQ
        self.gz     = self.gz + msg.angular_velocity.z / FREQ

        self.gx = self.gx * 0.96 + ax * 0.04
        self.gy = self.gy * 0.96 + ay * 0.04

        self.pub_msg.x = self.gx
        self.pub_msg.y = self.gy
        self.pub_msg.z = self.gz

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
