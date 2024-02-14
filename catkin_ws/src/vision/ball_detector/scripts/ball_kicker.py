#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

# rostopic pub -1 /vision/ball_detector/ball_kick_indicator std_msgs/Bool -- True

def publish_to_right_leg(right_leg_pose):
    print("Publishing to the right leg...")
    right_leg_pose_msg = Float32MultiArray()
    right_leg_pose_msg.data = right_leg_pose
    print(right_leg_pose_msg)
    right_leg_pub.publish(right_leg_pose_msg)

def callback_kick_ball(msg):
    if msg.data:
        print('KICK THE BALL')
        publish_to_right_leg([-0.192, -0.151, -1.005, 0.837, -0.252, 0.092])
    else:
        print('I am waiting to kick the ball')

def main():
    global right_leg_pub
    rospy.init_node("ball_kicker")
    rospy.Subscriber("vision/ball_detector/ball_kick_indicator", Bool, callback_kick_ball)
    right_leg_pub = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray, queue_size = 1)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass