#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float32MultiArray

# rostopic pub -1 /vision/ball_detector/ball_kick_indicator std_msgs/Int8 -- ARG
def publish_to_left_arm(pose):
    msg = Float32MultiArray()
    msg.data = pose
    left_arm_pub.publish(msg)

def publish_to_right_arm(pose):
    msg = Float32MultiArray()
    msg.data = pose
    right_arm_pub.publish(msg)

def publish_to_head(pose):
    msg = Float32MultiArray()
    msg.data = pose
    head_pub.publish(msg)

def publish_to_left_leg(pose):
    msg = Float32MultiArray()
    msg.data = pose
    left_leg_pub.publish(msg)

def publish_to_right_leg(pose):
    msg = Float32MultiArray()
    msg.data = pose
    right_leg_pub.publish(msg)

def callback_kick_ball(msg):
    # 0 <- initialize ready-to-kick position
    # 1 <- kick the ball
    if msg.data == 0:
        publish_to_head([0.002, 0.414])
        # publish_to_right_leg([-0.264, -0.410, -1.155, 1.104, -0.465, 0.555])
        publish_to_right_leg([-0.457, -0.374, -1.181, 1.267, -0.322, 0.560])
        publish_to_left_leg([-0.012, 0.135, -1.230, 1.307, -0.485, 0.229])
        publish_to_right_arm([-0.158, -0.541, -0.115])
        publish_to_left_arm([-0.193, 0.216, -0.144])
    elif msg.data == 1:
        pass
        publish_to_right_leg([-0.440, -0.362, -1.367, 0.765, 0.029, 0.565])
    else:
        print('No valid instruction received')

def main():
    global right_arm_pub
    global left_arm_pub
    global right_leg_pub
    global left_leg_pub
    global head_pub

    rospy.init_node("ball_kicker")

    rospy.Subscriber("vision/ball_detector/kick_ball_indicator", Int8, callback_kick_ball)

    right_arm_pub = rospy.Publisher("/hardware/arm_right_goal_pose", Float32MultiArray, queue_size = 1)
    left_arm_pub = rospy.Publisher("/hardware/arm_left_goal_pose", Float32MultiArray, queue_size = 1)
    right_leg_pub = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray, queue_size = 1)
    left_leg_pub = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size = 1)
    head_pub = rospy.Publisher("/hardware/head_goal_pose", Float32MultiArray, queue_size = 1)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass