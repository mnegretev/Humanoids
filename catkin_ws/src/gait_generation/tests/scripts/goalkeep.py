#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from ctrl_msgs.srv import Getup

def main():
    rospy.init_node("goal_keep")
    pathr = rospy.get_param("pathr")
    pathl = rospy.get_param("pathl")
    rospy.wait_for_service('getup')
    
    pub_leg_left_goal_pose = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_leg_right_goal_pose = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray, queue_size=1)
    pub_arm_left_goal_pose = rospy.Publisher("/hardware/arm_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_arm_right_goal_pose = rospy.Publisher("/hardware/arm_right_goal_pose", Float32MultiArray, queue_size=1)

    rospy.sleep(0.5)  # Esperar a que los publishers estén listos

    msg = Float32MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msga = Float32MultiArray()
    msga.data = [-2.7, 0.0, 0.0]

    pub_leg_right_goal_pose.publish(msg)
    pub_leg_left_goal_pose.publish(msg)
    pub_arm_right_goal_pose.publish(msga)
    pub_arm_left_goal_pose.publish(msga)

    try:
        a= input("Left or Right (L or R):_")
        getup = rospy.ServiceProxy('getup', Getup)
        if a=='L':
          succes = getup(pathl)
        else:
           succes = getup(pathr)
        print(succes)
    except rospy.ServiceException as e:
        print(f"Something went wrong: {e}")

if __name__ == '__main__':
    main()
