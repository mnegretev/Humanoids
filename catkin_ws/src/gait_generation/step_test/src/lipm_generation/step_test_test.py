#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Bool
import numpy  
import time

def start():
    # Inicio del ciclo para adquirir la pose inicial
    for right, left in zip(start_pose["right"], start_pose["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        rate.sleep()
    
    #time.sleep(2)

def init_pose():
    # Inicio del ciclo para adquirir la pose de inicio de la caminata

    first_half_step_file = rospy.get_param("~left_first_halfstep")
    first_half_step = numpy .load(first_half_step_file)
    for right, left in zip(first_half_step["right"], first_half_step["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        fast_rate.sleep()
    #time.sleep(2)

def walk_cycle():
    # Inicio del ciclo de caminata 

    # Paso derecho 
        right_full_step_file = rospy.get_param("~right_full_step")
        second_step = numpy .load(right_full_step_file)
        for right, left in zip(second_step["right"], second_step["left"]):
            right_leg_goal_pose = Float32MultiArray()
            right_leg_goal_pose.data = right
            pub_leg_right_goal_pose.publish(right_leg_goal_pose)

            left_leg_goal_pose = Float32MultiArray()
            left_leg_goal_pose.data = left
            pub_leg_left_goal_pose.publish(left_leg_goal_pose)
            middle_rate.sleep()

    #Paso izquierdo 
        left_full_step_file = rospy.get_param("~left_full_step")
        third_step = numpy .load(left_full_step_file)
        for right, left in zip(third_step["right"], third_step["left"]):
            right_leg_goal_pose = Float32MultiArray()
            right_leg_goal_pose.data = right
            pub_leg_right_goal_pose.publish(right_leg_goal_pose)

            left_leg_goal_pose = Float32MultiArray()
            left_leg_goal_pose.data = left
            pub_leg_left_goal_pose.publish(left_leg_goal_pose)
            middle_rate.sleep()

def callback(data):
    global walk_state 
    if data.data == True:
        start()
        init_pose()
    walk_state = data.data
    print(walk_state)


def main():
    global pub_leg_left_goal_pose, pub_leg_right_goal_pose, walk_state, rate, middle_rate, fast_rate, start_pose
    rospy.init_node("step_test_node")
    pub_leg_left_goal_pose = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_leg_right_goal_pose = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray , queue_size=1)
    walk_state=False
    start_pose_file = rospy.get_param("~start_pose")
    start_pose = numpy .load(start_pose_file)
    timstep = start_pose["timestep"]
    rate = rospy.Rate(int(1/(timstep)))
    middle_rate = rospy.Rate(int(1/(timstep)))
    fast_rate = rospy.Rate(int(1/(timstep/3)))
    right = start_pose["right"][5]  
    left = start_pose["left"][5]   
    print(right)
    print(left)
    right_leg_goal_pose = Float32MultiArray()
    right_leg_goal_pose.data = right
    pub_leg_right_goal_pose.publish(right_leg_goal_pose)

    left_leg_goal_pose = Float32MultiArray()
    left_leg_goal_pose.data = left
    pub_leg_left_goal_pose.publish(left_leg_goal_pose)

    rate.sleep()
    rospy.Subscriber("/walk_state", Bool, callback)
    while not rospy.is_shutdown():
        if walk_state ==True:
            walk_cycle()
        else: 
            pub_leg_right_goal_pose.publish(right_leg_goal_pose)
            pub_leg_left_goal_pose.publish(left_leg_goal_pose)
            rospy.sleep(1)
        
    rospy.spin()
    
    

if __name__ == '__main__':
    exit(main())