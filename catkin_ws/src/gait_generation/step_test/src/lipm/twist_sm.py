#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import smach
import smach_ros

def start():
    for right, left in zip(start_pose["right"], start_pose["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        start_rate.sleep()

def first_step():
    for right, left in zip(left_first_step["right"], left_first_step["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        first_rate.sleep()


def move_com():
    for right, left in zip(twist_move_com_left["right"], twist_move_com_left["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        second_rate.sleep()

def tirth_step():
    for right, left in zip(twist_right_third_step["right"], twist_right_third_step["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        tirth_rate.sleep()

def final_step():
    for right, left in zip(twist_right_final_stop["right"], twist_right_final_stop["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        fourth_rate.sleep()


class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'])
        self.state = "INIT"

    def execute(self, userdata):
        rospy.loginfo('STATE MACHINE TWIST ->' + self.state)
        start()
        return 'succ'
    


def main():
    global pub_leg_left_goal_pose, pub_leg_right_goal_pose, start_pose, left_first_step, twist_move_com_left, twist_right_third_step, twist_right_final_stop, start_rate, first_rate, second_rate,tirth_rate,fourth_rate
    rospy.init_node("step_test_node")
    pub_leg_left_goal_pose = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_leg_right_goal_pose = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray , queue_size=1)
    start_pose_file = rospy.get_param("~twist_start_pose")
    start_pose = np.load(start_pose_file)
    timstep = start_pose["timestep"]
    start_rate = rospy.Rate(int(1/(timstep)))

    left_first_step_pose_file = rospy.get_param("~twist_left_first_step")
    left_first_step = np.load(left_first_step_pose_file)
    timstep = left_first_step["timestep"]
    first_rate = rospy.Rate(int(1/(timstep)*3))


    twist_move_com_left_file = rospy.get_param("~twist_move_com_left")
    twist_move_com_left = np.load(twist_move_com_left_file)
    timstep = twist_move_com_left["timestep"]
    second_rate = rospy.Rate(int(1/(timstep))*3)


    twist_right_third_step_file = rospy.get_param("~twist_right_third_step")
    twist_right_third_step = np.load(twist_right_third_step_file)
    timstep = twist_right_third_step["timestep"]
    tirth_rate = rospy.Rate(int(1/(timstep))*3)

    twist_right_final_stop_file = rospy.get_param("~twist_right_final_stop")
    twist_right_final_stop = np.load(twist_right_final_stop_file)
    timstep = twist_right_final_stop["timestep"]
    fourth_rate = rospy.Rate(int(1/(timstep))*3)

    while True:
        start()
        first_step()
        move_com()
        tirth_step()
        final_step()
    # sm = smach.StateMachine(outcomes=['exit'])


    # with sm:
    
    #     smach.StateMachine.add('Initial', Initial(),
    #                            transitions={'succ': 'Crouch',
    #                                         'fail': 'Initial'})
    #     smach.StateMachine.add('Crouch', Crouch(),
    #                            transitions={'succ': 'Full_step_Right',
    #                                         'repeat': 'Crouch'})
    #     smach.StateMachine.add('Full_step_Right', Full_step_Right(),
    #                            transitions={'succ': 'Full_step_Left', 
    #                                         'left': 'Half_step_Right'})
    #     smach.StateMachine.add('Full_step_Left', Full_step_Left(), 
    #                             transitions={'succ': 'Full_step_Right', 
    #                                          'right': 'Half_step_Left'})
    #     smach.StateMachine.add('Half_step_Left', Half_step_Left(), 
    #                             transitions={'succ': 'Crouch', 
    #                                          'fail': 'Half_step_Left',
    #                                          'end' : 'get_up'})
    #     smach.StateMachine.add('Half_step_Right', Half_step_Right(), 
    #                             transitions={'succ': 'Crouch', 
    #                                          'fail': 'Half_step_Left',
    #                                          'end' : 'get_up'})
    #     smach.StateMachine.add('get_up', get_up(), 
    #                             transitions={'succ': 'Initial', 
    #                                          'fail': 'get_up'})


    # outcome = sm.execute()

if __name__ == '__main__':
    main()
