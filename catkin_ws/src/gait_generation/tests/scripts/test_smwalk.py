#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, Bool, Float32
import numpy
import smach
import smach_ros

def  Dotraj(file):
    timestep = file["timestep"]
    rate = rospy.Rate(int(1/timestep))
    for right, left in zip(file["right"], file["left"]):
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        right_leg_pub.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        left_leg_pub.publish(left_leg_goal_pose)
        rate.sleep()

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'],
                             input_keys=['file' ])
        self.state = "INIT"

    def execute(self, userdata):
        print(f"Walk_test_sm------------>{self.state}")
        try:
            Dotraj(userdata.file)
            return 'succ'
        except Exception as e:
            print(f"Ocurrio un error {e}")
            return 'fail'
            
class First_step(smach.State):
    def __init__(self):
       smach.State.__init__(self, outcomes=['succ', 'fail'],
                             input_keys=['file' ])
       self.state = "First"

    def execute(self, userdata):
        print(f"Walk_test_sm------------>{self.state}")
        try:
            Dotraj(userdata.file)
            return 'succ'
        except Exception as e:
            print(f"Ocurrio un error {e}")
            return 'fail'
            
class Right_full(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail', 'end'],
                             input_keys=['file' ])
        self.state = "R_full_step"

    def execute(self, userdata):
        print(f"Walk_test_sm------------>{self.state}")
        try:
            Dotraj(userdata.file)
            return 'succ'
        except Exception as e:
            print(f"Ocurrio un error {e}")
            return 'fail'
            
class Left_full(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail', 'end'],
                             input_keys=['file' ])
        self.state = "L_full_step"

    def execute(self, userdata):
        print(f"Walk_test_sm------------>{self.state}")
        try:
            Dotraj(userdata.file)
            return 'succ'
        except Exception as e:
            print(f"Ocurrio un error {e}")
            return 'fail'
            
class Right_end(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'],
                             input_keys=['file' ])
        self.state = "R_end_step"

    def execute(self, userdata):
        print(f"Walk_test_sm------------>{self.state}")
        try:
            Dotraj(userdata.file)
            return 'succ'
        except Exception as e:
            print(f"Ocurrio un error {e}")
            return 'fail'
            
class Left_end(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'],
                             input_keys=['file' ])
        self.state = "L_end_step"

    def execute(self, userdata):
        print(f"Walk_test_sm------------>{self.state}")
        try:
            Dotraj(userdata.file)
            return 'succ'
        except Exception as e:
            print(f"Ocurrio un error {e}")
            return 'fail'
        
class End(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'],
                             input_keys=['file' ])
        self.state = "End"

    def execute(self, userdata):
        print(f"Walk_test_sm------------>{self.state}")
        try:
            Dotraj(userdata.file)
            return 'succ'
        except Exception as e:
            print(f"Ocurrio un error {e}")
            return 'fail'
            

def main():
    global left_leg_pub, right_leg_pub
    rospy.init_node("walk_test")
    left_leg_pub = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size=1)
    right_leg_pub = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray, queue_size=1)
    start_pose_file = rospy.get_param("~start_pose")
    first_half_step_file = rospy.get_param("~left_first_halfstep")
    right_full_step_file = rospy.get_param("~right_full_step")
    left_full_step_file = rospy.get_param("~left_full_step")
    right_end_step_file = rospy.get_param("~right_end_step")
    left_end_step_file = rospy.get_param("~left_end_step")
    end_pose_file = rospy.get_param("~end_pose")
    sm=smach.StateMachine(outcomes=['exit'])

    sm.userdata.file_start_pose = numpy .load(start_pose_file)
    sm.userdata.file_first_half_step = numpy .load(first_half_step_file)
    sm.userdata.file_second_step = numpy .load(right_full_step_file)
    sm.userdata.file_third_step = numpy .load(left_full_step_file)
    sm.userdata.file_fourthr = numpy .load(right_end_step_file)
    sm.userdata.file_fourthl = numpy .load(left_end_step_file)
    sm.userdata.file_end_pose = numpy .load(end_pose_file)

    with sm:
    
        smach.StateMachine.add('Init', Init(),
                               transitions={'succ': 'First',
                                            'fail': 'Init'},
                                remapping={'file': 'file_start_pose'})
        smach.StateMachine.add('First', First_step(),
                               transitions={'succ': 'Second',
                                            'fail': 'First'},
                                remapping={'file': 'file_first_half_step'})
        smach.StateMachine.add('Second', Right_full(),
                               transitions={'succ': 'Tirth',
                                            'fail': 'Init',
                                            'end': 'Fourthl'},
                                remapping={'file': 'file_second_step'})
        smach.StateMachine.add('Tirth', Left_full(),
                               transitions={'succ': 'First',
                                            'fail': 'Init',
                                            'end': 'Fourthr'},
                                remapping={'file': 'file_third_step'})
        smach.StateMachine.add('Fourthr', Right_end(),
                               transitions={'succ': 'End',
                                            'fail': 'Fourthr'},
                                remapping={'file': 'file_fourthr'})
        smach.StateMachine.add('Fourthl', Left_end(),
                               transitions={'succ': 'End',
                                            'fail': 'Fourthl'},
                                remapping={'file': 'file_fourthl'})
        smach.StateMachine.add('End', End(),
                               transitions={'succ': 'exit',
                                            'fail': 'End'},
                                remapping={'file': 'file_end_pose'})


    outcome = sm.execute()

if __name__ == '__main__':
    main()
   


