#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_msgs.msg import Float32MultiArray, Bool, Float32
import numpy

def start():
    # Inicio del ciclo para adquirir la pose inicial
    for right, left in zip(start_pose["right"], start_pose["left"]):
        print(right)
        print(left)
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        rate.sleep()
    
    #time.sleep(2)

def end():
    # Inicio del ciclo para adquirir la pose inicial
    for right, left in zip(end_pose["right"], end_pose["left"]):
        print(right)
        print(left)
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
        print(right)
        print(left)
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = right
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = left
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        fast_rate.sleep()
    #time.sleep(2)
def end_step_right():
    # Paso derecho 
        right_full_step_file = rospy.get_param("~right_end_step")
        second_step = numpy .load(right_full_step_file)
        for right, left in zip(second_step["right"], second_step["left"]):
            print(right)
            print(left)
            right_leg_goal_pose = Float32MultiArray()
            right_leg_goal_pose.data = right
            pub_leg_right_goal_pose.publish(right_leg_goal_pose)

            left_leg_goal_pose = Float32MultiArray()
            left_leg_goal_pose.data = left
            pub_leg_left_goal_pose.publish(left_leg_goal_pose)
            middle_rate.sleep()

def end_step_left():
    #Paso izquierdo 
        left_full_step_file = rospy.get_param("~left_end_step")
        third_step = numpy .load(left_full_step_file)
        for right, left in zip(third_step["right"], third_step["left"]):
            print(right)
            print(left)
            right_leg_goal_pose = Float32MultiArray()
            right_leg_goal_pose.data = right
            pub_leg_right_goal_pose.publish(right_leg_goal_pose)

            left_leg_goal_pose = Float32MultiArray()
            left_leg_goal_pose.data = left
            pub_leg_left_goal_pose.publish(left_leg_goal_pose)
            middle_rate.sleep()

def right():
    # Inicio del ciclo de caminata 

    # Paso derecho 
        right_full_step_file = rospy.get_param("~right_full_step")
        second_step = numpy .load(right_full_step_file)
        for right, left in zip(second_step["right"], second_step["left"]):
            print(right)
            print(left)
            right_leg_goal_pose = Float32MultiArray()
            right_leg_goal_pose.data = right
            pub_leg_right_goal_pose.publish(right_leg_goal_pose)

            left_leg_goal_pose = Float32MultiArray()
            left_leg_goal_pose.data = left
            pub_leg_left_goal_pose.publish(left_leg_goal_pose)
            middle_rate.sleep()

def left():
    #Paso izquierdo 
        left_full_step_file = rospy.get_param("~left_full_step")
        third_step = numpy .load(left_full_step_file)
        for right, left in zip(third_step["right"], third_step["left"]):
            print(right)
            print(left)
            right_leg_goal_pose = Float32MultiArray()
            right_leg_goal_pose.data = right
            pub_leg_right_goal_pose.publish(right_leg_goal_pose)

            left_leg_goal_pose = Float32MultiArray()
            left_leg_goal_pose.data = left
            pub_leg_left_goal_pose.publish(left_leg_goal_pose)
            middle_rate.sleep()


def callback(data):
    global walk_state 
    walk_state = data.data
    print(walk_state)

def callback_ball(data):
    global steps
    distance = data.data
    steps=distance/0.036
    print(step)

def callback_end(data):
    global end_state
    end_state = data.data
    print(end_state)

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'])
        self.state = "INIT"

    def execute(self, userdata):
        global step, walk_state, end_state
        rospy.loginfo('STATE MACHINE WALK ->' + self.state)
        right_leg_goal_pose = Float32MultiArray()
        right_leg_goal_pose.data = [0.0,0.0,0.0,0.0,0.0,0.0]
        pub_leg_right_goal_pose.publish(right_leg_goal_pose)

        left_leg_goal_pose = Float32MultiArray()
        left_leg_goal_pose.data = [0.0,0.0,0.0,0.0,0.0,0.0]
        pub_leg_left_goal_pose.publish(left_leg_goal_pose)
        middle_rate.sleep()
        step=rospy.wait_for_message("/ball_position", Float32, timeout=None)
        walk_state=True
        end_state=True
        step = (step.data)/0.036
        start()
        return 'succ'
    
class Crouch(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'repeat'])
        self.state = "CROUCH"

    def execute(self, userdata):
        rospy.loginfo('STATE MACHINE WALK -> ' + self.state)
        if walk_state == True:
            init_pose()
            return 'succ'
        else:
            return 'repeat'

class Full_step_Right(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'left'])
        self.state = "RIGHT_STEP"

    def execute(self, userdata):
        global step, walk_state
        rospy.loginfo('STATE MACHINE WALK -> ' + self.state)
        if walk_state == True and step >0:
            right()
            step -=1
            return 'succ'
        else:
            walk_state=False
            return 'left'
    
class Full_step_Left(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ','right'])
        self.state = "LEFT_STEP"

    def execute(self, userdata):
        global step, walk_state
        rospy.loginfo('STATE MACHINE WALK -> ' + self.state)
        if walk_state == True and step >0:
            left()
            step -=1
            return 'succ'
        else:
            walk_state=False
            return 'right'

class Half_step_Right(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail', 'end'])
        self.state = "RIGHT_END_STEP"

    def execute(self, userdata):
        rospy.loginfo('STATE MACHINE WALK -> ' + self.state)
        end_step_right()
        if end_state:
            return 'end'
        else:
            return 'succ'

class Half_step_Left(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail', 'end'])
        self.state = "LEFT_END_STEP"

    def execute(self, userdata):
        rospy.loginfo('STATE MACHINE WALK -> ' + self.state)
        end_step_left()
        if end_state:
            return 'end'
        else:
            return 'succ'
        
class get_up(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'])
        self.state = "GET_UP"

    def execute(self, userdata):
        global end_state
        rospy.loginfo('STATE MACHINE WALK -> ' + self.state)
        if end_state:
            end()
            end_state=False
        if walk_state:
            return 'succ'
        
        return 'fail'
        


def main():
    global pub_leg_left_goal_pose, pub_leg_right_goal_pose, walk_state,end_state, rate, middle_rate, fast_rate, start_pose, end_pose
    rospy.init_node("walk_sm")
    pub_leg_left_goal_pose = rospy.Publisher("/hardware/leg_left_goal_pose", Float32MultiArray, queue_size=1)
    pub_leg_right_goal_pose = rospy.Publisher("/hardware/leg_right_goal_pose", Float32MultiArray , queue_size=1)
    walk_state=False
    end_state=False
    start_pose_file = rospy.get_param("~start_pose")
    start_pose = numpy .load(start_pose_file)
    end_pose_file = rospy.get_param("~end_pose")
    end_pose = numpy .load(end_pose_file)
    timstep = start_pose["timestep"]
    # rate = rospy.Rate(10)
    # middle_rate = rospy.Rate(20)
    # fast_rate=rospy.Rate(30)
    rate = rospy.Rate(int(1/(timstep)))
    middle_rate = rospy.Rate(int(1/(timstep)))
    fast_rate = rospy.Rate(int(1/(timstep/3)))
    rospy.Subscriber("/walk_state", Bool, callback)
    rospy.Subscriber("/end_sm", Bool, callback_end)
    rospy.Subscriber("/ball_position", Float32, callback_ball)

    

    sm = smach.StateMachine(outcomes=['exit'])


    with sm:
    
        smach.StateMachine.add('Initial', Initial(),
                               transitions={'succ': 'Crouch',
                                            'fail': 'Initial'})
        smach.StateMachine.add('Crouch', Crouch(),
                               transitions={'succ': 'Full_step_Right',
                                            'repeat': 'Crouch'})
        smach.StateMachine.add('Full_step_Right', Full_step_Right(),
                               transitions={'succ': 'Full_step_Left', 
                                            'left': 'Half_step_Right'})
        smach.StateMachine.add('Full_step_Left', Full_step_Left(), 
                                transitions={'succ': 'Full_step_Right', 
                                             'right': 'Half_step_Left'})
        smach.StateMachine.add('Half_step_Left', Half_step_Left(), 
                                transitions={'succ': 'Crouch', 
                                             'fail': 'Half_step_Left',
                                             'end' : 'get_up'})
        smach.StateMachine.add('Half_step_Right', Half_step_Right(), 
                                transitions={'succ': 'Crouch', 
                                             'fail': 'Half_step_Left',
                                             'end' : 'get_up'})
        smach.StateMachine.add('get_up', get_up(), 
                                transitions={'succ': 'Initial', 
                                             'fail': 'get_up'})


    outcome = sm.execute()

if __name__ == '__main__':
    main()
