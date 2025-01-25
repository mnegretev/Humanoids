#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_msgs.msg import Float32MultiArray, Bool
import numpy
import time

def head_k_generator(centroid_msg):
	head_move = Point32() 
	
	global pan_robot, tilt_robot
	center_x = centroid_msg.x
	center_y = centroid_msg.y
	print(f"Centroid received {center_x}, {center_y}")
	head_cmd = Float32MultiArray()
	Cx = 320
	Cy = 240
	ex = center_x-Cx
	ey = center_y-Cy
	Kpan = 0.2/320
	Ktilt = 0.1/240
	dead_zone_x = 5
	dead_zone_y= 5


	pan = -Kpan * ex
	tilt = Ktilt * ey

	pan_robot  += pan
	tilt_robot += tilt

	head_cmd.data = [pan_robot, tilt_robot]
	print(f"Publishing pan_angle: {pan_robot}, tilt: {tilt_robot}")
	head_move.x = pan_robot
	head_move.y = tilt_robot


class StartingSearch(smach.State):
    def __init__(self, pub_head_goal,rate):
        smach.State.__init__(self, outcomes=['succ', 'fail'])
        self.state = "INIT"

    def execute(self, userdata):
        global ball_detected
                 
        return 'succ'

class BallFound(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ', 'fail'])
        self.state = "INIT"

    def execute(self, userdata):
        rospy.loginfo('STATE MACHINE WALK ->' + self.state)
        start()
        return 'succ'



def main():
    global ball_detected
    rospy.init_node("tracker_ball_sm")
    rospy.Subscriber("/ball_found_publisher", Point32, BallFound)

    start_pose1 = np.load("head_point1.npz")
    start_pose2 = np.load("head_point2.npz")
    start_pose3 = np.load("head_point3.npz")
    start_pose4 = np.load("head_point4.npz")
    timstep = start_pose1["timestep"]
    rate = rospy.Rate(int(0.3/(timstep)))

    for head in zip(start_pose1["head"]):     
            print(head)
            cmd_head = Float32MultiArray()
            cmd_head.data = head[0]
            pub_head_goal.publish(cmd_head)
            rate.sleep()     
        print ("First move finished")
        for head in zip(start_pose2["head"]):     
            print(head)
            cmd_head = Float32MultiArray()
            cmd_head.data = head[0]
            pub_head_goal.publish(cmd_head)
            rate.sleep()
        print ("Second move finished")
        for head in zip(start_pose3["head"]):     
            print(head)
            cmd_head = Float32MultiArray()
            cmd_head.data = head[0]
            pub_head_goal.publish(cmd_head)
            rate.sleep()
        print ("Third move finished")
        for head in zip(start_pose4["head"]):     
            print(head)
            cmd_head = Float32MultiArray()
            cmd_head.data = head[0]
            pub_head_goal.publish(cmd_head)
            rate.sleep()

    rospy.loginfo ('Opening eyes... o_o')

    sm = smach.StateMachine(outcomes=['Exit'])

    with sm:
        smach.StateMachine.add('STARTING_SEARCH_TRAJ',StartingSearch1()
                                transitions = {'succ': 'TRACKER_BALL',
                                                'fail': 'STARTING_SEARCH_TRAJ'})

        smach.StateMachine.add('TRACKER_BALL',BallFound()
                                transitions = {'succ': 'TRACKER_BALL',
                                                'fail': 'STARTING_SEARCH_TRAJ2'})
    outcome = sm.execute

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass