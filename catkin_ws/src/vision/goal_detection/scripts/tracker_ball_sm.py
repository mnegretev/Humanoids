#!/usr/bin/env python
import rospy
import smach
import smach_ros
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Point32
import numpy as np
import time
pan_robot = 0.0
tilt_robot = 0.0
class StartingSearch(smach.State):
    def __init__(self, subs_topic_name, pub_topic_name, rate = 10.0):
        smach.State.__init__(self, outcomes=['interrupted', 'timeout'])
        self.subs_topic_name = subs_topic_name
        self.pub_topic_name = pub_topic_name
        self.rate = rospy.Rate(rate)
        self.message_received = False
        self.subscriber = rospy.Subscriber(self.subs_topic_name, Point32, self.callback)
        self.publisher = rospy.Publisher(self.pub_topic_name, Float32MultiArray, queue_size=1)
    def callback (self, centroid_msg): 
        self.centroid_msg = centroid_msg
        self.message_received = True
        print (self.message_received)
    def execute(self, userdata):
        start_pose1_file = rospy.get_param("~head_point1")
        start_pose2_file = rospy.get_param("~head_point2")
        start_pose3_file = rospy.get_param("~head_point3")
        start_pose4_file = rospy.get_param("~head_point4")
        start_pose1 = np.load(start_pose1_file)
        start_pose2 = np.load(start_pose2_file)
        start_pose3 = np.load(start_pose3_file)
        start_pose4 = np.load(start_pose4_file)
        start_poses = [start_pose1, start_pose2, start_pose3, start_pose4]
        while not rospy.is_shutdown(): 
            print (self.message_received)
            if self.message_received :
                return 'interrupted'
            for start_pose in start_poses:
                timestep = start_pose["timestep"]
                rate = rospy.Rate(int(0.3 / timestep))

            for head in start_pose["head"]:
                if self.message_received:  # Check message_received
                    rospy.loginfo("Centroid detected, changing state...")
                    return 'interrupted'
                cmd_head = Float32MultiArray()
                cmd_head.data = head
                self.publisher.publish(cmd_head)
                rate.sleep()
        return 'timeout'

class BallFound(smach.State):
    def __init__(self, subs_topic_name, pub_topic_name, rate = 10.0):
        smach.State.__init__(self, outcomes=['succ', 'failed'])
        self.subs_topic_name = subs_topic_name
        self.pub_topic_name = pub_topic_name
        self.rate = rospy.Rate(rate)
        self.message_received = False
        self.subscriber = rospy.Subscriber(self.subs_topic_name, Point32, self.callback)
        self.publisher = rospy.Publisher(self.pub_topic_name, Float32MultiArray, queue_size=1)
    def callback (self, centroid_msg): 
        self.centroid_msg = centroid_msg
        self.message_received = True

    def execute(self, userdata):
        global pan_robot, tilt_robot
        while self.message_received :
            center_x = self.centroid_msg.x
            center_y = self.centroid_msg.y
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
            self.publisher.publish(head_cmd)
            self.rate.sleep()
        return 'failed'



def main():
    global ball_detected
    rospy.init_node("tracker_ball_sm")

    rospy.loginfo ('Opening eyes... o_o')

    sm = smach.StateMachine(outcomes=['Exit'])

    with sm:
        smach.StateMachine.add('STARTING_SEARCH_TRAJ',StartingSearch('/centroid_publisher', '/hardware/head_goal_pose'),
                                transitions = {'interrupted': 'TRACKER_BALL',
                                                'timeout': 'STARTING_SEARCH_TRAJ'})

        smach.StateMachine.add('TRACKER_BALL',BallFound('/centroid_publisher', '/hardware/head_goal_pose'),
                                transitions = {'succ': 'TRACKER_BALL',
                                                'failed': 'STARTING_SEARCH_TRAJ'})
    outcome = sm.execute ()

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass