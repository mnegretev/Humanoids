#!/usr/bin/env python
import math
import rospy
import smach
import smach_ros
from std_msgs.msg import Float32MultiArray, Bool, Float32
from geometry_msgs.msg import Point32
import numpy as np
import time
pan_robot = 0.0
tilt_robot = 0.0
class StartingSearch(smach.State):
	def __init__(self, subs_topic_name, pub_topic_name, rate = 10.0):
		smach.State.__init__(self, outcomes=['interrupted', 'timeout'],
					output_keys = ['last_pos'],
					input_keys = ['last_pos_out'])
		self.subs_topic_name = subs_topic_name
		self.pub_topic_name = pub_topic_name
		self.rate = rospy.Rate(rate)
		self.message_received = False
		self.subscriber = rospy.Subscriber(self.subs_topic_name, Point32, self.callback)
		self.publisher = rospy.Publisher(self.pub_topic_name, Float32MultiArray, queue_size=1)
		self.tries = 0

	def callback (self, centroid_msg): 
		self.centroid_msg = centroid_msg
		self.message_received = True

	def execute(self, userdata):
		self.tries += 1
		start_pose1_file = rospy.get_param("~head_point1")
		start_pose2_file = rospy.get_param("~head_point2")
		start_pose3_file = rospy.get_param("~head_point3")
		start_pose4_file = rospy.get_param("~head_point4")
		start_pose1 = np.load(start_pose1_file)
		start_pose2 = np.load(start_pose2_file)
		start_pose3 = np.load(start_pose3_file)
		start_pose4 = np.load(start_pose4_file)
		start_poses = [start_pose1, start_pose2, start_pose3, start_pose4]
		cmd_head = Float32MultiArray()
		if self.tries > 1 :
			first_head_pose = start_pose1["head"][0]
			last_pos_out = userdata.last_pos_out
			steps = 20
			interp_pan = np.linspace(last_pos_out[0], first_head_pose[0], steps)
			interp_tilt = np.linspace(last_pos_out[1], first_head_pose[1], steps)
			for pan, tilt in zip(interp_pan, interp_tilt):
				cmd_head.data = [pan, tilt]
				self.publisher.publish(cmd_head)
				self.rate.sleep()
			self.message_received = False
		while not rospy.is_shutdown():
			timestep = start_pose1["timestep"]
			rate = rospy.Rate(int(0.3 / timestep))
			for start_pose in start_poses :
				for head in start_pose["head"]:
					if self.message_received:  # Check message_received
						self.message_received = False
						rospy.loginfo("Centroid detected, changing state...")
						return 'interrupted'
					cmd_head.data = head
					userdata.last_pos = cmd_head.data
					self.publisher.publish(cmd_head)
					rate.sleep()
		return 'timeout'

class BallFound(smach.State):
	def __init__(self, subs_topic_name, pub_topic_name, rate = 10.0):
		smach.State.__init__(self, outcomes = ['succ', 'failed'],
					input_keys = ['last_pos'],
					output_keys = ['last_pos_out'])
		self.subs_topic_name = subs_topic_name
		self.pub_topic_name = pub_topic_name
		self.rate = rospy.Rate(rate)
		self.message_received = False
		self.subscriber = rospy.Subscriber(self.subs_topic_name, Point32, self.callback)
		self.tries = 0
		self.publisher = rospy.Publisher(self.pub_topic_name, Float32MultiArray, queue_size=1)
		self.ball_pos_pub = rospy.Publisher("/ball_position", Float32, queue_size=1)
	def callback (self, centroid_msg): 
		self.centroid_msg = centroid_msg
		self.message_received = True

	def execute(self, userdata):
		self.tries +=1
		global pan_robot, tilt_robot
		#if self.tries == 1:
		last_pos = userdata.last_pos
		pan_robot, tilt_robot = last_pos
		while self.message_received :
			center_x = self.centroid_msg.x
			center_y = self.centroid_msg.y
			print(f"Centroid received {center_x}, {center_y}")
			head_cmd = Float32MultiArray()
			Cx = 320
			Cy = 240
			ex = center_x-Cx
			ey = center_y-Cy
			Kpan = 0.05/320
			Ktilt = 0.025/240
			pan = -Kpan * ex
			tilt = Ktilt * ey
			pan_robot  += pan
			tilt_robot += tilt
			head_cmd.data = [pan_robot, tilt_robot]
			print(f"Publishing pan_angle: {pan_robot}, tilt: {tilt_robot}")
			distance = 0.85/math.tan(tilt_robot)
			msg = Float32()
			msg.data = distance
			self.ball_pos_pub.publish(msg)
			print(f'Ball distance is: {distance:.2f} meters.')
			self.publisher.publish(head_cmd)
			userdata.last_pos_out = [pan_robot, tilt_robot]
			self.rate.sleep()
			try:
				self.centroid_msg = rospy.wait_for_message(self.subs_topic_name, Point32, timeout=5)
			except rospy.ROSException:
				rospy.loginfo('Lost the ball while tracking')
				self.message_received = False
				return 'failed'
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
