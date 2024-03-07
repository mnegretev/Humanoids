#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ctrl_msgs.srv import CalculateDK
from ctrl_msgs.srv import CalculateIK

def main():
    
    print("Initializing step node by miguel")
    rospy.init_node("step_test")
    
    pubLegLeftGoalPose = rospy.Publisher("/hardware/leg_left_goal_pose", queue_size=1)
    pubRightLegGoalPose = rospy.Publisher("/hardware/leg_right_goal_pose", queue_size=10)

    cltCalculateIKLegLeft = rospy.ServiceProxy("/control/ik_leg_left", CalculateIK);
    cltCalculateIKLegLeft = rospy.ServiceProxy("/control/ik_leg_right", CalculateIK);

    
    pass