#!/usr/bin/env python3

import rospy
import smach as sm

class kick_off(sm.State):
    def __init__(self, outcomes=['next'], input_keys=['is_attacking'], output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

    def execute(self, ud):
        if ud.is_attacking == True:
            #attack method
            return 'start_tracking'
        else:
            rospy.sleep(10)
            return 'start_tracking'

class track(sm.State):
    def __init__(self, outcomes=..., input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

    def execute(self, ud):
        rospy.loginfo('Executing state BAS')