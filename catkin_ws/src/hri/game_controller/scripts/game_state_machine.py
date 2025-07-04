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


class initial(sm.State):
    def __init__(self, outcomes=['success','failed'], input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

    def execute(self, ud):
        while(rospy.wait_for_message('/GameController/GameState').state == 'INITIAL'):
            return 'failed'
        return 'success'
    
class positioning(sm.State):
    def __init__(self, outcomes=["in_place", "not_in_place"], input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

    def execute(self, ud):
        #IR A SU LUGAR
        print("El robot está yendo a su lugar")
        return "in_place"
    
class set_wait(sm.State):
    def __init__(self, outcomes=["play"], input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

    def execute(self, ud):
        rospy.sleep(10)
        return "play"

class set_kick(sm.State):
    def __init__(self, outcomes=["play"], input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

    def execute(self, ud):
        print("start kicking")
        return "play"
    
class free_kick(sm.State):
    def __init__(self, outcomes=["success", "fail"], input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

    def execute(self, ud):
        print("start free kick")
        return "play"
    
class goal_kick(sm.State):
    def __init__(self, outcomes=["success", "fail"], input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

    def execute(self, ud):
        print("start free kick")
        return "play"
    
class corner_kick(sm.State):
    def __init__(self, outcomes=["success", "fail"], input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

    def execute(self, ud):
        print("start free kick")
        return "play"
    
class throw_kick(sm.State):
    def __init__(self, outcomes=["success", "fail"], input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

    def execute(self, ud):
        print("start free kick")
        return "play"
    
class penalty_kick(sm.State):
    def __init__(self, outcomes=["success", "fail"], input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

    def execute(self, ud):
        print("start free kick")
        return "play"
    
class ball_in_play(sm.State):
    def __init__(self, outcomes=["playing","goal","ball_out","foul", "incapable_robot", ""], input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

    def execute(self, ud):
        if(game_state == "PLAYING"):
            print("playing")
            return "play"
        if(goal_var == True):
            print("playing")
            return "play"
        if(subgame_state == "PLAYING"):
            print("playing")
            return "play"
        if(subgame_state == "PLAYING"):
            print("playing")
            return "play"
        
def main():

    main_sm = sm.StateMachine(['end','rappi'])
    ready_sm = sm.StateMachine(['success','failed'])
    playing_sm = sm.StateMachine(["playing","goal","ball_out","foul", "incapable_robot", ""])

    with main_sm:
        sm.StateMachine.add('Init',initial(),transitions=)
