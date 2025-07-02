#!/usr/bin/env python3
import socket
import rospy
from gamestate import GameState
from game_controller_msgs.msg import GameStateInfo
from std_msgs.msg import String

def main():
    pub_game_state = rospy.Publisher('/GameControl/GameState', GameStateInfo, queue_size=10)
    #pub_secondary_game_state = rospy.Publisher('/GameControl/SecondaryGameState', String, queue_size=10)
    rospy.init_node('GameStateNode', anonymous=True)
    host = rospy.get_param('~host',"0.0.0.0")
    port = 3838
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    server_socket.bind((host, port))

    while not rospy.is_shutdown():
        data, addr = server_socket.recvfrom(1024)  
        try:
            gamestate = GameState.parse(data)
        except Exception as e:
            print (f"Un error ha ocurrido {e}")
            continue

        gamestate_msg = GameStateInfo()
        
        gamestate_msg.gc_header = gamestate.header.decode("utf-8")
        gamestate_msg.protocol_version = gamestate.version
        gamestate_msg.packet_number = gamestate.packet_number
        gamestate_msg.players_per_team = gamestate.players_per_team
        gamestate_msg.game_type = gamestate.game_type
        gamestate_msg.state = gamestate.game_state
        gamestate_msg.first_half = gamestate.first_half
        gamestate_msg.kick_off_team = gamestate.kick_of_team
        gamestate_msg.secondary_state = gamestate.secondary_state
        gamestate_msg.secondary_state_info = gamestate.secondary_state_info
        gamestate_msg.drop_in_team = gamestate.drop_in_team
        gamestate_msg.drop_in_time = gamestate.drop_in_time
        gamestate_msg.secs_remaining = gamestate.seconds_remaining
        gamestate_msg.secondary_time = gamestate.secondary_seconds_remaining

        pub_game_state.publish(gamestate_msg)    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass