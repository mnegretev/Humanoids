#!/usr/bin/env python3
import socket
import rospy
from gamestate import GameState
from std_msgs.msg import String

def main():
    pub_game_state = rospy.Publisher('/GameControl/GameState', String, queue_size=10)
    pub_secondary_game_state = rospy.Publisher('/GameControl/SecondaryGameState', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    host = rospy.get_param('~host',"0.0.0.0")
    port = 3838
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind((host, port))

    while not rospy.is_shutdown():
        data, addr = server_socket.recvfrom(1024)  


        if len(data) < GameState.sizeof():
            continue


        try:
            gamestate = GameState.parse(data)
        except Exception as e:
            continue
        pub_game_state.publish(gamestate.game_state)
        pub_secondary_game_state.publish(gamestate.secondary_state)
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass