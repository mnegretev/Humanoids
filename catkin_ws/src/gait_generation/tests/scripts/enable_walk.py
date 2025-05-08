#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

def walker():
    pub = rospy.Publisher('walker', Bool, queue_size=10)
    rospy.init_node('walker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    walk=False
    while not rospy.is_shutdown():
        input("Enter para avanzar")
        walk=True
        pub.publish(walk)
        input("Enter para parar")
        walk=False
        pub.publish(walk)

if __name__ == '__main__':
    try:
        walker()
    except rospy.ROSInterruptException:
        pass