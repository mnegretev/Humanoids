#!/usr/bin/env python3
import matplotlib.pyplot as plt
import rospy
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32MultiArray
from visualizer import Visualizer

def main():
    vis = Visualizer()
    rospy.init_node("plotter")
    rospy.Subscriber("vision/ball_detector/estimations", Float32MultiArray, vis.plot_callback)
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    plt.show(block=True) 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass