#!/usr/bin/env python3

import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
# Check: https://stackoverflow.com/questions/35145555/python-real-time-plotting-ros-data
class Visualizer:

    def __init__(self):
        self.fig, self.ax = plt.subplots()

        self.sampled_y_pos_plot, = plt.plot([], [], 'ro',label = "[m] (Muestras)", markersize = 2)
        self.measured_y_pos_plot, = plt.plot([], [], label = "[m] (Mediciones)")
        self.estimated_y_pos_plot, = plt.plot([], [], label = "[m] (Filtro Kalman)")
        self.estimated_y_vel_plot, = plt.plot([], [], label = "[m/s] (Filtro Kalman)")

        self.time = []
        self.sampled_y_pos = []
        self.measured_y_pos = []
        self.estimated_y_pos = []
        self.estimated_y_vel = []
        self.y_foot_position = 1000
        self.y_threshold = 1000
        self.kick_time = 1000

        self.ax.set_title("Posición y velocidad en el eje Y")
        self.ax.set_xlabel(xlabel = 'Tiempo [s]')
        self.ax.legend()
        self.ax.grid(True)

    def plot_init(self):
        self.ax.set_xlim(0, 35)
        self.ax.set_ylim(-2, 2)

    def plot_callback(self, msg):
        self.time.append(msg.data[0])
        self.sampled_y_pos.append(msg.data[4] if msg.data[4] < 1000 else None)
        self.measured_y_pos.append(msg.data[6])
        self.estimated_y_pos.append(msg.data[8])
        self.estimated_y_vel.append(msg.data[10])

        self.y_foot_position = msg.data[1]
        self.y_threshold = msg.data[2]
        self.kick_time = msg.data[11]
    
    def update_plot(self, frame):
        self.sampled_y_pos_plot.set_data(self.time, self.sampled_y_pos)
        self.measured_y_pos_plot.set_data(self.time, self.measured_y_pos)
        self.estimated_y_pos_plot.set_data(self.time, self.estimated_y_pos)
        self.estimated_y_vel_plot.set_data(self.time, self.estimated_y_vel)

        plt.axhline(y = self.y_foot_position, linestyle = '--', linewidth = 0.6, color = 'k')
        plt.axhline(y = self.y_threshold, linestyle = '--', linewidth = 0.6, color = 'k') 
        plt.axvline(x = self.kick_time, linestyle = '--', linewidth = 0.8, color = 'k')
