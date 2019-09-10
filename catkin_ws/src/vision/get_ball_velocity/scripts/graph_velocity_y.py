#!/usr/bin/env python
import rospy
import pylab

from pylab import *
from std_msgs.msg import Float32MultiArray


fig = pylab.figure(2)
ax = fig.add_subplot(111)
ax.grid(True)
ax.set_title("Realtime Velocity in y Plot")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Velocity (m/s)")
ax.axis([0,100,-1.5,1.5])

xAchse=pylab.arange(0,100,1)
yAchse=pylab.array([0]*100)
line1=ax.plot(xAchse,yAchse,'-')

values = [0 for x in range(100)]

manager = pylab.get_current_fig_manager()

vy=0

def real_time_plotter(arg):
  global values
  CurrentXAxis=pylab.arange(len(values)-100,len(values),1)
  line1[0].set_data(CurrentXAxis,pylab.array(values[-100:]))
  ax.axis([CurrentXAxis.min(),CurrentXAxis.max(),-1.5,1.5])
  manager.canvas.draw()

def ploting_velocity(arg):
  global values, vy
  Tnext=vy
  values.append(Tnext)

def callback(data):
    global vy
    vy=data.data[1]
    print("vy",vy)


def graph_velocity_y():
    rospy.init_node('graph_velocity_x_node', anonymous=True)
    rospy.Subscriber("/vision/get_ball_velocity/ball_velocity", Float32MultiArray, callback)
        
    timer = fig.canvas.new_timer(interval=10)
    timer.add_callback(real_time_plotter, ())
    timer.start()
    timer2 = fig.canvas.new_timer(interval=10)
    timer2.add_callback(ploting_velocity, ())
    timer2.start()

    pylab.show()

if __name__ == '__main__':
    graph_velocity_y()

 
