#!/usr/bin/env python

import rospy
import pylab
import numpy as np
import matplotlib.pyplot as plt


from pylab import *
from std_msgs.msg import Float32MultiArray
from matplotlib.collections import EventCollection



xAchse=pylab.arange(0,100,1)
yAchse=pylab.array([0]*100)

fig = pylab.figure(1)
ax = fig.add_subplot(111)
ax.grid(True)
ax.set_title("Realtime Velocity in x Plot")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Velocity (m/s)")
ax.axis([0,100,-1.5,1.5])
line1=ax.plot(xAchse,yAchse,'-')

manager = pylab.get_current_fig_manager()

values=[]
values = [0 for x in range(100)]

Ta=0.01
fa=1.0/Ta
fcos=3.5

#Konstant=cos(2*pi*fcos*Ta)
#Konstant=cos(pi*Ta)
Konstant=1
T0=1.0
T1=Konstant




def SinwaveformGenerator(arg):
    global values,Konstant,T1,T0
  #ohmegaCos=arccos(T1)/Ta
  #print "fcos=", ohmegaCos/(2*pi), "Hz"

    #Tnext=((Konstant*T1)*2)-T0
    Tnext=Konstant
#  if len(values)%100>70:
#    values.append(random()*2-1)
#  else:
#    values.append(Tnext)
    values.append(Tnext)
    T0=T1
    T1=Tnext
    #print("Konstant",Konstant)

def RealtimePloter(arg):
  global values
  CurrentXAxis=pylab.arange(len(values)-100,len(values),1)
  line1[0].set_data(CurrentXAxis,pylab.array(values[-100:]))
  ax.axis([CurrentXAxis.min(),CurrentXAxis.max(),-1.5,1.5])
  manager.canvas.draw()
  #print("Konstant",Konstant)
  #print("values",values)
  #print("CurrentXAxis.min()",CurrentXAxis.min())
  #print("CurrentXAxis.max()",CurrentXAxis.max())
  #manager.show()


def callback(data):
    global Konstant
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    Konstant=data.data[0]
    print("Konstant",Konstant)


def graph_velocity_x():
    rospy.init_node('graph_velocity_x_node', anonymous=True)
    rospy.Subscriber("/vision/get_ball_velocity/ball_velocity", Float32MultiArray, callback)
    rate = rospy.Rate(10);
    
 
    timer = fig.canvas.new_timer(interval=10)
    timer.add_callback(RealtimePloter, ())
    timer2 = fig.canvas.new_timer(interval=10)
    timer2.add_callback(SinwaveformGenerator, (Konstant))
    timer.start()
    timer2.start()

    pylab.show()

#    while not rospy.is_shutdown():
#        print("holiii")

#        rate.sleep()

if __name__ == '__main__':
    graph_velocity_x()
 
