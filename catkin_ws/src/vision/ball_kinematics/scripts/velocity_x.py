#!/usr/bin/env python
import rospy
import pylab

from pylab import *
from std_msgs.msg import Float32MultiArray

plt.rcParams['figure.figsize'] = (10, 8)

fig = pylab.figure(2)
ax = fig.add_subplot(111)
ax.grid(True)
ax.set_title("Realtime Velocity in x Plot")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Velocity (m/s)")
ax.axis([0,100,-3.5,3.5])

xAchse=pylab.arange(0,100,1)
yAchse=pylab.array([0]*100)
line1 = ax.plot(xAchse, yAchse, '-')
line2 = ax.plot(xAchse, yAchse, 'g--')

values = [0 for x in range(100)]
estimations = [0 for x in range(100)]

manager = pylab.get_current_fig_manager()

message_recivied = False

#------------------KALMAN PARAMETERS----------------------------
#State vector
xk_1 = 0.0

#Covariance Matrix
Pk_1 = 1.0

#Process noise Matrix 
Qk = 1e-5

#Error in Measurement
R = 0.0001

#Observation error
e = 0.0

#Measurement
zk_0 = 0.0
zk_1 = 0.0

def real_time_plotter(arg):
  global values
  CurrentXAxis = pylab.arange(len(values)-100,len(values),1)
  line1[0].set_data(CurrentXAxis,pylab.array(values[-100:]))
  line2[0].set_data(CurrentXAxis,pylab.array(estimations[-100:]))

  ax.axis([CurrentXAxis.min(),CurrentXAxis.max(),-3.5,3.5])
  manager.canvas.draw()

def ploting_velocity(arg):
  global values, zk_0, zk_1
  values.append(zk_1)
  
def ploting_estimation(arg):
  global estimations, xk_
  estimations.append(xk_)

def callback(data):
    global zk_0, zk_1, xk_1, Pk_1, xk_
    zk_0 = data.data[0]
    zk_1 = data.data[2]

    #print("zk_0", zk_0)
    #print("zk_1", zk_1)

    #------Kalman process--------
    #Prediction
    xk_ = xk_1 
    Pk_ = Pk_1 + Qk

    #Update
    Kk = Pk_ / (Pk_ + R)
    xk = xk_ + Kk *(zk_1 - xk_)
    Pk = (1 - Kk)*Pk_
    
    print('xk_1',xk_1)
    print('xk_', xk_)  
    #print('Pk_',Pk_)
    #print('Kk', Kk)
   
    print('xk_', xk_)
    xk_1 = xk
    Pk_1 = Pk

def graph_velocity_x():
    rospy.init_node('velocity_x_node', anonymous=True)
    rospy.Subscriber("/vision/ball_kinematics/ball_kinematics", Float32MultiArray, callback)
        
    timer = fig.canvas.new_timer(interval=30)
    timer.add_callback(real_time_plotter, ())
    timer.start()
    timer2 = fig.canvas.new_timer(interval=30)
    timer2.add_callback(ploting_velocity, ())
    timer2.start()
    timer3 = fig.canvas.new_timer(interval=30)
    timer3.add_callback(ploting_estimation, ())
    timer3.start()    
    pylab.show()



if __name__ == '__main__':
    graph_velocity_x()

 
