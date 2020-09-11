#!/usr/bin/env python
import os
import json
import rospy
import numpy
import rospkg

from random import gauss
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

flag_once = False
curr_time = 0
time_2_kick = 0
data_samples = 7
next_position = 0
distance_remaining = 0

time_movement = 0.15

abs_path = ""
all_positions = []
number_positions  = 0
first_measurement = True

g  = 9.81
dt = 0.0333
mu_d = 0.15

counter = 1

pub = rospy.Publisher('/robot_stop', Bool, queue_size=1000)

#Prediction parameters
print "-------------Initial state---------------"
Xn = numpy.array([[-0.3], 
				  [ 0.3]])
print "Xn"
print  Xn

#Covariance estimation matrix
Pn  = numpy.array([[0.1],
				   [0.1]])
print "Pn"
print  Pn

#Model noise
#Q = numpy.zeros((4,4))
Q = numpy.array([[0.000001],
				 [0.0]])

print "Q"
print  Q

#Process noise
w = numpy.array([[0.015],
	             [0.0]])


#Measurement noise
Rn = numpy.array([[0.00001],
	              [0.00001]])

print "Rn"
print  Rn

#Measurement vector
Z = numpy.array([[0.0], 
				 [0.0]])


def estimator():
	global Xn

	Xn = numpy.array([Xn[0] +  Xn[1]*dt + w[0],
					   Xn[1] - mu_d*g*dt + w[1]])


def prediction_state():
	global Xn, Xn1, Xn_, Pn, Pn1, Pn_
	'''print "-------------Prediction state--------------"'''

	Xn1 = numpy.array([Xn[0] +  Xn[1]*dt + w[0],
					   Xn[1] - mu_d*g*dt + w[1]])



	Pn1 = numpy.array([Pn[0] + dt**2 * Pn[1]**2 + Q[0],
		                       Pn[1]            + Q[1]])


	# n-> n+1
	Xn_ = Xn1
 	Pn_ = Pn1



def correction_state():
	global Kn, Xn, Xn_, Xn1, Pn, Pn_, Z, counter

	print ""
	print "----------- Correction State: ",counter,"-------------"
	counter += 1

	K0 = Pn_[0] / ( Pn_[0] + Rn[0] )
	K1 = Pn_[1] / ( Pn_[1] + Rn[1] )

	#Kalman gain
	Kn = numpy.array([K0,
		              K1])



	Xn[0] = Xn_[0] + Kn[0] * (Z[0] - Xn_[0])
	Xn[1] = Xn_[1] + Kn[1] * (Z[1] - Xn_[1])

	Pn[0] = (1 - Kn[0]) * Pn_[0]
	Pn[1] = (1 - Kn[1]) * Pn_[1]

	print "Xn[0]: ", float(Xn[0])
	#print "Xn[1]: ", float(Xn[1])
	
	'''print "Z"
	print  Z
	print "Xn_"
	print  Xn_
	print "Pn_"
	print  Pn_

	print "Kn"
	print  Kn
	
	print "Xn"
	print  Xn
	
	print "Pn"
	print  Pn'''


def measurement_input(data): 
	global first_measurement, number_positions, Z, time_2_kick, curr_time, next_position, distance_remaining, flag_once


	Z[1] = (data.data[3] - Z[0]) / dt
	Z[0] =  data.data[3] + gauss(0, 0.025)
  
	position_list = list()


	if(number_positions < data_samples):
		if not first_measurement:
			correction_state()

			position_list.append(data.data[1])
			position_list.append(float(Z[0]))
			position_list.append(float(Xn_[0]))
			position_list.append(float(Xn[0]))

			all_positions.append(position_list)
			number_positions += 1

			next_position = float(Xn[0]) -1
			curr_time = 0
			prediction_state();


	else:
		while(next_position < float(Xn[0])):
			next_position = float(Xn[0])
			if next_position < 0.1 :
				print "extrapolation: ", next_position
				time_2_kick += dt

			estimator()

		if flag_once == False:
			print ""
			print "time_2_kick: ", time_2_kick
			flag_once = True

		if curr_time > time_2_kick - time_movement:
			pub.publish(Bool(True))

		curr_time += dt

	first_measurement = False


def save_ball_positions():
	global number_positions, all_positions, time_2_kick
	with open(abs_path + '/scripts/kalman_estimator_data.txt', 'a') as filehandle:
		for i in range(number_positions):
			json.dump(all_positions[i], filehandle)
			filehandle.write("\n")

	print "------>", number_positions, " positions saved :)" 


def kalman_estimator():
	global abs_path, all_positions
	rospy.init_node('kalman_estimator', anonymous=True)
	print "Starting kalman_estimator by Luis Nava"
	rospy.Subscriber("/vision/ball_position/ball_position", Float32MultiArray, measurement_input)

	rospack = rospkg.RosPack()
	rospack.list()

	abs_path = rospack.get_path('ball_position')
	if os.path.exists(abs_path + '/scripts/kalman_estimator_data.txt'):
		os.remove(abs_path + '/scripts/kalman_estimator_data.txt')

	print "Loading positions..."
	rospy.spin()

	save_ball_positions()
	
if __name__ == '__main__':
	prediction_state()
	kalman_estimator()
