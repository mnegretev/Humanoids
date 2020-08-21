#!/usr/bin/env python
import os
import json
import rospy
import numpy
import rospkg


from std_msgs.msg import Float32MultiArray


abs_path = ""
all_positions = []
number_positions  = 0
first_measurement = True

g  = 9.81
dt = 0.0333
mu_d = 0.3636

#Prediction parameters
print "-------------Initial state---------------"
Xn = numpy.array([[0.0], 
				  [0.0], 
				  [0.0], 
				  [0.0]])
print "Xn"
print  Xn

#Covariance estimation matrix
Pn  = numpy.array([[1.0],
				   [1.0],
				   [1.0],
				   [1.0]])
print "Pn"
print  Pn

#Extrapolation matrix
F1 = [ 1, 0, dt,  0]
F2 = [ 0, 1,  0, dt]
F3 = [ 0, 0,  1,  0]
F4 = [ 0, 0,  0,  1]

F = numpy.array([F1, F2, F3, F4])
print "F" 
print  F

#Model noise
Q = numpy.zeros((4,4))
Q[0,0] = 0.001
Q[1,1] = 0.001
Q[2,2] = 0.001
Q[3,3] = 0.001
print "Q"
print  Q

#Process noise
w = numpy.array([[0.0],
	             [0.0],
	             [0.0],
	             [0.0]])


#Measurement noise
Rn = numpy.array([[0.01],
	              [0.01],
	              [0.01],
	              [0.01]])

#Measurement vector
Z = numpy.array([[0.0], 
				 [0.0],
				 [0.0],
				 [0.0]])


def prediction_state():
	global Xn, Xn1, Xn_, Pn, Pn1, Pn_
	print "-------------Prediction state--------------"
	Xn1 = numpy.array([Xn[0] +  Xn[2]*dt + w[0],
			    	   Xn[1] +  Xn[3]*dt + w[1],
					   Xn[2] - mu_d*g*dt + w[2],
					   Xn[3] - mu_d*g*dt + w[3]])

	print "Xn1"
	print  Xn1


	Pn1 = numpy.dot(F, Pn) 

	print "Pn1"
	print  Pn1

	# n-> n+1
	Xn_ = Xn1
 	Pn_ = Pn1

 	print "Xn_"
 	print  Xn_
 	print "Pn_"
 	print  Pn_


def estimation_state():
	global Kn, Xn, Xn_, Xn1, Pn, Pn_, Z

	print "----------- Estimation State -------------"

	print "Xn"
	print  Xn
	print "Xn_"
	print  Xn_
	print "Pn_"
	print  Pn_
	print "Z"
	print  Z

	K0 = Pn_[0] / ( Pn_[0] + Rn[0] )
	K1 = Pn_[1] / ( Pn_[1] + Rn[1] )
	K2 = Pn_[2] / ( Pn_[2] + Rn[2] )
	K3 = Pn_[3] / ( Pn_[3] + Rn[3] )

	#Kalman gain
	Kn = numpy.array([K0,
		              K1,
		              K2,
		              K3])

	print "Kn"
	print  Kn

	Xn[0] = Xn_[0] + Kn[0] * (Z[0] - Xn_[0])
	Xn[1] = Xn_[1] + Kn[1] * (Z[1] - Xn_[1])
	Xn[2] = Xn_[2] + Kn[2] * (1)
	Xn[3] = Xn_[3] + Kn[2] * (1)

	Pn[0] = (1 - Kn[0]) * Pn_[0]
	Pn[1] = (1 - Kn[1]) * Pn_[1]
	Pn[2] = (1 - Kn[2]) * Pn_[2]
	Pn[3] = (1 - Kn[3]) * Pn_[3]

	#print "Pn1"
	#print  Pn1
	#print "Rn"
	#print  Rn
	print "Xn"
	print  Xn
	print "Pn"
	print  Pn

	prediction_state();

def measurement_input(data): 
	global first_measurement, number_positions, Z

	#position_list = list(data.data)
	#all_positions.append(position_list)

	#number_positions += 1


	Z[0] = data.data[2]
	Z[1] = data.data[3]
	Z[2] = (data.data[2] - Z[2]) / dt  
	Z[3] = (data.data[3] - Z[3]) / dt

	position_list = list(data.data)
	#position_list.append(float(Z[2]))
	#position_list.append(float(Z[3]))
	position_list.append(float(Xn1[0]))
	position_list.append(float(Xn1[1]))
	position_list.append(float(Xn[0]))
	position_list.append(float(Xn[1]))
	'''print "Z"
	print  Z'''

	if not first_measurement:
		estimation_state()
		
		all_positions.append(position_list)
		number_positions += 1

	first_measurement = False

def save_ball_positions():
	global number_positions, all_positions
	with open(abs_path + '/scripts/kalman_data.txt', 'a') as filehandle:
		for i in range(number_positions):
			json.dump(all_positions[i], filehandle)
			filehandle.write("\n")

	print "------>", number_positions, " positions saved :)" 


def kalman_processing_node():
	global abs_path, all_positions
	rospy.init_node('kalman_processing_node', anonymous=True)
	print "Starting kalman_processing_node by Luis Nava"
	rospy.Subscriber("/vision/ball_position/ball_position", Float32MultiArray, measurement_input)

	rospack = rospkg.RosPack()
	rospack.list()

	abs_path = rospack.get_path('ball_position')
	if os.path.exists(abs_path + '/scripts/kalman_data.txt'):
		os.remove(abs_path + '/scripts/kalman_data.txt')

	print "Loading positions..."
	rospy.spin()

	save_ball_positions()
	
if __name__ == '__main__':
	prediction_state()
	kalman_processing_node()