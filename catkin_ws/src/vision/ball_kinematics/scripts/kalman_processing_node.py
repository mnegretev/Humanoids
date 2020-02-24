#!/usr/bin/env python
import os
import json
import rospy
import numpy
import rospkg

from std_msgs.msg import Float32MultiArray

x_ = 0
y_ = 0
t0 = 0
first_sample = True
abs_path = ""

def initialMatrixes():
	global F, G, u , Xn, Pn, Q, H, R
	print ""
	print "------------------Initializing Parameters--------------------"
	dt = 0.05
	mu = 0.1
	g  = 9.81

	#Extrapolation matrix
	F1 = [ 1, 0, dt,  0]
	F2 = [ 0, 1,  0, dt]
	F3 = [ 0, 0,  1,  0]
	F4 = [ 0, 0,  0,  1]

	F = numpy.array([F1, F2, F3, F4])
	print "F" 
	#print  F

	#Control matrix
	G1 = [ 0.5*dt**2,     0    ]
	G2 = [     0    , 0.5*dt**2]
	G3 = [	   dt   ,     0    ]
	G4 = [	   0    ,     dt   ]

	G = numpy.array([G1, G2, G3, G4])
	print "G"
	#print  G

	#Control input
	u1 = [-mu*g]
	u2 = [-mu*g]

	u = numpy.array([u1, u2])
	print "u"
	#print  u

	#Initial state
	Xn = numpy.array([[0], [0], [0], [0]])
	print "Xn"
	#print  Xn

	#Covariance estimation matrix
	Pn = numpy.identity(4)
	print "Pn"
	#print  Pn

	#Observation matrix
	H = numpy.identity(4)
	print "H"
	#print  H

	#Process error matrix
	Q = numpy.zeros((4,4))
	print "Q"
	#print  Q

	#Measurement error
	R1 = [0.1,   0,   0,   0]
	R2 = [  0, 0.1,   0,   0]
	R3 = [  0,   0, 0.1,   0]
	R4 = [  0,   0,   0, 0.1]
	R  = numpy.array([R1, R2, R3, R4])

def extrapolationState():
	global Xn, Xn1, Pn, Pn1
	print ""
	print "---Extrapolation State---"
	#Extrapolation state
	Xn1 = numpy.dot(F, Xn) + numpy.dot(G, u)
	#Extrapolation estimate covariance

	Pn1 = numpy.dot(F, numpy.dot(Pn, F.transpose())) + Q
	print "Xn"
	print Xn
	print "Xn1"
	print Xn1
	print "Pn"
	print Pn
	print "Pn1"
	print Pn1

def estimationState():
	global Xn, Xn_, Pn
	print ""
	print "---Estimation State---"

	#Changing state
	Pn_ = Pn1
	print "Pn_"
	print  Pn_

	#Kalman gain
	HT = H.transpose()
	
	K1 = numpy.dot(Pn_, HT)
	print "K1"
	#print  K1

	K2 = numpy.dot(H, numpy.dot(Pn_, H)) + R
	print "K2"
	#print  K2

	K  = numpy.dot(K1, numpy.linalg.inv(K2))
	print "K"
	print  K

	#Updating state matrix
	Xn_ = Xn1
	Xn = Xn_ +  numpy.dot(K, Z - numpy.dot(H, Xn_))

	print "Xn_"
	print  Xn_
	print "Xn"
	print  Xn

	#Updating covariance matrix
	Pn = numpy.dot(numpy.identity(4) - numpy.dot(K, H), Pn_)
	print "Pn"
	print  Pn

	extrapolationState();

def measurementInput(data): 
	global abs_path, first_sample, x_, y_, t0, Z, Xn_
	print ""
	print "-----------------------Measurement State-------------------"
	ti = rospy.get_time()
	if first_sample :
		t0 = ti
		x_ = data.data[0]
		y_ = data.data[1]
		first_sample = False 

	dx = data.data[0] - x_;
	dy = data.data[1] - y_;
	dt = ti - t0


	positions_list = list(data.data)
	positions_list.append(dt)

	
	x_ = data.data[0]
	y_ = data.data[1]
	t0 = ti

	#Measurement matrix
	Z = numpy.array([[x_], [y_], [dx], [dy]])
	print "Z"
	print  Z
	estimationState()

	positions_list.append(float(Xn[0]))
	positions_list.append(float(Xn[1]))
	positions_list.append(float(Xn_[0]))
	positions_list.append(float(Xn_[1]))

	with open(abs_path + '/scripts/kalman_data.txt', 'a') as filehandle:
		json.dump(positions_list, filehandle)
		filehandle.write("\n")

def kalman_processing_node():
	global abs_path
	rospy.init_node('kalman_processing_node', anonymous=True)
	rospy.Subscriber("/vision/ball_kinematics/ball_kinematics", Float32MultiArray, measurementInput)

	rospack = rospkg.RosPack()
	rospack.list()

	abs_path = rospack.get_path('ball_kinematics')
	if os.path.exists(abs_path + '/scripts/kalman_data.txt'):
		os.remove(abs_path + '/scripts/kalman_data.txt')

	rospy.spin()

if __name__ == '__main__':
	initialMatrixes()
	extrapolationState()
	kalman_processing_node()
