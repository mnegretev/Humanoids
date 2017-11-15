import numpy as np

class Tz():
	def Tras_x(self,x):
		Tx = np.matrix([[1,0,0,x],
			                   [0,1,0,0],
			                   [0,0,1,0],
			                   [0,0,0,1]])
		return Tx

	def Tras_y(self,y):
	    Ty = np.matrix([[1,0,0,0],
	                   [0,1,0,y],
	                   [0,0,1,0],
	                   [0,0,0,1]])
	    return Ty

	def Tras_z(self,z):
	    Tz = np.matrix([[1,0,0,0],
	                   [0,1,0,0],
	                   [0,0,1,z],
	                   [0,0,0,1]])
	    return Tz

	def T_Roll(self,theta_x):
	    T_th_x = np.matrix([[ 1 ,      0          ,        0         ,   0],
	                         [0 , np.cos(theta_x) , -np.sin(theta_x) ,   0],
	                         [0 , np.sin(theta_x) , np.cos(theta_x)  ,   0],
	                         [0 ,      0          ,        0         ,   1]])
	    return T_th_x

	def T_Pitch(self,theta_y):
	    T_th_y = np.matrix([[ np.cos(theta_y)  ,      0    ,  np.sin(theta_y)   ,   0],
	                        [       0          ,      1    ,         0          ,   0],
	                        [-np.sin(theta_y)  ,      0    ,  np.cos(theta_y)   ,   0],
	                        [       0          ,      0    ,         0          ,   1]])
	    return T_th_y

	def T_Yaw(self,theta_z):
	    T_th_z = np.matrix([[ np.cos(theta_z)  ,   -np.sin(theta_z) ,   0  ,   0],
	                        [ np.sin(theta_z)  ,    np.cos(theta_z) ,   0  ,   0],
	                        [        0         ,           0        ,   1  ,   0],
	                        [        0         ,           0        ,   0  ,   1]])
	    return T_th_z