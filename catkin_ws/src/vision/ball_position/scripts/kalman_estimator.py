#!/usr/bin/env python
import os
import json
import rospy
import numpy
import rospkg

from random import gauss
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray


#VARIABLES FISICAS
g  = 9.81	   #ACELERACION DE LA GRAVEDAD
dt = 0.033333  #TIEMPO DE MUESTREO
mu_d = 0.15    #COEFICIENTE DE FRICCION DINAMICA

#CONTADOR DE MUESTRAS
measurements = 0

#NUMERO MINIMO DE MUESTRAS PARA EMPEZAR LA ESTIMACION 
data_samples = 8

#MUESTRAS TOTALES
x_positions = []
y_positions = []

#BANDERA PARA DEJAR DE REGISTRAR DATOS
log_out = False

#TIEMPO ESTIMADO PARA LLEGAR A LA META 
time_to_kick = 0

#POSICION DE LATENCIA DEL PIE IZQUIERDO
foot_pose = 0.05

#TIEMPO DE TRAYECTORIA DE PATEO
movement_time = 0.1

#RANGO DE PATEO EN EL EJE X
range_in_x = 0.39

#PUBLICADOR PARA PATEAR EL BALON
kick = rospy.Publisher('/robot_stop', Bool, queue_size=1000)

#POSICION INICIAL
print "------------- ESTADO INICIAL ---------------"
print "Numero de muestras por registar: ", data_samples

Xn = numpy.array([[-1.2],
				  [-0.8],
				  [ 3.5],
				  [ 3.5]])
print "X0"
print  Xn

#MATRIZ DE COVARIANZA DEL PROCESO
Pn  = 5 * numpy.identity(4)

print "P0"
print  Pn

#MATRIZ DE PREDICCION
F1 = [ 1, 0, dt,  0]
F2 = [ 0, 1,  0, dt]	
F3 = [ 0, 0,  1,  0]
F4 = [ 0, 0,  0,  1]

F = numpy.array([F1, F2, F3, F4])


#MATRIZ DE COVARIANZA DEL RUIDO DE LA ESTIMACION
Q  = 0.000002 * numpy.identity(4)

print "Q"
print  Q

#MATRIZ DE OBSERVACION
H1 = [ 1, 0, 0, 0]
H2 = [ 0, 1, 0, 0]


H = numpy.array([H1, H2])

print "H"
print  H

#MATRIZ DE COVARIANZA DE RUIDO EN LA MEDICION
R = 0.00002 * numpy.identity(2)

print "Rn"
print  R

#FUERZA DE FRICCION
Fr = numpy.array([[     0    ],
				  [     0    ],
				  [-dt*mu_d*g],
				  [-dt*mu_d*g]])

#print "Fr"
#print  Fr

#FUNCION DE PREDICCION ESTADO SIGUIENTE
def prediction_state():
	global Xn_, Pn_, Xn1
	#print ""
	#print "------------- ESTADO DE PREDICCION --------------"

	#PREDICCION DEL ESTADO SIGUIENTE
	Xn1 = numpy.dot(F, Xn) + Fr  
	#print "Xn1"
	#print  Xn1

	#PREDICCION DE LA COVARIANZA
	Pn1 = numpy.dot(numpy.dot(F, Pn), F.transpose()) + Q
	#Pn1 = numpy.diag(numpy.diag(Pn1)) + Q
	#print "Pn1"
	#print  Pn1

	# n -> n+1
	Xn_ = Xn1
	Pn_ = Pn1

#FUNCION PARA GUARDAR EL ESTADO DEL BALON
def catch_x_position(Rx, Z1, x_, xn):
	position_list = list()
	position_list.append(round(Rx,3)) #VALOR EXACTO
	position_list.append(round(Z1,3)) #VALOR MEDIDO
	position_list.append(round(x_,3)) #PREDICCION
	position_list.append(round(xn,3)) #VALOR CORREGIDO

	x_positions.append(position_list)

def catch_y_position(Ry, Z2, y_, yn):
	position_list = list()
	position_list.append(round(Ry,3)) #VALOR EXACTO
	position_list.append(round(Z2,3)) #VALOR MEDIDO
	position_list.append(round(y_,3)) #PREDICCION
	position_list.append(round(yn,3)) #VALOR CORREGIDO

	y_positions.append(position_list)

#FUNCION DE PATEO
def wait_for_kick():
	rate = rospy.Rate(30)
	
	#CONTADOR DE TIEMPO
	current_time = 0
	while current_time < time_to_kick - movement_time:
		current_time += dt
		rate.sleep()

	print "Pateando balon..."
	kick.publish(Bool(True))


#FUNCION QUE CALCULA EL TIEMPO RESTANTE PARA EL PATEO
def estimator():
	global Xn, Xn1, time_to_kick
	
	print "\nMuestreo completado."


	if float(Xn[1]) < 0:
		print "Distancia restante:", 100 * round(0 - float(Xn[1]), 3) , "cm"
		print "Estimando posiciones..."
		while float(Xn[1]) < foot_pose:
			prediction_state()
			Xn = Xn1
			time_to_kick += dt
			#print "Xn[1]:", round(float(Xn[1]),3), "\tXn[3]:", round(float(Xn[3]),3)

			if Xn[3] < 0:
				print "\nEl balon no tiene suficiente impulso"
				break
		
		#CONDICIONAL CUANDO EL BALON LLEGA A LA META
		if Xn[1] >= foot_pose:
			if time_to_kick < movement_time: #UMBRAL DE TIEMPO DE PATEO
				print "\nTiempo insuficiente para patear."
			else:		
				print "\nTiempo estimado para patear:", time_to_kick
				wait_for_kick()
				'''if(Xn[0] <= range_in_x): #SI EL BALON ESTA DENTRO DE RANGO EN X
					print "\nTiempo estimado para patear:", time_to_kick
					wait_for_kick()
				
				else: print "\nBalon fuera de rango en X"'''

	else: print "\nEl balon esta fuera de rango en Y"

#FUNCION DE RESPUESTA AL RECIBIR LOS DATOS DE ENTRADA
def measurement_input(data):
	global Xn, Pn, measurements, log_out
	measurements += 1
	#print ""
	#print "------------- VALORES DE ENTRADA --------------"
	#print "Medicion numero:", measurements

	#VALOR EXACTO DE POSICION
	Rx = data.data[0]
	Ry = data.data[1]

	#VECTOR DE MEDICION
	Z1 = data.data[2]
	Z2 = data.data[3]

	Z = numpy.array([[Z1],
				     [Z2]])
	#print "Z"
	#print  Z

	if measurements - 1 < data_samples:
		#print ""
		#print "------------- ESTADO DE CORRECCION --------------"
		

		#CALCULO DE LA GANANCIA DE KALMAN
		K1 = numpy.dot(Pn_, H.transpose())
		K2 = numpy.dot(numpy.dot(H,Pn_), H.transpose()) + R 


		K = numpy.dot(K1, numpy.linalg.inv(K2))

		#print "K"
		#print  K

		#CORRECCION DE LA POSICION
		Xn = Xn_ + numpy.dot(K, Z - numpy.dot(H, Xn_))

		#print "Xn"
		#print  Xn


		#CORRECCION DE LA COVARIANZA DEL PROCESO
		Pn = numpy.dot(numpy.identity(4)- numpy.dot(K, H), Pn_)
		
		#print "Pn"
		#print  Pn

		catch_x_position(Rx, Z1, float(Xn_[0]), float(Xn[1]))
		catch_y_position(Ry, Z2, float(Xn_[1]), float(Xn[1]))
		prediction_state()

	if measurements == data_samples:
		log_out = True

	
#FUNCION PRINCIPAL
def kalman_estimator():
	#INICIALIZA EL NODO
	rospy.init_node('kalman_estimator', anonymous=True)
	print ""
	print "Inicializando nodo kalman_estimator por Luis Nava"

	#TOPICO AL CUAL SE SUBSCRIBE PARA OBTENER LOS DATOS DE POSICION
	rospy.Subscriber("/vision/ball_position/ball_position", Float32MultiArray, measurement_input)

	#METODOS PARA GUARDAR INFORMACION EN UN ARCHIVO DE TEXTO
	rospack = rospkg.RosPack()
	rospack.list()

	abs_path = rospack.get_path('ball_position')

	if os.path.exists(abs_path + '/scripts/kalman_data_x.txt'):
		os.remove(abs_path + '/scripts/kalman_data_x.txt')	
	
	if os.path.exists(abs_path + '/scripts/kalman_data_y.txt'):
		os.remove(abs_path + '/scripts/kalman_data_y.txt')

	print "Esperando datos de entrada..."

	#BUCLE PARA LA RECEPCION DE MENSAJES
	while not rospy.core.is_shutdown():
		if log_out: break

	#FUNCION PARA ESTIMAR EL TIEMPO PARA PATEAR
	estimator()

	print "\nFinalizando nodo..."

	#SALVANDO POSICIONES EN UN ARCHIVO DE TEXTO
	with open(abs_path + '/scripts/kalman_data_x.txt', 'a') as filehandle:
		for i in range(data_samples):
			json.dump(x_positions[i], filehandle)
			filehandle.write("\n")

	with open(abs_path + '/scripts/kalman_data_y.txt', 'a') as filehandle:
		for i in range(data_samples):
			json.dump(y_positions[i], filehandle)
			filehandle.write("\n")

	print data_samples, "Muestras almacenadas en kalman_data_x.txt y kalman_data_y.txt"

if __name__ == '__main__':
	prediction_state()
	kalman_estimator()