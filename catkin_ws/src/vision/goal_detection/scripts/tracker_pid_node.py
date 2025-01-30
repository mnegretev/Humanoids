#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32MultiArray
import time
pan_robot=0.0
tilt_robot=0.0
class PIDController :
	def __init__(self, Kp, Ki, Kd):
		self.Kp= Kp
		self.Ki= Ki
		self.Kd= Kd
		self.prev_error = 0
		self.integral = 0
	def execute (self, error, dt):
		self.integral += error * dt
		derivative = (error - self.prev_error)
		self.prev_error = error
		
		return self.Kp * error +self.Ki *self.integral + self.Kd * derivative
	
pid_pan = PIDController (Kp = 0.05/320, Ki = 0.00005/320, Kd = 0.02/320)
pid_tilt = PIDController (Kp = 0.025/240, Ki = 0.000025/240, Kd = 0.01/240)
last_time = time.time()

def head_k_generator(centroid_msg):
    global pan_robot, tilt_robot, last_time
	
    center_x = centroid_msg.x
    center_y = centroid_msg.y
    print(f"Centroid received {center_x}, {center_y}")
    head_cmd = Float32MultiArray()
    # Error
    Cx = 320
    Cy = 240
    ex = center_x-Cx
    ey = center_y-Cy
    
    current_time = time.time()
    dt = (current_time - last_time)
    last_time = current_time
    
    pan_correction = pid_pan.execute (-ex, dt)
    tilt_correction = pid_tilt.execute(ey, dt)
	
    pan_robot  += pan_correction
    tilt_robot += tilt_correction

    head_cmd.data = [pan_robot, tilt_robot]
    print(f"Publishing pan_angle: {pan_robot}, tilt: {tilt_robot}")
    pub_head_goal.publish(head_cmd)

def main ():
    rospy.init_node("tracker_pid_node", anonymous=True)

    global pub_head_goal,rate, head_move_pub, searching
    pub_head_goal = rospy.Publisher("/hardware/head_goal_pose", Float32MultiArray, queue_size=1)      
    rospy.Subscriber("/centroid_publisher", Point32, head_k_generator)  
    rospy.spin()

if __name__=="__main__":
    main()
