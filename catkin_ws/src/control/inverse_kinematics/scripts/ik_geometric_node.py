#!/usr/bin/env python3
import math
import sys
import csv
import rospy
import numpy
import tf
import tf.transformations as tft
import urdf_parser_py.urdf
from std_msgs.msg import Float64MultiArray
from manip_msgs.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ctrl_msgs.srv import CalculateIK, CalculateIKResponse 

prompt = ""
NAME = "Frías Hernández Camille Emille Román"
def forward_kinematics(q, T, W):
    x,y,z,R,P,Y = 0,0,0,0,0,0
    H = tft.identity_matrix()
    for i in range(len(q)):
        Ri = tft.rotation_matrix(q[i], W[i])
        H = tft.concatenate_matrices(H,T[i], Ri)
    H = tft.concatenate_matrices(H, T[i])
    x, y, z = H[0,3], H[1,3], H[2,3]
    R, P, Y = list(tft.euler_from_matrix(H))
    return numpy.asarray([x,y,z,R,P,Y])

def jacobian(q, T, W):
    delta_q = 0.000001
    J = numpy.asarray([[0.0 for a in q] for i in range(6)])
    qn = numpy.asarray([q,] * len(q)) + delta_q * numpy.identity(len(q))
    qp = numpy.asarray([q,] * len(q)) - delta_q * numpy.identity(len(q))
    for i in range(len(q)):
        J[:,i] = (forward_kinematics(qn[i],T, W) - forward_kinematics(qp[i], T,W)) / (delta_q * 2.0)
    
    return J

def inverse_kinematics(x, y, z, roll, pitch, yaw, T, W, init_guess=numpy.zeros(6), max_iter=200):
    pd = numpy.asarray([x,y,z,roll,pitch,yaw])
    global total_iterations
    q = init_guess
    iterations = 1
    tol = 0.001
    p = forward_kinematics(q,T,W)
    e = p - pd
    e[3:6] = (e[3:6] + math.pi)%(2*math.pi) - math.pi
    while numpy.linalg.norm(e) > tol and iterations < max_iter:
        J = jacobian(q, T, W)
        q = (q - numpy.dot(numpy.linalg.pinv(J),e) + math.pi)%(2*math.pi) - math.pi
        p = forward_kinematics(q, T, W)
        e = p - pd
        e[3:6] = (e[3:6] + math.pi)%(2*math.pi) - math.pi
        iterations += 1
        total_iterations+=1
    success = iterations < max_iter and angles_in_joint_limits(q)
    return success, q

def get_model_info(joint_names):
    robot_model = urdf_parser_py.urdf.URDF.from_parameter_server('/robot_description')
    joints = []
    transforms = []
    for name in joint_names:
        for joint in robot_model.joints:
            if joint.name == name:
                joints.append(joint)
    for joint in joints:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms.append(tft.concatenate_matrices(T,R))
    return joints, transforms

def angles_in_joint_limits(q):
    for i in range(len(q)):
        if q[i] < joints[i].limit.lower or q[i] > joints[i].limit.upper:
            print(prompt+"Articular position out of joint bounds")
            return False
    return True

        
def callback_ik_leg_left(req):
    global max_iterations, joints, transforms
    joints, transforms = get_model_info(left_leg)
    if not (len(joints) > 5 and len(transforms) > 5):
        print("Inverse kinematics.->Cannot get model info from parameter server")
        sys.exit(-1)

    [x,y,z,R,P,Y] = [req.x,req.y,req.z,req.roll,req.pitch,req.yaw]
    print(prompt+"Calculating inverse kinematics for pose: " + str([x,y,z,R,P,Y]))
    data = [0.000000000720284443, -0.080730319, -0.100167155, 0.261276811, -0.171109661, 0.080730319]
    init_guess = numpy.array(data, dtype=numpy.float32) 
    resp = CalculateIKResponse()

    W = [joints[i].axis for i in range(len(joints))]  
    success, q = inverse_kinematics(x, y, z, R, P, Y,transforms, W, init_guess, max_iterations) 
    if not success:
        return False
    resp.joint_values = q
    return resp        
            
def callback_ik_leg_right(req):
    global max_iterations, joints, transforms
    joints, transforms = get_model_info(right_leg)
    if not (len(joints) > 5 and len(transforms) > 5):
        print("Inverse kinematics.->Cannot get model info from parameter server")
        sys.exit(-1)

    [x,y,z,R,P,Y] = [req.x,req.y,req.z,req.roll,req.pitch,req.yaw]
    print(prompt+"Calculating inverse kinematics for pose: " + str([x,y,z,R,P,Y]))
    data = [0.000000000720284443, -0.080730319, -0.100167155, 0.261276811, -0.171109661, 0.080730319]
    init_guess = numpy.array(data, dtype=numpy.float32) 
    resp = CalculateIKResponse()

    W = [joints[i].axis for i in range(len(joints))]  
    success, q = inverse_kinematics(x, y, z, R, P, Y,transforms, W, init_guess, max_iterations) 
    if not success:
        return False
    resp.joint_values = q
    return resp   

def main():
    global  max_iterations, joints, transforms, prompt, left_leg, right_leg, total_iterations
    total_iterations=0 
    left_leg = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee_pitch", "left_ankle_pitch", "left_ankle_roll"]
    right_leg = ["right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee_pitch", "right_ankle_pitch", "right_ankle_roll"]
    print("INITIALIZING INVERSE KINEMATIC NODE - " + NAME)
    rospy.init_node("ik_geometric")
    prompt = rospy.get_name().upper() + ".->"
    max_iterations = rospy.get_param("~max_iterations", 400)
    print(prompt+"max_iterations: " + str(max_iterations))

    rospy.Service("/manipulation/ik_leg_left_pose"              , CalculateIK, callback_ik_leg_left) 
    rospy.Service("/manipulation/ik_leg_right_pose"              , CalculateIK, callback_ik_leg_right) 
    loop = rospy.Rate(40)
    while not rospy.is_shutdown():
        loop.sleep()
if __name__ == '__main__':
    main()
