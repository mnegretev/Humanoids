import math
import sys
import rospy
import numpy
import tf
import tf.transformations as tft
import urdf_parser_py.urdf


def get_model_info(joint_names):
    robot_model = urdf_parser_py.urdf.URDF.from_parameter_server()
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

def main():
    print("INITIALIZING INVERSE KINEMATIC NODE - ")
    rospy.init_node("ik_geometric")
    loop = rospy.Rate(40)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()