#!/usr/bin/env python3
import rospy
import numpy as np
from ctrl_msgs.srv import CalculateDK, CalculateDKRequest
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def get_cartesian_positions(joint_trajectories):
    rospy.wait_for_service('/manipulation/fk_leg_right_pose')
    rospy.wait_for_service('/manipulation/fk_leg_left_pose')
    if side == "right":
        fk_service = rospy.ServiceProxy('/manipulation/fk_leg_right_pose', CalculateDK)
    else:
        fk_service = rospy.ServiceProxy('/manipulation/fk_leg_left_pose', CalculateDK)
    cartesian_positions = []

    for i, angles in enumerate(joint_trajectories):
        try:
            req = CalculateDKRequest()
            req.joint_values = angles
            resp = fk_service(req)
            cartesian_positions.append([resp.x, resp.y, resp.z])
        except rospy.ServiceException as e:
            rospy.logerr(f"Error calling FK service for sample {i}: {e}")
            cartesian_positions.append([None, None, None])
    
    return np.array(cartesian_positions)

def plot_trajectory(positions):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs, ys, zs = positions[:, 0], positions[:, 1], positions[:, 2]

    ax.plot(xs, ys, zs, label='Trayectoria del efector final')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Trayectoria 3D del efector final')
    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    global side
    rospy.init_node("traviz")
    side = rospy.get_param("~side", "right")
    npz_file = rospy.get_param("~NPZ")
    data = np.load(npz_file)
    leg_joint_angles = data[side]

    print("Calculando posiciones cartesianas...")
    positions = get_cartesian_positions(leg_joint_angles)

    print("Graficando...")
    plot_trajectory(positions)
