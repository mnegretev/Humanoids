import rospy

from geometry_msgs.msg import Pose, TransformStamped, PointStamped, Point, Quaternion
from tf2_geometry_msgs import do_transform_point
from vision_msgs.msg import VisionObject
from vision_msgs.srv import ProcessObject
#from tf2_ros import Tras
#from tf import TransformerROS
import tf2_ros
import tf.transformations as tft
import numpy as np

PI = 3.14159265
HFOV = 120
VFOV = 60
HRES = 640
VRES = 640
BALL_RADIUS = 0.06207

# def rotate_cam_frame(cv,cu,T_frame):
#     theta = (VFOV/VRES)*(cv-(VRES/2))
#     phi = (HFOV/HRES)*(cu-(HRES/2))

#     Rot_mtx = tft.euler_matrix(0,theta,phi,'sxyz')
#     #tft.quaternion_from_euler
#     Rot_frame = Rot_mtx * T_frame
#     return np.array(Rot_frame)

def broadcaster_frame_object(frame, child_frame, pose):   # Emite la transformacion en el frame base_link,
    #br = tf2_ros.TransformBroadcaster()
    br =  tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.frame_id = frame
    t.child_frame_id = child_frame 
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation.x = pose.orientation.x
    t.transform.rotation.y = pose.orientation.y
    t.transform.rotation.z = pose.orientation.z
    t.transform.rotation.w = pose.orientation.w
    br.sendTransform(t)

def get_direction_vector(cv,cu):
    theta = (VFOV/VRES)*(cv-(VRES/2))
    phi = (HFOV/HRES)*(cu-(HRES/2))

    Rot_mtx = tft.euler_matrix(0,theta,phi,'sxyz')
    dir_vect = Rot_mtx @ np.array([1.0,0.0,0.0])

    direc = PointStamped()
    direc.header.frame_id = 'camera_optical'
    direc.point.x = dir_vect[0]
    direc.point.y = dir_vect[1]
    direc.point.z = dir_vect[2]

    return direc

def callback_process_object(msg):
    global tf_listener, tf_buf, scaling_constant, cam_range
    u = msg.x
    v = msg.y
    w = msg.width
    h = msg.height
    cu = (u+w)/2
    cv = (v+h)/2

    #d = scaling_constant * (w+h)/2

    oc = PointStamped()
    oc.header.frame_id = 'camera_optical'
    oc.point = Point(x=0,y=0,z=0)

    cam_pos = tf_buf.transform(oc,'left_foot_plane_link')
    ball_dir = tf_buf.transform(get_direction_vector(cv,cu),'left_foot_plane_link')

    print(cam_pos,q)
    cam_pos = np.array([cam_pos.x,cam_pos.y,cam_pos.z])
    ball_dir = np.array([ball_dir.x,ball_dir.y,ball_dir.z])

    line_vect = (cam_pos-ball_dir)/np.linalg.norm(cam_pos-ball_dir)

    L = (BALL_RADIUS-cam_pos[0])/line_vect[0]
    xb = cam_pos[0] + L*line_vect[0]
    yb = cam_pos[1] + L*line_vect[1]
    zb = BALL_RADIUS

    obj_pose = Pose()
    obj_pose.position.x = xb
    obj_pose.position.y = yb
    obj_pose.position.z = zb
    obj_pose.orientation = Quaternion(x=0,y=0,z=0,w=1)

    broadcaster_frame_object('left_foot_plane_link','ball_frame',obj_pose)
    
    # try:
    #     cam_frame = tf_buf.lookup_transform('left_foot_plane_link','camera_optical', rospy.Time())
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     print('No transform available')

    # Rot_frame = rotate_cam_frame(cv,cu,cam_frame)
    # cam_pos = Rot_frame @ np.array(0.0,0.0,0.0,1.0)
    # line_vect = Rot_frame @ np.array(1.0,0.0,0.0,1.0)
    # line_vect = line_vect/np.linalg.norm(line_vect)

    # L = (BALL_RADIUS-cam_pos[0])/line_vect[0]
    # xb = cam_pos[0] + L*line_vect[0]
    # yb = cam_pos[1] + L*line_vect[1]
    # zb = BALL_RADIUS

    resp = VisionObject()
    resp.header = msg.header
    resp.id = msg.id
    resp.confidence = msg.confidence
    resp.pose = Pose()
    resp.x = xb
    resp.y = yb

    return resp


def main():
    global tf_listener, tf_buf, scaling_constant, cam_range
    rospy.sleep(0.01)
    print('Starting Process Object Service')
    cam_range = rospy.get_param('/cam_angle_range',PI)
    scaling_constant = rospy.get_param('/cam_scaling_constant',1)
    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    rospy.init_node('process_obj_srv')
    rospy.Service('/intercept_plane_service' ,ProcessObject ,callback_process_object)
    loop = rospy.Rate(1)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass