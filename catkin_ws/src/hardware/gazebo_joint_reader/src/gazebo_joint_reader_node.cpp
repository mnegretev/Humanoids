#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/GetJointProperties.h"
#include "gazebo_msgs/GetLinkState.h"
#include "tf/transform_broadcaster.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING GAZEBO JOINT READER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "gazebo_joint_reader");
    ros::NodeHandle n;
    ros::Rate loop(30);

    ros::Publisher pub_legs_current_pose      = n.advertise<std_msgs::Float32MultiArray>("legs_current_pose", 1);
    ros::Publisher pub_leg_left_current_pose  = n.advertise<std_msgs::Float32MultiArray>("leg_left_current_pose", 1);
    ros::Publisher pub_leg_right_current_pose = n.advertise<std_msgs::Float32MultiArray>("leg_right_current_pose", 1);
    ros::Publisher pub_arms_current_pose      = n.advertise<std_msgs::Float32MultiArray>("arms_current_pose", 1);
    ros::Publisher pub_arm_left_current_pose  = n.advertise<std_msgs::Float32MultiArray>("arm_left_current_pose", 1);
    ros::Publisher pub_arm_right_current_pose = n.advertise<std_msgs::Float32MultiArray>("arm_right_current_pose", 1);
    ros::Publisher pub_head_current_pose      = n.advertise<std_msgs::Float32MultiArray>("head_current_pose", 1);
    ros::Publisher pub_joint_current_angles   = n.advertise<std_msgs::Float32MultiArray>("joint_current_angles", 1);
    std_msgs::Float32MultiArray msg_joint_current_angles;
    msg_joint_current_angles.data.resize(20);
    ros::service::waitForService("/gazebo/get_joint_properties", 10);
    ros::service::waitForService("/gazebo/get_link_state", 10);
    ros::ServiceClient clt_joints       = n.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
    ros::ServiceClient clt_trunk_state  = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    gazebo_msgs::GetJointProperties srv_joints;
    gazebo_msgs::GetLinkState       srv_trunk_state;
    srv_trunk_state.request.link_name = "nimbro_op::trunk_link";

    ros::Publisher  pub_joint_states = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    sensor_msgs::JointState joint_states;

    joint_states.name.resize(20);
    joint_states.position.resize(20);

    joint_states.name[0]  ="left_hip_yaw";   
    joint_states.name[1]  ="left_hip_roll";  
    joint_states.name[2]  ="left_hip_pitch"; 
    joint_states.name[3]  ="left_knee_pitch";
    joint_states.name[4]  ="left_ankle_pitch";
    joint_states.name[5]  ="left_ankle_roll";
  
    joint_states.name[6]  ="right_hip_yaw";
    joint_states.name[7]  ="right_hip_roll";
    joint_states.name[8]  ="right_hip_pitch";
    joint_states.name[9]  ="right_knee_pitch";
    joint_states.name[10] ="right_ankle_pitch";
    joint_states.name[11] ="right_ankle_roll";

    joint_states.name[12] ="left_shoulder_pitch";
    joint_states.name[13] ="left_shoulder_roll";
    joint_states.name[14] ="left_elbow_pitch";

    joint_states.name[15] ="right_shoulder_pitch";
    joint_states.name[16] ="right_shoulder_roll";
    joint_states.name[17] ="right_elbow_pitch";

    joint_states.name[18] ="neck_yaw";
    joint_states.name[19] ="head_pitch";

    tf::TransformBroadcaster br;
    tf::Transform t;

    while(ros::ok())
    {
        joint_states.header.stamp = ros::Time::now();
        for(size_t i=0; i < joint_states.name.size(); i++)
        {
            srv_joints.request.joint_name = joint_states.name[i];
            clt_joints.call(srv_joints);
            joint_states.position[i] = srv_joints.response.position[0];
            msg_joint_current_angles.data[i] = joint_states.position[i];
        }

        clt_trunk_state.call(srv_trunk_state);
        t.setOrigin(tf::Vector3(srv_trunk_state.response.link_state.pose.position.x,
                                srv_trunk_state.response.link_state.pose.position.y,
                                srv_trunk_state.response.link_state.pose.position.z));
        t.setRotation(tf::Quaternion(srv_trunk_state.response.link_state.pose.orientation.x,
                                     srv_trunk_state.response.link_state.pose.orientation.y,
                                     srv_trunk_state.response.link_state.pose.orientation.z,
                                     srv_trunk_state.response.link_state.pose.orientation.w));

        br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "map", "trunk_link"));
        pub_joint_states.publish(joint_states);
        pub_joint_current_angles.publish(msg_joint_current_angles);
        ros::spinOnce();
        loop.sleep();
    }
}
