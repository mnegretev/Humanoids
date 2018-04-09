#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/GetLinkProperties.h"
#include "geometry_msgs/PointStamped.h"

std::vector<double>      link_mass;
std::vector<std::string> link_name;
double links_total_mass;
geometry_msgs::PointStamped msg_FCoM;

void callback_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    int trunk_index  = 31;

    double CoM_x = 0;
    double CoM_y = 0;
    double CoM_z = 0;

    for(int i=trunk_index; i < msg->name.size(); i++)
    {
        CoM_x += link_mass[i-trunk_index]*msg->pose[i].position.x;
        CoM_y += link_mass[i-trunk_index]*msg->pose[i].position.y;
        CoM_z += link_mass[i-trunk_index]*msg->pose[i].position.z;
    }
    CoM_x /= links_total_mass;
    CoM_y /= links_total_mass;
    CoM_z /= links_total_mass;

    //std::cout << "Center of mass in :" << CoM_x << "\t" << CoM_y << "\t" << CoM_z << std::endl;
    msg_FCoM.point.x = CoM_x;
    msg_FCoM.point.y = CoM_y;
    msg_FCoM.point.z = 0;  //Floor projection of the center of mass.
    
    //From idx 31 to the end, are the nimbro op links
    //std::cout << "Link " << msg->name[link_index] << " orientation=" << msg->pose[link_index].orientation.x;
    //std::cout << "\t" << msg->pose[link_index].orientation.y << "\t" << msg->pose[link_index].orientation.z << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING CONTROL TEST NODE BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "control_test");
    ros::NodeHandle n;
    ros::Rate loop(30);

    ros::Subscriber gazebo_link_states  = n.subscribe("/gazebo/link_states", 1, callback_link_states);
    ros::Publisher  pub_FCoM = n.advertise<geometry_msgs::PointStamped>("/cart_table/FCoM", 1);

    link_name.resize(21);
    link_mass.resize(21);
    link_name[0]  = "nimbro_op::trunk_link";
    link_name[1]  = "nimbro_op::left_hip_yaw_link";
    link_name[2]  = "nimbro_op::left_hip_roll_link";
    link_name[3]  = "nimbro_op::left_thigh_link";
    link_name[4]  = "nimbro_op::left_shank_link";
    link_name[5]  = "nimbro_op::left_ankle_link";
    link_name[6]  = "nimbro_op::left_foot_link";
    link_name[7]  = "nimbro_op::left_shoulder_pitch_link";
    link_name[8]  = "nimbro_op::left_upper_arm_link";
    link_name[9]  = "nimbro_op::left_lower_arm_link";
    link_name[10] = "nimbro_op::neck_link";
    link_name[11] = "nimbro_op::head_link";
    link_name[12] = "nimbro_op::right_hip_yaw_link";
    link_name[13] = "nimbro_op::right_hip_roll_link";
    link_name[14] = "nimbro_op::right_thigh_link";
    link_name[15] = "nimbro_op::right_shank_link";
    link_name[16] = "nimbro_op::right_ankle_link";
    link_name[17] = "nimbro_op::right_foot_link";
    link_name[18] = "nimbro_op::right_shoulder_pitch_link";
    link_name[19] = "nimbro_op::right_upper_arm_link";
    link_name[20] = "nimbro_op::right_lower_arm_link";

    ros::service::waitForService("/gazebo/get_link_properties", 15);
    ros::ServiceClient clt_link_props = n.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");
    gazebo_msgs::GetLinkProperties srv_props;
    
    links_total_mass = 0;
    for(int i=0; i < link_name.size(); i++)
    {
        srv_props.request.link_name = link_name[i];
        clt_link_props.call(srv_props);
        link_mass[i] = srv_props.response.mass;
        links_total_mass += link_mass[i];
    }
    msg_FCoM.header.frame_id = "map";

    while(ros::ok())
    {
        msg_FCoM.header.stamp = ros::Time::now();
        pub_FCoM.publish(msg_FCoM);
        ros::spinOnce();
        loop.sleep();
    }
}
