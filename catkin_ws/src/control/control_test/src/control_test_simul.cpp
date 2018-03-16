#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"

void callback_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    //int trunk_index  = 31;
    
    //From idx 31 to the end, are the nimbro op links
    //std::cout << "Link " << msg->name[link_index] << " orientation=" << msg->pose[link_index].orientation.x;
    //std::cout << "\t" << msg->pose[link_index].orientation.y << "\t" << msg->pose[link_index].orientation.z << std::endl;
    
}

void callback_model_states(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    int nimbro_index = 5;
    std::cout << "Model " << msg->name[nimbro_index] << " position: " << msg->pose[nimbro_index].position.x;
    std::cout << "\t" << msg->pose[nimbro_index].position.y << "\t" << msg->pose[nimbro_index].position.z << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING CONTROL TEST NODE BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "control_test");
    ros::NodeHandle n;
    ros::Rate loop(20);

    ros::Subscriber gazebo_link_states  = n.subscribe("/gazebo/link_states", 1, callback_link_states);
    ros::Subscriber gazebo_model_states = n.subscribe("/gazebo/model_states", 1, callback_model_states);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
}
