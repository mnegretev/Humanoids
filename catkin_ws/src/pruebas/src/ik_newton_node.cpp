#include "ros/ros.h"
#include "urdf/model.h"




int main (int argc, char ** argv)
{
    std::cout << "Initializing Inverse kinematics node" << std::endl;
    ros::init(argc, argv, "ik_newton_node");
    ros::NodeHandle n;
    ros::Rate loop(10);
    std::string urdf_string;
    std::string joint;

    std::cout << "loading model" << std::endl;
    if(!n.getParam("/robot_description", urdf_string))
    {
        std::cout << "Can't read /robot_description" << std::endl;
        return -1;
    }

    urdf::Model robot_model;
    if (!robot_model.initString(urdf_string)) 
    {
        ROS_ERROR("Can't load robot description from string");
       return -1;
    }
    
    //std::cout << robot_model.getRoot()<< std::endl;
   for (const auto& pair : robot_model.joints_) 
    {
        std::cout << "Joint: " << pair.first << std::endl;
    }

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
}