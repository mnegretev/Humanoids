#include <iostream>

#include "ros/ros.h"

int main(int argc, char* argv[])
{
    ROS_INFO("INITIALIZING LIPM WALK NODE BY MIGUEL GARCIA");
    ros::init(argc, argv, "lipm_walk_node");
    ros::NodeHandle n;
    
    return 0;
}
