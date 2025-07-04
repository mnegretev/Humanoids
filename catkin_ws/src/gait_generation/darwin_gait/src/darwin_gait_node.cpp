#include <iostream>
#include <ros/ros.h>
#include "WalkHandler.hpp"
#include "IKWalk.hpp"
#include "darwin_gait/WalkGains.h" // Include your updated service header
#include "std_msgs/Float32MultiArray.h"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "walk_node");
    ros::NodeHandle n;
    WalkNode node(n, 40);
    node.start();
    ros::spin();
    return 0;
}

