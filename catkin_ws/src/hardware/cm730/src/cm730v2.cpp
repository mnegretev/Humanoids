#include "ros/ros.h"
#include "cm730_utils.h"


int main(int argc, char** argv)
{   
    ros::init(argc, argv, "cm730v2");
    CM730::Node node("/dev/ttyUSB0");
    
    if(!node.start()) return -1;

    while(ros::ok())
    {
        bool torque_enable_param;
        if (!ros::param::get("torque_enable", torque_enable_param))
        {
            ROS_ERROR("Missing param in config file: %s", "torque_enable");
            break;
        }
        if(torque_enable_param)
        {
            if(!node.writePresentPositions()) break;
        }
        while(ros::ok())
        {
            node.readAndPublishAllPositions();
            ros::spinOnce();
        }
    }
    node.stop();
    return 0;
}
