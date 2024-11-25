#include "ros/ros.h"
#include "cm730_utils.h"


int main(int argc, char** argv)
{   
    ros::init(argc, argv, "cm730v2");
    CM730::Node node("/dev/ttyUSB0");
    
    if(!node.start()) return -1;

    if(node.writePresentPositions())
    {
        return -1;
    }
    while(ros::ok())
    {   
        node.readAndPublishAllPositions();
        ros::spinOnce();
    }

    node.stop();
    
    return 0;
}
