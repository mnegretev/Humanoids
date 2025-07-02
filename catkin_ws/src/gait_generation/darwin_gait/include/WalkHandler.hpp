#ifndef WALKHANDLER_H
#define WALKHANDLER_H

#include <ros/ros.h>
#include <darwin_gait/SetGains.h>
#include "IKWalk.hpp"

class WalkNode {
public:
    WalkNode(ros::NodeHandle& nh);
    void runWalk(
        const Rhoban::IKWalkParameters& params, 
        double timeLength, 
        double& phase, 
        double& time,
        ros::Publisher& pub,
        ros::Rate& rate);

    void fillHumanoidParameters();

    bool handleSetGains(your_package::SetGains::Request& req,
                        your_package::SetGains::Response& res);

private:
    ros::NodeHandle nh_;
    ros::Publisher legs_pub_;
    ros::ServiceServer walk_server_;
    ros::Rate rate_;
    struct Rhoban::IKWalkParameters params;
    
};

#endif // MY_ROS_NODE_H