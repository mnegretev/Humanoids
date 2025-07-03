#ifndef WALKHANDLER_H
#define WALKHANDLER_H

#include <ros/ros.h>
#include "darwin_gait/WalkGains.h"
#include "IKWalk.hpp"

class WalkNode {
public:
    WalkNode(ros::NodeHandle& nh, int rate);
    bool runWalk( 
        double timeLength, 
        double phase, 
        double time);

    bool fillHumanoidParameters();

    bool handleSetGains(darwin_gait::WalkGains::Request& req,
                        darwin_gait::WalkGains::Response& res);

    bool start();

private:
    ros::NodeHandle& nh_;
    ros::Publisher legs_pub_;
    ros::Publisher arms_pub_;
    ros::ServiceServer walk_server_;
    ros::Rate rate_;
    struct Rhoban::IKWalkParameters params_;
    
};

template <typename T>
bool getParamWithLog(const std::string& param_name, T& value, const T& default_value);

#endif // MY_ROS_NODE_H