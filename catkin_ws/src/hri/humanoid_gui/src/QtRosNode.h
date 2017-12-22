#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    bool gui_closed;

    ros::Publisher pubLegLeftGoalPose;
    ros::Publisher pubLegRightGoalPose;
    std_msgs::Float32MultiArray msgLegLeftGoalPose;
    std_msgs::Float32MultiArray msgLegRightGoalPose;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publishLegLeftGoalPose(std::vector<float> legLeftGoalPose);
    void publishLegRightGoalPose(std::vector<float> legRightGoalPose);

signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
