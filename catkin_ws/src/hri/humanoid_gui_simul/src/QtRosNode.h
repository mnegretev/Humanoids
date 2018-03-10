#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "control_msgs/CalculateIK.h"
#include "control_msgs/CalculateDK.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    bool gui_closed;

    ros::Publisher pubLegLeftHipYaw;
    ros::Publisher pubLegLeftHipRoll;
    ros::Publisher pubLegLeftHipPitch;
    ros::Publisher pubLegLeftKneePitch;
    ros::Publisher pubLegLeftAnklePitch;
    ros::Publisher pubLegLeftAnkleRoll;
    ros::Publisher pubLegRightHipYaw;
    ros::Publisher pubLegRightHipRoll;
    ros::Publisher pubLegRightHipPitch;
    ros::Publisher pubLegRightKneePitch;
    ros::Publisher pubLegRightAnklePitch;
    ros::Publisher pubLegRightAnkleRoll;  

    ros::ServiceClient cltCalculateIKLegLeft;
    ros::ServiceClient cltCalculateIKLegRight;
    ros::ServiceClient cltCalculateDKLegLeft;
    ros::ServiceClient cltCalculateDKLegRight;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publishLegLeftGoalPose(std::vector<float> legLeftGoalPose);
    void publishLegRightGoalPose(std::vector<float> legRightGoalPose);
    bool callIKLegLeft(float x, float y, float z, float roll, float pitch, float yaw,std::vector<float>& result);
    bool callIKLegRight(float x, float y, float z, float roll, float pitch,float yaw,std::vector<float>& result);
    bool callDKLegLeft(std::vector<float>& joints, float& x, float& y, float& z, float& roll, float& pitch, float& yaw);
    bool callDKLegRight(std::vector<float>& joints, float& x, float& y, float& z, float& roll, float& pitch,float& yaw);

signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
