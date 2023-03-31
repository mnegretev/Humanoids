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
#include "std_srvs/Empty.h"
#include "ctrl_msgs/CalculateIK.h"
#include "ctrl_msgs/CalculateDK.h"

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
    ros::Publisher pubLegsGoalPose;
    ros::Publisher pubArmLeftGoalPose;
    ros::Publisher pubArmRightGoalPose;
    ros::Publisher pubHeadGoalPose;
    std_msgs::Float32MultiArray msgLegLeftGoalPose;
    std_msgs::Float32MultiArray msgLegRightGoalPose;
    std_msgs::Float32MultiArray msgLegsGoalPose;
    std_msgs::Float32MultiArray msgArmLeftGoalPose;
    std_msgs::Float32MultiArray msgArmRightGoalPose;
    std_msgs::Float32MultiArray msgHeadGoalPose;
    ros::ServiceClient cltCalculateIKLegLeft;
    ros::ServiceClient cltCalculateIKLegRight;
    ros::ServiceClient cltCalculateDKLegLeft;
    ros::ServiceClient cltCalculateDKLegRight;

    std_srvs::Empty reset_world;    
    ros::ServiceClient srvResetGazeboWorld;


    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publishLegLeftGoalPose(std::vector<float> legLeftGoalPose);
    void publishLegRightGoalPose(std::vector<float> legRightGoalPose);
    void publishLegsGoalPose(std::vector<float> legsGoalPose);
    void publishArmLeftGoalPose(std::vector<float> armLeftGoalPose);
    void publishArmRightGoalPose(std::vector<float> armRightGoalPose);
    void publishHeadGoalPose(std::vector<float> headGoalPose);
    void publishResetWorldGazebo();
    bool callIKLegLeft(float x, float y, float z, float roll, float pitch, float yaw,std::vector<float>& result);
    bool callIKLegRight(float x, float y, float z, float roll, float pitch,float yaw,std::vector<float>& result);
    bool callDKLegLeft(std::vector<float>& joints, float& x, float& y, float& z, float& roll, float& pitch, float& yaw);
    bool callDKLegRight(std::vector<float>& joints, float& x, float& y, float& z, float& roll, float& pitch,float& yaw);
    bool getLegLeftPositionOnce(std::vector<float>& joints);
    bool getLegRightPositionOnce(std::vector<float>& joints);
    bool getArmLeftPositionOnce(std::vector<float>& joints);
    bool getArmRightPositionOnce(std::vector<float>& joints);
    bool getHeadPositionOnce(std::vector<float>& joints);
    bool getAllJointCurrentAngles(std::vector<float>& angles);

signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
