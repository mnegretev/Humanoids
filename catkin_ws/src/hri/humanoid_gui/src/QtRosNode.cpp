#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{
    pubLegLeftGoalPose  = n->advertise<std_msgs::Float32MultiArray>("/hardware/leg_left_goal_pose", 1);
    pubLegRightGoalPose = n->advertise<std_msgs::Float32MultiArray>("/hardware/leg_right_goal_pose", 1);
    cltCalculateIKLegLeft  = n->serviceClient<control_msgs::CalculateIK>("/control/ik_leg_left");
    cltCalculateIKLegRight = n->serviceClient<control_msgs::CalculateIK>("/control/ik_leg_right");
    cltCalculateDKLegLeft  = n->serviceClient<control_msgs::CalculateDK>("/control/dk_leg_left");
    cltCalculateDKLegRight = n->serviceClient<control_msgs::CalculateDK>("/control/dk_leg_right");
        
    ros::Rate loop(10);
    while(ros::ok() && !this->gui_closed)
    {
        //std::cout << "Ros node running..." << std::endl;
        emit updateGraphics();
        loop.sleep();
        ros::spinOnce();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
}

void QtRosNode::publishLegLeftGoalPose(std::vector<float> legLeftGoalPose)
{
    msgLegLeftGoalPose.data = legLeftGoalPose;
    pubLegLeftGoalPose.publish(msgLegLeftGoalPose);
}

void QtRosNode::publishLegRightGoalPose(std::vector<float> legRightGoalPose)
{
    msgLegRightGoalPose.data = legRightGoalPose;
    pubLegRightGoalPose.publish(msgLegRightGoalPose);
}

bool QtRosNode::callIKLegLeft(float x, float y, float z, float roll, float pitch, float yaw,
                                       std::vector<float>& result)
{
    result.resize(6);
    control_msgs::CalculateIK srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    srv.request.roll = roll;
    srv.request.pitch = pitch;
    srv.request.yaw = yaw;
    if(!cltCalculateIKLegLeft.call(srv))
        return false;

    for(int i=0; i < 6; i++)
        result[i] = (float)srv.response.joint_values[i];
    return true;
}

bool QtRosNode::callIKLegRight(float x, float y, float z, float roll, float pitch, float yaw,
                                        std::vector<float>& result)
{
    result.resize(6);
    control_msgs::CalculateIK srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    srv.request.roll = roll;
    srv.request.pitch = pitch;
    srv.request.yaw = yaw;
    if(!cltCalculateIKLegRight.call(srv))
        return false;

    for(int i=0; i < 6; i++)
        result[i] = (float)srv.response.joint_values[i];
    return true;
}
bool QtRosNode::callDKLegLeft(std::vector<float>& joints, float& x, float& y, float& z, float& roll, float& pitch, float& yaw)
{
    control_msgs::CalculateDK srv;
    srv.request.joint_values = joints;
    cltCalculateDKLegLeft.call(srv);
    x = srv.response.x;
    y = srv.response.y;
    z = srv.response.z;
    roll = srv.response.roll;
    pitch = srv.response.pitch;
    yaw = srv.response.yaw;
}

bool QtRosNode::callDKLegRight(std::vector<float>& joints, float& x, float& y, float& z, float& roll, float& pitch,float& yaw)
{
    control_msgs::CalculateDK srv;
    srv.request.joint_values = joints;
    cltCalculateDKLegRight.call(srv);
    x = srv.response.x;
    y = srv.response.y;
    z = srv.response.z;
    roll = srv.response.roll;
    pitch = srv.response.pitch;
    yaw = srv.response.yaw;
}
