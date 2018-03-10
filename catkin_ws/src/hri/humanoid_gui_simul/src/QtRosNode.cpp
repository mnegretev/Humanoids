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
    pubLegLeftHipYaw      = n->advertise<std_msgs::Float64>("/nimbro/left_hip_yaw_position_controller/command",1);
    pubLegLeftHipRoll     = n->advertise<std_msgs::Float64>("/nimbro/left_hip_roll_position_controller/command",1);
    pubLegLeftHipPitch    = n->advertise<std_msgs::Float64>("/nimbro/left_hip_pitch_position_controller/command",1);
    pubLegLeftKneePitch   = n->advertise<std_msgs::Float64>("/nimbro/left_knee_pitch_position_controller/command",1);
    pubLegLeftAnklePitch  = n->advertise<std_msgs::Float64>("/nimbro/left_ankle_pitch_position_controller/command",1);
    pubLegLeftAnkleRoll   = n->advertise<std_msgs::Float64>("/nimbro/left_ankle_roll_position_controller/command",1); 
    pubLegRightHipYaw     = n->advertise<std_msgs::Float64>("/nimbro/right_hip_yaw_position_controller/command",1);    
    pubLegRightHipRoll    = n->advertise<std_msgs::Float64>("/nimbro/right_hip_roll_position_controller/command",1);   
    pubLegRightHipPitch   = n->advertise<std_msgs::Float64>("/nimbro/right_hip_pitch_position_controller/command",1);  
    pubLegRightKneePitch  = n->advertise<std_msgs::Float64>("/nimbro/right_knee_pitch_position_controller/command",1); 
    pubLegRightAnklePitch = n->advertise<std_msgs::Float64>("/nimbro/right_ankle_pitch_position_controller/command",1);
    pubLegRightAnkleRoll  = n->advertise<std_msgs::Float64>("/nimbro/right_ankle_roll_position_controller/command",1); 
    
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
    std_msgs::Float64 msg_hip_yaw;
    std_msgs::Float64 msg_hip_roll;
    std_msgs::Float64 msg_hip_pitch;
    std_msgs::Float64 msg_knee_pitch;
    std_msgs::Float64 msg_ankle_pitch;
    std_msgs::Float64 msg_ankle_roll; 
    msg_hip_yaw.data     = legLeftGoalPose[0];
    msg_hip_roll.data    = legLeftGoalPose[1];
    msg_hip_pitch.data   = legLeftGoalPose[2];
    msg_knee_pitch.data  = legLeftGoalPose[3]; 
    msg_ankle_pitch.data = legLeftGoalPose[4];
    msg_ankle_roll.data  = legLeftGoalPose[5];
    pubLegLeftHipYaw    .publish(msg_hip_yaw);
    pubLegLeftHipRoll   .publish(msg_hip_roll);
    pubLegLeftHipPitch  .publish(msg_hip_pitch);
    pubLegLeftKneePitch .publish(msg_knee_pitch);
    pubLegLeftAnklePitch.publish(msg_ankle_pitch);
    pubLegLeftAnkleRoll .publish(msg_ankle_roll);
}

void QtRosNode::publishLegRightGoalPose(std::vector<float> legRightGoalPose)
{
    std_msgs::Float64 msg_hip_yaw;
    std_msgs::Float64 msg_hip_roll;
    std_msgs::Float64 msg_hip_pitch;
    std_msgs::Float64 msg_knee_pitch;
    std_msgs::Float64 msg_ankle_pitch;
    std_msgs::Float64 msg_ankle_roll; 
    msg_hip_yaw.data     = legRightGoalPose[0];
    msg_hip_roll.data    = legRightGoalPose[1];
    msg_hip_pitch.data   = legRightGoalPose[2];
    msg_knee_pitch.data  = legRightGoalPose[3]; 
    msg_ankle_pitch.data = legRightGoalPose[4];
    msg_ankle_roll.data  = legRightGoalPose[5];
    pubLegRightHipYaw    .publish(msg_hip_yaw);
    pubLegRightHipRoll   .publish(msg_hip_roll);
    pubLegRightHipPitch  .publish(msg_hip_pitch);
    pubLegRightKneePitch .publish(msg_knee_pitch);
    pubLegRightAnklePitch.publish(msg_ankle_pitch);
    pubLegRightAnkleRoll .publish(msg_ankle_roll);
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
