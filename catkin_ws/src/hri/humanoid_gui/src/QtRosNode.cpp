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
    pubLegsGoalPose     = n->advertise<std_msgs::Float32MultiArray>("/hardware/legs_goal_pose", 1);
    pubArmLeftGoalPose  = n->advertise<std_msgs::Float32MultiArray>("/hardware/arm_left_goal_pose", 1);
    pubArmRightGoalPose = n->advertise<std_msgs::Float32MultiArray>("/hardware/arm_right_goal_pose", 1);
    pubHeadGoalPose     = n->advertise<std_msgs::Float32MultiArray>("/hardware/head_goal_pose", 1);
    srvResetGazeboWorld = n->serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    cltCalculateIKLegLeft   = n->serviceClient<ctrl_msgs::CalculateIK>("/control/ik_leg_left");
    cltCalculateIKLegRight  = n->serviceClient<ctrl_msgs::CalculateIK>("/control/ik_leg_right");
    cltCalculateDKLegLeft   = n->serviceClient<ctrl_msgs::CalculateDK>("/control/dk_leg_left");
    cltCalculateDKLegRight  = n->serviceClient<ctrl_msgs::CalculateDK>("/control/dk_leg_right");
    cltPolynomialTrajectory = n->serviceClient<manip_msgs::GetPolynomialTrajectory>("/manipulation/polynomial_trajectory");
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

void QtRosNode::publishLegsGoalPose(std::vector<float> legsGoalPose)
{
    msgLegsGoalPose.data = legsGoalPose;
    pubLegsGoalPose.publish(msgLegsGoalPose);
}

void QtRosNode::publishArmLeftGoalPose(std::vector<float> armLeftGoalPose)
{
    msgArmLeftGoalPose.data = armLeftGoalPose;
    pubArmLeftGoalPose.publish(msgArmLeftGoalPose);
}

void QtRosNode::publishArmRightGoalPose(std::vector<float> armRightGoalPose)
{
    msgArmRightGoalPose.data = armRightGoalPose;
    pubArmRightGoalPose.publish(msgArmRightGoalPose);
}

void QtRosNode::publishHeadGoalPose(std::vector<float> headGoalPose)
{
    msgHeadGoalPose.data = headGoalPose;
    pubHeadGoalPose.publish(msgHeadGoalPose);
}

bool QtRosNode::callIKLegLeft(float x, float y, float z, float roll, float pitch, float yaw,
                                       std::vector<float>& result)
{
    result.resize(6);
    ctrl_msgs::CalculateIK srv;
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
    ctrl_msgs::CalculateIK srv;
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
    ctrl_msgs::CalculateDK srv;
    srv.request.joint_values = joints;
    cltCalculateDKLegLeft.call(srv);
    x = srv.response.x;
    y = srv.response.y;
    z = srv.response.z;
    roll = srv.response.roll;
    pitch = srv.response.pitch;
    yaw = srv.response.yaw;
    return true;
}

bool QtRosNode::callDKLegRight(std::vector<float>& joints, float& x, float& y, float& z, float& roll, float& pitch,float& yaw)
{
    ctrl_msgs::CalculateDK srv;
    srv.request.joint_values = joints;
    cltCalculateDKLegRight.call(srv);
    x = srv.response.x;
    y = srv.response.y;
    z = srv.response.z;
    roll = srv.response.roll;
    pitch = srv.response.pitch;
    yaw = srv.response.yaw;
    return true;
}

bool QtRosNode::callPolynomialTrajectory(std::vector<double>  p_init, std::vector<double> p_final)
{
    manip_msgs::GetPolynomialTrajectory srv;
    srv.request.p1 = p_init;
    srv.request.p2 = p_final;
    srv.request.duration = 1;
    srv.request.time_step = 0.05;
    cltPolynomialTrajectory.call(srv);
    

}

bool QtRosNode::getAllJointCurrentAngles(std::vector<float>& angles)
{
    angles.resize(20);
    try
    {
	std_msgs::Float32MultiArray::ConstPtr msg = ros::topic::waitForMessage<std_msgs::Float32MultiArray>(
	    "/hardware/joint_current_angles", ros::Duration(10));
	if(msg->data.size() != 20)
	{
	    std::cout << "QtRosNode.->current joint angles message should be 20 values long " << std::endl;
	    return false;
	}
	for(int i=0; i < 20; i++)
	    angles[i] = msg->data[i];
    }
    catch(...)
    {
	std::cout << "QtRosNode.->Cannot get current joint angles from topic :'(" << std::endl;
    }
    return true;
}

bool QtRosNode::getAllJointCurrentAngles(std::vector<double>& angles)
{
    angles.resize(20);
    try
    {
	std_msgs::Float32MultiArray::ConstPtr msg = ros::topic::waitForMessage<std_msgs::Float32MultiArray>(
	    "/hardware/joint_current_angles", ros::Duration(10));
	if(msg->data.size() != 20)
	{
	    std::cout << "QtRosNode.->current joint angles message should be 20 values long " << std::endl;
	    return false;
	}
	for(int i=0; i < 20; i++)
	    angles[i] = msg->data[i];
    }
    catch(...)
    {
	std::cout << "QtRosNode.->Cannot get current joint angles from topic :'(" << std::endl;
    }
    return true;
}

void QtRosNode::publishResetWorldGazebo()
{
    srvResetGazeboWorld.call(reset_world);
}
