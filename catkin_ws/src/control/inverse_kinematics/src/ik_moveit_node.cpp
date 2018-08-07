#include "ros/ros.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "ctrl_msgs/CalculateIK.h"
#include "ctrl_msgs/CalculateDK.h"

robot_state::RobotStatePtr kinematic_state;
robot_state::JointModelGroup* joint_group_leg_left;
robot_state::JointModelGroup* joint_group_leg_right;
std::vector<std::string> joint_names_leg_left;
std::vector<std::string> joint_names_leg_right;

bool ik_leg_left(ctrl_msgs::CalculateIK::Request &req, ctrl_msgs::CalculateIK::Response &res)
{
    std::cout << "IK.->Calculating inverse kinematics for left leg..." << std::endl;
    Eigen::Affine3d desired_pose = Eigen::Affine3d::Identity();
    desired_pose.translate(Eigen::Vector3d(req.x, req.y, req.z));
    desired_pose.rotate(Eigen::AngleAxisd(req.yaw,   Eigen::Vector3d(0,0,1)));
    desired_pose.rotate(Eigen::AngleAxisd(req.pitch, Eigen::Vector3d(0,1,0)));
    desired_pose.rotate(Eigen::AngleAxisd(req.roll,  Eigen::Vector3d(1,0,0)));
    //std::cout << "IK.->Desired position:" << std::endl;
    //std::cout << desired_pose.translation() << std::endl;
    //std::cout << "IK.->Desired orientation:" << std::endl;
    //std::cout << desired_pose.rotation() << std::endl;

    bool found_ik = kinematic_state->setFromIK(joint_group_leg_left, desired_pose, 5, 0.1);
    std::vector<double> result;
    if(found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_group_leg_left, result);
        res.joint_values.resize(result.size());
        for(int i=0; i < result.size(); i++) res.joint_values[i] = result[i];
        //std::cout << "IK.->Resulting joint values for left leg:" << std::endl;
        //for(int i=0; i < res.joint_values.size(); i++)
        //    std::cout << joint_names_leg_left[i] << " = " << res.joint_values[i] << std::endl;
    }
    else
        std::cout << "IK.->Cannot calculate inverse kinematics for left leg:'(" << std::endl;
    
    return found_ik;
}

bool ik_leg_right(ctrl_msgs::CalculateIK::Request &req, ctrl_msgs::CalculateIK::Response &res)
{
    std::cout << "IK.->Calculating inverse kinematics for right leg..." << std::endl;
    Eigen::Affine3d desired_pose = Eigen::Affine3d::Identity();
    desired_pose.translate(Eigen::Vector3d(req.x, req.y, req.z));
    desired_pose.rotate(Eigen::AngleAxisd(req.yaw,   Eigen::Vector3d(0,0,1)));
    desired_pose.rotate(Eigen::AngleAxisd(req.pitch, Eigen::Vector3d(0,1,0)));
    desired_pose.rotate(Eigen::AngleAxisd(req.roll,  Eigen::Vector3d(1,0,0)));
    //std::cout << "IK.->Desired position:" << std::endl;
    //std::cout << desired_pose.translation() << std::endl;
    //std::cout << "IK.->Desired orientation:" << std::endl;
    //std::cout << desired_pose.rotation() << std::endl;

    bool found_ik = kinematic_state->setFromIK(joint_group_leg_right, desired_pose, 5, 0.1);
    std::vector<double> result;
    if(found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_group_leg_right, result);
        res.joint_values.resize(result.size());
        for(int i=0; i < result.size(); i++) res.joint_values[i] = result[i];
        //std::cout << "IK.->Resulting joint values for right leg:" << std::endl;
        //for(int i=0; i < res.joint_values.size(); i++)
        //    std::cout << joint_names_leg_right[i] << " = " << res.joint_values[i] << std::endl;
    }
    else
        std::cout << "IK.->Cannot calculate inverse kinematics for right leg:'(" << std::endl;
    
    return found_ik;
}

bool dk_leg_left(ctrl_msgs::CalculateDK::Request &req, ctrl_msgs::CalculateDK::Response &res)
{
    if(req.joint_values.size() != 6)
        return false;
    std::vector<double> joints;
    joints.resize(6);
    for(int i=0; i < joints.size(); i++) joints[i] = req.joint_values[i];

    kinematic_state->setJointGroupPositions("leg_left", joints);
    Eigen::Affine3d pose = kinematic_state->getGlobalLinkTransform("left_foot_link");
    res.x = pose.translation()[0];
    res.y = pose.translation()[1];
    res.z = pose.translation()[2];
    Eigen::Vector3d euler = pose.linear().eulerAngles(0,1,2);
    res.roll  = euler[0];
    res.pitch = euler[1];
    res.yaw   = euler[2];
    return true;
}

bool dk_leg_right(ctrl_msgs::CalculateDK::Request &req, ctrl_msgs::CalculateDK::Response &res)
{
    if(req.joint_values.size() != 6)
        return false;
    std::vector<double> joints;
    joints.resize(6);
    for(int i=0; i < joints.size(); i++) joints[i] = req.joint_values[i];
    
    kinematic_state->setJointGroupPositions("leg_right", joints);
    Eigen::Affine3d pose = kinematic_state->getGlobalLinkTransform("right_foot_link");
    res.x = pose.translation()[0];
    res.y = pose.translation()[1];
    res.z = pose.translation()[2];
    Eigen::Vector3d euler = pose.linear().eulerAngles(0,1,2);
    res.roll  = euler[0];
    res.pitch = euler[1];
    res.yaw   = euler[2];
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING INVERSE KINEMATICS NODE BY MARCOSOFT, USING MOVEIT" << std::endl;
    ros::init(argc, argv, "ik_moveit_node");
    ros::NodeHandle n;
    ros::ServiceServer srvIKLegLeft  = n.advertiseService("/control/ik_leg_left",  ik_leg_left);
    ros::ServiceServer srvIKLegRight = n.advertiseService("/control/ik_leg_right", ik_leg_right);
    ros::ServiceServer srvDKLegLeft  = n.advertiseService("/control/dk_leg_left",  dk_leg_left);
    ros::ServiceServer srvDKLegRight = n.advertiseService("/control/dk_leg_right", dk_leg_right);
    ros::Rate loop(10);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    std::cout << "Kinematic model frame: " << kinematic_model->getModelFrame() << std::endl;

    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    joint_group_leg_left  = kinematic_model->getJointModelGroup("leg_left");
    joint_group_leg_right = kinematic_model->getJointModelGroup("leg_right");
    joint_names_leg_left  = joint_group_leg_left->getVariableNames();
    joint_names_leg_right = joint_group_leg_right->getVariableNames();

    while(ros::ok())
    {
	ros::spinOnce();
	loop.sleep();
    }
}
