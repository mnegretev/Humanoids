#include "ros/ros.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING INVERSE KINEMATICS NODE BY MARCOSOFT, USING MOVEIT" << std::endl;
    ros::init(argc, argv, "ik_moveit_node");
    ros::NodeHandle n;
    ros::Rate loop(10);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    std::cout << "Kinematic model: " << kinematic_model->getModelFrame() << std::endl;

    while(ros::ok())
    {
	ros::spinOnce();
	loop.sleep();
    }
}
