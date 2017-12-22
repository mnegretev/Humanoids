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
