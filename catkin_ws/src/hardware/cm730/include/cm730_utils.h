#ifndef _CM730_UTILS_H_
#define _CM730_UTILS_H_
#pragma once

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "servo_utils.h"
#include <map>

namespace CM730
{
    class Node 
    {
    private: 
        ros::NodeHandle n;
        ros::Rate       rate{30};
        ros::Subscriber sub_legs_goal_pose; 
        ros::Subscriber sub_leg_left_goal_pose;  
        ros::Subscriber sub_leg_right_goal_pose;
        ros::Subscriber sub_arms_goal_pose;
        ros::Subscriber sub_arm_left_goal_pose;
        ros::Subscriber sub_arm_right_goal_pose; 
        ros::Subscriber sub_head_goal_pose;
    
        ros::Publisher pub_legs_current_pose;
        ros::Publisher pub_leg_left_current_pose;
        ros::Publisher pub_leg_right_current_pose;
        ros::Publisher pub_arms_current_pose;
        ros::Publisher pub_arm_left_current_pose;
        ros::Publisher pub_arm_right_current_pose;
        ros::Publisher pub_head_current_pose;
        ros::Publisher pub_joint_current_angles;
        ros::Publisher pub_joint_states;
        
        sensor_msgs::JointState     msg_joint_states;
        std_msgs::Float32MultiArray msg_joint_current_angles;

        std::vector<Servo::servo_t> all_servos;

        Servo::CommHandler comm;

        std::vector<uint16_t> present_position; //In Dynamixel bits
        std::vector<uint16_t> goal_position;    //In Dynamixel bits

    public:
        //Constructor
        Node(std::string port_name);

        //Methods
        bool start();
        void stop();
        
        bool fillServoParameters(const std::vector<std::string>& servo_names, std::vector<Servo::servo_t>& servo_list);
        bool readAndPublishAllPositions();
        bool writePresentPositions();
        bool writePositions(const std::vector<Servo::servo_t>& servos);

        //Callbacks
        void callback_legs_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void callback_leg_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void callback_leg_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg); 
        void callback_arms_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void callback_arm_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void callback_arm_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void callback_head_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    };
}

#endif
