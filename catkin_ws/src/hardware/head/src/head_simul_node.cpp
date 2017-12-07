#include "ros/ros.h" 
#include "std_msgs/Float32MultiArray.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <unistd.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define DEVICE_NAME      			"/dev/ttyUSB0"
#define BAUDRATE         			57600
#define PROTOCOL_VERSION 			1.0

#define SERVO_MX_RANGE_IN_BITS 		4096
#define SERVO_MX_RANGE_IN_RADS 		2*M_PI
#define SERVO_MX_BITS_PER_RAD  		SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS

#define HEAD_NECK_PITCH_ZERO     	2048
#define HEAD_NECK_PITCH_CW             1

#define HEAD_NECK_JAW_ZERO     		2048
#define HEAD_NECK_JAW_CW               1

#define SERVO_MX_STEP_SIMUL 		  30

uint16_t goal_position [2];

int position_zero_bits[2] = {
    HEAD_NECK_PITCH_ZERO,
	HEAD_NECK_JAW_ZERO
};

int clockwise_direction[2] = {
    HEAD_NECK_PITCH_CW,
	HEAD_NECK_JAW_CW
};

void callback_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=0; i < 2; i++)
  	goal_position[i] = uint16_t(msg->data[i] * SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS * clockwise_direction[i] + position_zero_bits[i]);
}

int main(int argc, char** argv)
 {     
    //Initilize joint_states_publisher
    std::cout << "INITIALIZING HEAD NODE..." << std::endl;
    ros::init(argc, argv, "head");
    ros::NodeHandle n;
    ros::Rate loop(30);
    ros::Subscriber subHeadGoalPose = n.subscribe("head_goal_pose", 1, callback_goal_pose);
    ros::Publisher  joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

    sensor_msgs::JointState joint_states_head;
  
    joint_states_head.name.resize(2);
    joint_states_head.position.resize(2);
 
    joint_states_head.name[0] ="neck_yaw";
    joint_states_head.name[1] ="head_pitch";
  
    uint16_t dxl_current_pos_sim [2];
    for(int i=0; i < 2; i++)
    {
        goal_position[i] = position_zero_bits[i];
        dxl_current_pos_sim[i] = position_zero_bits[i];
    }

    while(ros::ok())
    {
        for(int i=0; i < 2; i++)
        {
            int error = goal_position[i] - dxl_current_pos_sim[i];
            if(error >  SERVO_MX_STEP_SIMUL) error =  SERVO_MX_STEP_SIMUL;
            if(error < -SERVO_MX_STEP_SIMUL) error = -SERVO_MX_STEP_SIMUL;
            dxl_current_pos_sim[i] += error;
        }
  
        joint_states_head.header.stamp = ros::Time::now();

        for(int i=0; i < 2; i++)
        {
            joint_states_head.position[i] = ((int)(dxl_current_pos_sim[i]) - position_zero_bits[i]) * clockwise_direction[i] * 
            (SERVO_MX_RANGE_IN_RADS/SERVO_MX_RANGE_IN_BITS);
        }
  
        //Send the joint state and transform
        joint_pub.publish(joint_states_head);
        
        ros::spinOnce();
        loop.sleep();
    }
  } 
