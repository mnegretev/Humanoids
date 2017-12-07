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

#define LEG_LEFT_YAW_ZERO			2048
#define LEG_LEFT_YAW_CW  			   1

#define LEG_LEFT_PITCH_ZERO     	2048
#define LEG_LEFT_PITCH_CW              1

#define LEG_LEFT_ROLL_ZERO     		2048
#define LEG_LEFT_ROLL_CW               1

#define LEG_LEFT_KNEE_PITCH_ZERO    2048
#define LEG_LEFT_KNEE_PITCH_CW         1

#define LEG_LEFT_ANKLE_PITCH_ZERO   2048
#define LEG_LEFT_ANKLE_PITCH_CW        1

#define LEG_LEFT_ANKLE_ROLL_ZERO    2048
#define LEG_LEFT_ANKLE_ROLL_CW         1


#define LEG_RIGHT_YAW_ZERO			2048
#define LEG_RIGHT_YAW_CW       		   1

#define LEG_RIGHT_PITCH_ZERO     	2048
#define LEG_RIGHT_PITCH_CW       	   1

#define LEG_RIGHT_ROLL_ZERO     	2048
#define LEG_RIGHT_ROLL_CW       	   1

#define LEG_RIGHT_KNEE_PITCH_ZERO   2048
#define LEG_RIGHT_KNEE_PITCH_CW        1

#define LEG_RIGHT_ANKLE_PITCH_ZERO  2048
#define LEG_RIGHT_ANKLE_PITCH_CW       1

#define LEG_RIGHT_ANKLE_ROLL_ZERO   2048
#define LEG_RIGHT_ANKLE_ROLL_CW        1

#define SERVO_MX_STEP_SIMUL 30

uint16_t goal_position [12];

int position_zero_bits[12] = {
    LEG_LEFT_YAW_ZERO, 
    LEG_LEFT_PITCH_ZERO, 
    LEG_LEFT_ROLL_ZERO, 
    LEG_LEFT_KNEE_PITCH_ZERO, 
    LEG_LEFT_ANKLE_PITCH_ZERO,
    LEG_LEFT_ANKLE_ROLL_ZERO, 
    LEG_RIGHT_YAW_ZERO, 
    LEG_RIGHT_PITCH_ZERO, 
    LEG_RIGHT_ROLL_ZERO, 
    LEG_RIGHT_KNEE_PITCH_ZERO, 
    LEG_RIGHT_ANKLE_PITCH_ZERO, 
    LEG_RIGHT_ANKLE_ROLL_ZERO 
};

int clockwise_direction[12] = {
    LEG_LEFT_YAW_CW, 
    LEG_LEFT_PITCH_CW, 
    LEG_LEFT_ROLL_CW, 
    LEG_LEFT_KNEE_PITCH_CW, 
    LEG_LEFT_ANKLE_PITCH_CW,
    LEG_LEFT_ANKLE_ROLL_CW, 
    LEG_RIGHT_YAW_CW, 
    LEG_RIGHT_PITCH_CW, 
    LEG_RIGHT_ROLL_CW, 
    LEG_RIGHT_KNEE_PITCH_CW, 
    LEG_RIGHT_ANKLE_PITCH_CW, 
    LEG_RIGHT_ANKLE_ROLL_CW 
};

void callback_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=0; i < 12; i++)
	goal_position[i] = uint16_t(msg->data[i] * SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS * clockwise_direction[i] + position_zero_bits[i]);
}

void callback_goal_pose_left(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=0; i < 6; i++)
	goal_position[i] = uint16_t(msg->data[i] * SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS * clockwise_direction[i] + position_zero_bits[i]);
}
void callback_goal_pose_right(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=6; i < 12; i++)
	goal_position[i] = uint16_t(msg->data[i - 6] * SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS * clockwise_direction[i] + position_zero_bits[i]);
}

int main(int argc, char** argv)
 {     
    //Initilize joint_states_publisher
    std::cout << "INITIALIZING LEGS NODE..." << std::endl;
    ros::init(argc, argv, "legs");
    ros::NodeHandle n;
    ros::Rate loop(30);
    ros::Subscriber subLegsGoalPose     = n.subscribe("legs_goal_pose", 1, callback_goal_pose);
    ros::Subscriber subLegLeftGoalPose  = n.subscribe("leg_left_goal_pose", 1, callback_goal_pose_left);
    ros::Subscriber subLegRightGoalPose = n.subscribe("leg_right_goal_pose", 1, callback_goal_pose_right);
    ros::Publisher  joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

    sensor_msgs::JointState joint_state_legs;
  
    joint_state_legs.name.resize(12);
    joint_state_legs.position.resize(12);
  
    joint_state_legs.name[0] ="left_hip_yaw";   
    joint_state_legs.name[1] ="left_hip_roll";  
    joint_state_legs.name[2] ="left_hip_pitch"; 
    joint_state_legs.name[3] ="left_knee_pitch";
    joint_state_legs.name[4] ="left_ankle_pitch";
    joint_state_legs.name[5] ="left_ankle_roll";
  
    joint_state_legs.name[6]  ="right_hip_yaw";
    joint_state_legs.name[7]  ="right_hip_roll";
    joint_state_legs.name[8]  ="right_hip_pitch";
    joint_state_legs.name[9]  ="right_knee_pitch";
    joint_state_legs.name[10] ="right_ankle_pitch";
    joint_state_legs.name[11] ="right_ankle_roll";

    uint16_t dxl_current_pos_sim [12];

    for(int i=0; i < 12; i++)
    {
		goal_position[i] = position_zero_bits[i];
		dxl_current_pos_sim[i] = position_zero_bits[i];
    }
    
    while(ros::ok())
    {
		for(int i=0; i < 12; i++)
		{
	    	int error = goal_position[i] - dxl_current_pos_sim[i];
	    	if(error >  SERVO_MX_STEP_SIMUL) error =  SERVO_MX_STEP_SIMUL;
	    	if(error < -SERVO_MX_STEP_SIMUL) error = -SERVO_MX_STEP_SIMUL;
	    	dxl_current_pos_sim[i] += error;
		}
	
		joint_state_legs.header.stamp = ros::Time::now();

		for(int i=0; i < 12; i++)
		{
	    	joint_state_legs.position[i] = ((int)(dxl_current_pos_sim[i]) - position_zero_bits[i]) * clockwise_direction[i] *
			(SERVO_MX_RANGE_IN_RADS/SERVO_MX_RANGE_IN_BITS);
		}
	
		//Send the joint state and transform
		joint_pub.publish(joint_state_legs);
    	  
		ros::spinOnce();
		loop.sleep();
    }
 }
