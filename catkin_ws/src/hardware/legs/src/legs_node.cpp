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

#define DEVICE_NAME                   "/dev/ttyUSB0"
#define BAUDRATE         		          57600
#define PROTOCOL_VERSION 			          1.0

#define ADDR_MX_TORQUE_ENABLE            24
#define TORQUE_ENABLE         	      	  1
#define TORQUE_DISABLE         	          0
#define ADDR_MX_TORQUE_MAX            	 14
#define TORQUE_MAX                     1023	// Range 0-1023
#define ADDR_MX_TORQUE_LIMIT             34
#define TORQUE_LIMIT                   1023	// Range 0-1023

#define ADDR_MX_CURRENT_POSITION         36
#define ADDR_MX_GOAL_POSITION            30

#define LEG_LEFT_YAW_ID                   0
#define LEG_LEFT_PITCH_ID                 1
#define LEG_LEFT_ROLL_ID                  2
#define LEG_LEFT_KNEE_PITCH_ID            3
#define LEG_LEFT_ANKLE_PITCH_ID           4
#define LEG_LEFT_ANKLE_ROLL_ID            5

#define LEG_RIGHT_YAW_ID                  6
#define LEG_RIGHT_PITCH_ID                7
#define LEG_RIGHT_ROLL_ID                 8
#define LEG_RIGHT_KNEE_PITCH_ID          9
#define LEG_RIGHT_ANKLE_PITCH_ID         10
#define LEG_RIGHT_ANKLE_ROLL_ID          11

#define SERVO_MX_RANGE_IN_BITS 	   	   4096
#define SERVO_MX_RANGE_IN_RADS 	    (2*M_PI)
#define SERVO_MX_BITS_PER_RAD  	   	SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS

#define LEG_LEFT_YAW_ZERO			         2048
#define LEG_LEFT_YAW_CW  			            1

#define LEG_LEFT_PITCH_ZERO        	   2048
#define LEG_LEFT_PITCH_CW                 1

#define LEG_LEFT_ROLL_ZERO     	   	   2048
#define LEG_LEFT_ROLL_CW                  1

#define LEG_LEFT_KNEE_PITCH_ZERO       2048
#define LEG_LEFT_KNEE_PITCH_CW            1
   
#define LEG_LEFT_ANKLE_PITCH_ZERO      2048
#define LEG_LEFT_ANKLE_PITCH_CW           1

#define LEG_LEFT_ANKLE_ROLL_ZERO       2048
#define LEG_LEFT_ANKLE_ROLL_CW            1


#define LEG_RIGHT_YAW_ZERO			       2048
#define LEG_RIGHT_YAW_CW       		        1

#define LEG_RIGHT_PITCH_ZERO     	     2048
#define LEG_RIGHT_PITCH_CW       	        1

#define LEG_RIGHT_ROLL_ZERO     	     2048
#define LEG_RIGHT_ROLL_CW       	        1

#define LEG_RIGHT_KNEE_PITCH_ZERO      2048
#define LEG_RIGHT_KNEE_PITCH_CW           1

#define LEG_RIGHT_ANKLE_PITCH_ZERO     2048
#define LEG_RIGHT_ANKLE_PITCH_CW          1

#define LEG_RIGHT_ANKLE_ROLL_ZERO      2048
#define LEG_RIGHT_ANKLE_ROLL_CW           1

#define SERVO_MX_STEP_SIMUL              30

bool new_goal_position = false;

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

    new_goal_position = true;
}

void callback_goal_pose_left(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=0; i < 6; i++)
	  goal_position[i] = uint16_t(msg->data[i] * SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS * clockwise_direction[i] + position_zero_bits[i]);

    new_goal_position = true;
}

void callback_goal_pose_right(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=6; i < 12; i++)
	  goal_position[i] = uint16_t(msg->data[i - 6] * SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS * clockwise_direction[i] + position_zero_bits[i]);

    new_goal_position = true;
}

int main(int argc, char** argv)
 {     
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

    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if(portHandler->openPort())
		 std::cout << "Legs.->Serial port successfully openned" << std::endl;
    else
    	{
		 std::cout << "Legs.->Cannot open serial port" << std::endl;
		 return -1;
    	}

    if(portHandler->setBaudRate(BAUDRATE))
		 std::cout << "Legs.->Baudrate successfully set to " << BAUDRATE << std::endl;
    else
    	{
	     std::cout << "Legs.->Cannot set baud rate" << std::endl;
		 return -1;
    	}

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    for(int i=0; i < 12; i++)
    {
        packetHandler->write2ByteTxRx(portHandler, i, ADDR_MX_TORQUE_MAX, TORQUE_MAX, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, i, ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT, &dxl_error);
    }

    uint16_t dxl_current_pos [12];
    uint16_t dxl_current_pos_test [12];

    for(int i=0; i < 12; i++)
    {
        dxl_current_pos [i] = position_zero_bits[i];
        dxl_current_pos_test[i] = position_zero_bits[i];
    }

    while(ros::ok())
    {
      for(int i=0; i < 12; i++)
      {
         dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, i, ADDR_MX_CURRENT_POSITION, &dxl_current_pos_test[i], &dxl_error);

         if (dxl_comm_result != COMM_SUCCESS)
            {
            //packetHandler->printTxRxResult(dxl_comm_result);
            }
        else if (dxl_error != 0)
            {
            //packetHandler->printRxPacketError(dxl_error);
            }
        else if (dxl_comm_result == COMM_SUCCESS)
            {
            //packetHandler->printTxRxResult(dxl_comm_result);
            dxl_current_pos[i]=dxl_current_pos_test[i];
            }
      }

      joint_state_legs.header.stamp = ros::Time::now();

      for(int i=0; i < 12; i++)
      {
         joint_state_legs.position[i] = ((int)(dxl_current_pos[i]) - position_zero_bits[i]) * clockwise_direction[i] *
         (SERVO_MX_RANGE_IN_RADS/SERVO_MX_RANGE_IN_BITS);
      }

      joint_pub.publish(joint_state_legs);
       
      if(new_goal_position = true)
	    {
	        new_goal_position = false;

          for(int i=0; i < 12; i++)
          {
	           packetHandler->write1ByteTxRx(portHandler, i, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	           packetHandler->write2ByteTxRx(portHandler, i, ADDR_MX_GOAL_POSITION, goal_position[i], &dxl_error);
          }
	    }
      ros::spinOnce();
      loop.sleep();
    }
}

