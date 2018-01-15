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

#define ADDR_MX_TORQUE_ENABLE            24
#define TORQUE_ENABLE                     1
#define TORQUE_DISABLE                    0
#define ADDR_MX_TORQUE_MAX               14
#define TORQUE_MAX                     1023 // Range 0-1023
#define ADDR_MX_TORQUE_LIMIT             34
#define TORQUE_LIMIT                   1023 // Range 0-1023

#define ADDR_MX_CURRENT_POSITION         36
#define ADDR_MX_GOAL_POSITION            30

#define HEAD_NECK_PITCH_ID               18
#define HEAD_NECK_JAW_ID                 19

#define SERVO_MX_RANGE_IN_BITS 		4096
#define SERVO_MX_RANGE_IN_RADS 		(2*M_PI)
#define SERVO_MX_BITS_PER_RAD  		SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS

#define HEAD_NECK_PITCH_ZERO     	2048
#define HEAD_NECK_PITCH_CW             1

#define HEAD_NECK_JAW_ZERO     		2048
#define HEAD_NECK_JAW_CW               1

#define SERVO_MX_STEP_SIMUL 		  30

bool new_goal_position = false;

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

    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if(portHandler->openPort())
         std::cout << "Head.->Serial port successfully openned" << std::endl;
    else
        {
         std::cout << "Head.->Cannot open serial port" << std::endl;
         return -1;
        }

    if(portHandler->setBaudRate(BAUDRATE))
         std::cout << "Head.->Baudrate successfully set to " << BAUDRATE << std::endl;
    else
        {
         std::cout << "Head.->Cannot set baud rate" << std::endl;
         return -1;
        }

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    for(int i=18; i < 20; i++)
    {
        packetHandler->write2ByteTxRx(portHandler, i, ADDR_MX_TORQUE_MAX, TORQUE_MAX, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, i, ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT, &dxl_error);
    }
  
    uint16_t dxl_current_pos [2];
    uint16_t dxl_current_pos_test [2];

    for(int i=0; i < 2; i++)
    {
        dxl_current_pos [i] = position_zero_bits[i];
        dxl_current_pos_test[i] = position_zero_bits[i];
    }

    while(ros::ok())
    {
        for(int i=18; i < 20; i++)
        {
            dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, i, ADDR_MX_CURRENT_POSITION, &dxl_current_pos_test[i - 18], &dxl_error);

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
                dxl_current_pos[i - 18]=dxl_current_pos_test[i - 18];
                }
        }
  
        joint_states_head.header.stamp = ros::Time::now();

        for(int i=0; i < 2; i++)
        {
            joint_states_head.position[i] = ((int)(dxl_current_pos[i]) - position_zero_bits[i]) * clockwise_direction[i] * 
            (SERVO_MX_RANGE_IN_RADS/SERVO_MX_RANGE_IN_BITS);
        }
  
        //Send the joint state and transform
        joint_pub.publish(joint_states_head);

        if(new_goal_position = true)
        {
          new_goal_position = false;

          for(int i=18; i <20; i++)
          {
               packetHandler->write1ByteTxRx(portHandler, i, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
               packetHandler->write2ByteTxRx(portHandler, i, ADDR_MX_GOAL_POSITION, goal_position[i - 18], &dxl_error);
          }
        }
        
        ros::spinOnce();
        loop.sleep();
    }
  } 
