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

#define DEVICE_NAME                      "/dev/ttyUSB0"
#define BAUDRATE                         57600
#define PROTOCOL_VERSION                 1.0

#define ADDR_MX_TORQUE_ENABLE            24
#define TORQUE_ENABLE                     1
#define TORQUE_DISABLE                    0
#define ADDR_MX_TORQUE_MAX               14
#define TORQUE_MAX                     1023 // Range 0-1023
#define ADDR_MX_TORQUE_LIMIT             34
#define TORQUE_LIMIT                   1023 // Range 0-1023

#define ADDR_MX_CURRENT_POSITION         36
#define ADDR_MX_GOAL_POSITION            30

#define ARM_LEFT_SHOULDER_PITCH_ID          12
#define ARM_LEFT_SHOULDER_ROLL_ID           13
#define ARM_LEFT_ELBOW_PITCH_ID             14
#define ARM_RIGHT_SHOULDER_PITCH_ID         15
#define ARM_RIGHT_SHOULDER_ROLL_ID          16
#define ARM_RIGHT_ELBOW_PITCH_ID            17

#define SERVO_MX_RANGE_IN_BITS           4096
#define SERVO_MX_RANGE_IN_RADS           (2*M_PI)
#define SERVO_MX_BITS_PER_RAD            SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS

#define ARM_LEFT_SHOULDER_PITCH_ZERO     2048
#define ARM_LEFT_SHOULDER_PITCH_CW          1

#define ARM_LEFT_SHOULDER_ROLL_ZERO      2048
#define ARM_LEFT_SHOULDER_ROLL_CW           1

#define ARM_LEFT_ELBOW_PITCH_ZERO        2048
#define ARM_LEFT_ELBOW_PITCH_CW             1


#define ARM_RIGHT_SHOULDER_PITCH_ZERO    2048
#define ARM_RIGHT_SHOULDER_PITCH_CW         1

#define ARM_RIGHT_SHOULDER_ROLL_ZERO     2048
#define ARM_RIGHT_SHOULDER_ROLL_CW          1

#define ARM_RIGHT_ELBOW_PITCH_ZERO       2048
#define ARM_RIGHT_ELBOW_PITCH_CW            1

#define SERVO_MX_STEP_SIMUL                30

bool new_goal_position = false;

uint16_t goal_position [6];

int position_zero_bits[6] = 
{
  ARM_LEFT_SHOULDER_PITCH_ZERO,
  ARM_LEFT_SHOULDER_ROLL_ZERO,
  ARM_LEFT_ELBOW_PITCH_ZERO,
  ARM_RIGHT_SHOULDER_PITCH_ZERO,
  ARM_RIGHT_SHOULDER_ROLL_ZERO,
  ARM_RIGHT_ELBOW_PITCH_ZERO 
};

int clockwise_direction[6] = 
{
  ARM_LEFT_SHOULDER_PITCH_CW,
  ARM_LEFT_SHOULDER_ROLL_CW,
  ARM_LEFT_ELBOW_PITCH_CW,
  ARM_RIGHT_SHOULDER_PITCH_CW,
  ARM_RIGHT_SHOULDER_ROLL_CW,
  ARM_RIGHT_ELBOW_PITCH_CW
};

void callback_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  for(int i=0; i < 6; i++)
    goal_position[i] = uint16_t(msg->data[i] * SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS * clockwise_direction[i] + position_zero_bits[i]);
}

void callback_goal_pose_left(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  for(int i=0; i < 6; i++)
    goal_position[i] = uint16_t(msg->data[i] * SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS * clockwise_direction[i] + position_zero_bits[i]);
}
void callback_goal_pose_right(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  for(int i=3; i < 6; i++)
    goal_position[i] = uint16_t(msg->data[i - 3] * SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS * clockwise_direction[i] + position_zero_bits[i]);
}

int main(int argc, char** argv)
{     
    //Initilize joint_state_publisher
    std::cout << "INITIALIZING ARMS NODE..." << std::endl;
    ros::init(argc, argv, "arms");
    ros::NodeHandle n;
    ros::Rate loop(30);
    ros::Subscriber subArmsGoalPose     = n.subscribe("arms_goal_pose", 1, callback_goal_pose);
    ros::Subscriber subArmLeftGoalPose  = n.subscribe("arm_left_goal_pose", 1, callback_goal_pose_left);
    ros::Subscriber subArmRightGoalPose = n.subscribe("arm_right_goal_pose", 1, callback_goal_pose_right);
    ros::Publisher  joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

    sensor_msgs::JointState joint_state_arms;
  
    joint_state_arms.name.resize(6);
    joint_state_arms.position.resize(6);
 
    joint_state_arms.name[0] ="left_shoulder_pitch";
    joint_state_arms.name[1] ="left_shoulder_roll";
    joint_state_arms.name[2] ="left_elbow_pitch";
 
    joint_state_arms.name[3] ="right_shoulder_pitch";
    joint_state_arms.name[4] ="right_shoulder_roll";
    joint_state_arms.name[5] ="right_elbow_pitch";

    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if(portHandler->openPort())
        std::cout << "Arms.->Serial port successfully openned" << std::endl;
    else
    {
        std::cout << "Arms.->Cannot open serial port" << std::endl;
        return -1;
    }

    if(portHandler->setBaudRate(BAUDRATE))
        std::cout << "Arms.->Baudrate successfully set to " << BAUDRATE << std::endl;
    else
    {
        std::cout << "Arms.->Cannot set baud rate" << std::endl;
        return -1;
    }

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    for(int i=12; i < 18; i++)
    {
        packetHandler->write2ByteTxRx(portHandler, i, ADDR_MX_TORQUE_MAX, TORQUE_MAX, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, i, ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT, &dxl_error);
    }

    uint16_t dxl_current_pos [6];
    uint16_t dxl_current_pos_test [6];

    for(int i=0; i < 6; i++)
    {
        dxl_current_pos [i] = position_zero_bits[i];
        dxl_current_pos_test[i] = position_zero_bits[i];
    }

    while(ros::ok())
    {
        for(int i=12; i < 18; i++)
        {
            dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, i, ADDR_MX_CURRENT_POSITION, &dxl_current_pos_test[i - 12], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                //packetHandler->printTxRxResult(dxl_comm_result);
            }
            else
                if (dxl_error != 0)
                {
                    //packetHandler->printRxPacketError(dxl_error);
                }
            else if (dxl_comm_result == COMM_SUCCESS)
            {
                //packetHandler->printTxRxResult(dxl_comm_result);
                dxl_current_pos[i - 12]=dxl_current_pos_test[i - 12];
            }
        }

        joint_state_arms.header.stamp = ros::Time::now();

        for(int i=0; i < 6; i++)
        {
            joint_state_arms.position[i] = ((int)(dxl_current_pos[i]) - position_zero_bits[i]) * clockwise_direction[i] *
            (SERVO_MX_RANGE_IN_RADS/SERVO_MX_RANGE_IN_BITS);
        }

        joint_pub.publish(joint_state_arms);
       
        if(new_goal_position = true)
        {
            new_goal_position = false;
            for(int i=12; i <18; i++)
            {
                packetHandler->write1ByteTxRx(portHandler, i, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
                packetHandler->write2ByteTxRx(portHandler, i, ADDR_MX_GOAL_POSITION, goal_position[i - 12], &dxl_error);
            }
        }
        ros::spinOnce();
        loop.sleep();
    }
} 