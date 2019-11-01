#include "ros/ros.h" 
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16MultiArray.h"
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
#define BAUDRATE                          1000000
#define PROTOCOL_VERSION                      1.0

#define ADDR_MX_TORQUE_ENABLE            24
#define TORQUE_ENABLE                     1
#define TORQUE_DISABLE                    0
#define ADDR_MX_TORQUE_MAX               14
#define TORQUE_MAX                     1023 // Range 0-1023
#define ADDR_MX_TORQUE_LIMIT             34
#define TORQUE_LIMIT                   1023 // Range 0-1023

#define ADDR_MX_TORQUE_MODE              70
#define TORQUE_MODE_ON                    1
#define TORQUE_MODE_OFF                   0

#define ADDR_MX_MOVING_SPEED             32 // Range 0-1023
#define MX_MOVING_SPEED                  40 // The unit is about 0.114 rpm

#define ADDR_MX_CURRENT_POSITION         36
#define ADDR_MX_GOAL_POSITION            30

#define ADDR_MX_GOAL_TORQUE              71

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

#define SERVO_MX_RANGE_IN_BITS         4096
#define SERVO_MX_RANGE_IN_RADS      (2*M_PI)
#define SERVO_MX_BITS_PER_RAD       SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS

#define LEG_LEFT_YAW_ZERO                    1543
#define LEG_LEFT_YAW_CW                         1

#define LEG_LEFT_PITCH_ZERO            1600
#define LEG_LEFT_PITCH_CW                 1

#define LEG_LEFT_ROLL_ZERO             1800
#define LEG_LEFT_ROLL_CW                  1

#define LEG_LEFT_KNEE_PITCH_ZERO       2100
#define LEG_LEFT_KNEE_PITCH_CW            1
   
#define LEG_LEFT_ANKLE_PITCH_ZERO      2048
#define LEG_LEFT_ANKLE_PITCH_CW           1

#define LEG_LEFT_ANKLE_ROLL_ZERO       1800
#define LEG_LEFT_ANKLE_ROLL_CW            1


#define LEG_RIGHT_YAW_ZERO                 1050
#define LEG_RIGHT_YAW_CW                    1

#define LEG_RIGHT_PITCH_ZERO             2440
#define LEG_RIGHT_PITCH_CW                  1

#define LEG_RIGHT_ROLL_ZERO              2680
#define LEG_RIGHT_ROLL_CW                   1

#define LEG_RIGHT_KNEE_PITCH_ZERO      2048
#define LEG_RIGHT_KNEE_PITCH_CW           1

#define LEG_RIGHT_ANKLE_PITCH_ZERO     2048
#define LEG_RIGHT_ANKLE_PITCH_CW          1

#define LEG_RIGHT_ANKLE_ROLL_ZERO      2048
#define LEG_RIGHT_ANKLE_ROLL_CW           1

#define SERVO_MX_STEP_SIMUL              30

bool new_goal_position = false;
bool new_torque_mode = false;

uint16_t goal_position [12];
uint16_t goal_torque [12];  //If you use from 0~1023, torque is on toward CCW, and when you set it to 0, it stops.
                            //If you use from 1024~2047, torque is on toward CW, and when you set it to 1024, it stops.

dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;

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

int dxl_ID[12] = {
    LEG_LEFT_YAW_ID,         
    LEG_LEFT_PITCH_ID,       
    LEG_LEFT_ROLL_ID,        
    LEG_LEFT_KNEE_PITCH_ID,  
    LEG_LEFT_ANKLE_PITCH_ID, 
    LEG_LEFT_ANKLE_ROLL_ID,  
    LEG_RIGHT_YAW_ID,        
    LEG_RIGHT_PITCH_ID,      
    LEG_RIGHT_ROLL_ID,       
    LEG_RIGHT_KNEE_PITCH_ID, 
    LEG_RIGHT_ANKLE_PITCH_ID,
    LEG_RIGHT_ANKLE_ROLL_ID 
};

void callback_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=0; i < 12; i++)
    {
        goal_position[i] = uint16_t(msg->data[i] * SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS * clockwise_direction[i] + position_zero_bits[i]);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ID[i], ADDR_MX_TORQUE_MODE, TORQUE_MODE_OFF, &dxl_error);
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
        }
    }
    new_goal_position = true;
}

void callback_goal_pose_left(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=0; i < 6; i++)
    {
        goal_position[i] = uint16_t(msg->data[i] * SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS * clockwise_direction[i] + position_zero_bits[i]);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ID[i], ADDR_MX_TORQUE_MODE, TORQUE_MODE_OFF, &dxl_error);
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
        }
    }
    new_goal_position = true;
}

void callback_goal_pose_right(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=6; i < 12; i++)
    {
        goal_position[i] = uint16_t(msg->data[i - 6] * SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS * clockwise_direction[i] + position_zero_bits[i]);
       dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ID[i], ADDR_MX_TORQUE_MODE, TORQUE_MODE_OFF, &dxl_error);
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
        }     
    }
    new_goal_position = true;
}

void callback_goal_torque(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=0; i < 12; i++)
    {
        goal_torque[i] = uint16_t(1024 * msg->data[i] + 1024);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ID[i], ADDR_MX_TORQUE_MODE, TORQUE_MODE_ON, &dxl_error);
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
        }
    }
    new_torque_mode = true;
}

void callback_goal_torque_left(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=0; i < 6; i++)
    {
        goal_torque[i] = uint16_t(1024 * msg->data[i] + 1024);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ID[i], ADDR_MX_TORQUE_MODE, TORQUE_MODE_ON, &dxl_error);
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
        }      
    }
    new_torque_mode = true;
}

void callback_goal_torque_right(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=6; i < 12; i++)
    {
        goal_torque[i] = uint16_t(1024 * msg->data[i-6] + 1024);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ID[i], ADDR_MX_TORQUE_MODE, TORQUE_MODE_ON, &dxl_error);
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
        }      
    }
    new_torque_mode = true;
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
    ros::Subscriber subLegsGoalTorque   = n.subscribe("legs_goal_torque", 1, callback_goal_torque);
    ros::Subscriber subLegLeftGoalTorque   = n.subscribe("legs_left_goal_torque", 1, callback_goal_torque_left);
    ros::Subscriber subLegRightGoalTorque   = n.subscribe("legs_right_goal_torque", 1, callback_goal_torque_right);
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
    for(int i=0; i < 12; i++)
    {
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_ID[i], ADDR_MX_TORQUE_MAX, TORQUE_MAX, &dxl_error);
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
        }
    }
    for(int i=0; i < 12; i++)
    {
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_ID[i], ADDR_MX_MOVING_SPEED, MX_MOVING_SPEED, &dxl_error);
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
        }   
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
            dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_ID[i], ADDR_MX_CURRENT_POSITION, &dxl_current_pos_test[i], &dxl_error);
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
        if(new_goal_position == true)
        {
            new_goal_position = false;
            for(int i=0; i < 12; i++)
            {
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_ID[i], ADDR_MX_GOAL_POSITION, goal_position[i], &dxl_error);
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
                }
            }
        }
        if(new_torque_mode == true)
        {
            new_torque_mode = false;
            for(int i=0; i < 12; i++)
            {
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_ID[i], ADDR_MX_GOAL_TORQUE, goal_torque[i], &dxl_error);
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
                }
            }
        }
        ros::spinOnce();
        loop.sleep();
    }
}
