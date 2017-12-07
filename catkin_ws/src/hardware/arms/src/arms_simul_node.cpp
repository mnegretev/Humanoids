#include "ros/ros.h" 
#include "std_msgs/Float32.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
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

#define SERVO_MX_RANGE_IN_BITS           4096
#define SERVO_MX_RANGE_IN_RADS           2*M_PI
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

uint16_t goal_position [6];

int position_zero_bits[6] = {
    ARM_LEFT_SHOULDER_PITCH_ZERO,
    ARM_LEFT_SHOULDER_ROLL_ZERO,
    ARM_LEFT_ELBOW_PITCH_ZERO,
    ARM_RIGHT_SHOULDER_PITCH_ZERO,
    ARM_RIGHT_SHOULDER_ROLL_ZERO,
    ARM_RIGHT_ELBOW_PITCH_ZERO 
};

int clockwise_direction[6] = {
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
    goal_position[i] = uint16_t(msg->data[i] * 4096/(2*M_PI) * clockwise_direction[i] + position_zero_bits[i]);
}

void callback_goal_pose_left(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=0; i < 6; i++)
    goal_position[i] = uint16_t(msg->data[i] * 4096/(2*M_PI) * clockwise_direction[i] + position_zero_bits[i]);
}
void callback_goal_pose_right(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for(int i=3; i < 6; i++)
    goal_position[i] = uint16_t(msg->data[i - 3] * 4096/(2*M_PI) * clockwise_direction[i] + position_zero_bits[i]);
}

int main(int argc, char** argv)
 {     
    //Initilize joint_states_publisher
    std::cout << "INITIALIZING ARMS NODE..." << std::endl;
    ros::init(argc, argv, "arms");
    ros::NodeHandle n;
    ros::Rate loop(30);
    ros::Subscriber subArmsGoalPose     = n.subscribe("arms_goal_pose", 1, callback_goal_pose);
    ros::Subscriber subArmLeftGoalPose  = n.subscribe("arm_left_goal_pose", 1, callback_goal_pose_left);
    ros::Subscriber subArmRightGoalPose = n.subscribe("arm_right_goal_pose", 1, callback_goal_pose_right);
    ros::Publisher  joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

    sensor_msgs::JointState joint_states_arms;
  
    joint_states_arms.name.resize(6);
    joint_states_arms.position.resize(6);
 
    joint_states_arms.name[0] ="left_shoulder_pitch";
    joint_states_arms.name[1] ="left_shoulder_roll";
    joint_states_arms.name[2] ="left_elbow_pitch";
 
    joint_states_arms.name[3] ="right_shoulder_pitch";
    joint_states_arms.name[4] ="right_shoulder_roll";
    joint_states_arms.name[5] ="right_elbow_pitch";

    uint16_t dxl_current_pos_sim [6];
    for(int i=0; i < 6; i++)
    {
        goal_position[i] = position_zero_bits[i];
        dxl_current_pos_sim[i] = position_zero_bits[i];
    }

    while(ros::ok())
    {
        for(int i=0; i < 6; i++)
        {
            int error = goal_position[i] - dxl_current_pos_sim[i];
            if(error >  SERVO_MX_STEP_SIMUL) error =  SERVO_MX_STEP_SIMUL;
            if(error < -SERVO_MX_STEP_SIMUL) error = -SERVO_MX_STEP_SIMUL;
            dxl_current_pos_sim[i] += error;
        }
  
        joint_states_arms.header.stamp = ros::Time::now();

        for(int i=0; i < 6; i++)
        {
            joint_states_arms.position[i] = ((int)(dxl_current_pos_sim[i]) - position_zero_bits[i]) * clockwise_direction[i] * 
            (2*M_PI/4096.0);
        }
  
        //Send the joint state and transform
        joint_pub.publish(joint_states_arms);
        
        ros::spinOnce();
        loop.sleep();
    }
  } 
