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

#define DEVICE_NAME      "/dev/ttyUSB0"
#define BAUDRATE         57600
#define PROTOCOL_VERSION 1.0

#define SERVO_MX_RANGE_IN_BITS 4096
#define SERVO_MX_RANGE_IN_RADS 2*M_PI
#define SERVO_MX_BITS_PER_RAD  SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS

#define LEG_LEFT_YAW_ZERO     2048
#define LEG_LEFT_YAW_CW       1

#define LEG_LEFT_PITCH_ZERO     2048
#define LEG_LEFT_PITCH_CW       1

#define LEG_LEFT_ROLL_ZERO     2048
#define LEG_LEFT_ROLL_CW       1

#define LEG_LEFT_KNEE_PITCH_ZERO     2048
#define LEG_LEFT_KNEE_PITCH_CW       1

#define LEG_LEFT_ANKLE_PITCH_ZERO     2048
#define LEG_LEFT_ANKLE_PITCH_CW       1

#define LEG_LEFT_ANKLE_ROLL_ZERO     2048
#define LEG_LEFT_ANKLE_ROLL_CW       1


#define LEG_RIGHT_YAW_ZERO     2048
#define LEG_RIGHT_YAW_CW       1

#define LEG_RIGHT_PITCH_ZERO     2048
#define LEG_RIGHT_PITCH_CW       1

#define LEG_RIGHT_ROLL_ZERO     2048
#define LEG_RIGHT_ROLL_CW       1

#define LEG_RIGHT_KNEE_PITCH_ZERO     2048
#define LEG_RIGHT_KNEE_PITCH_CW       1

#define LEG_RIGHT_ANKLE_PITCH_ZERO     2048
#define LEG_RIGHT_ANKLE_PITCH_CW       1

#define LEG_RIGHT_ANKLE_ROLL_ZERO     2048
#define LEG_RIGHT_ANKLE_ROLL_CW       1

#define SERVO_MX_STEP_SIMUL 30

uint16_t goal_position [12] = \
{ \
	LEG_LEFT_YAW_ZERO, \
	LEG_LEFT_PITCH_ZERO, \
	LEG_LEFT_ROLL_ZERO, \
	LEG_LEFT_KNEE_PITCH_ZERO, \
	LEG_LEFT_ANKLE_PITCH_ZERO, \
	LEG_LEFT_ANKLE_ROLL_ZERO, \
	LEG_RIGHT_YAW_ZERO, \
	LEG_RIGHT_PITCH_ZERO, \
	LEG_RIGHT_ROLL_ZERO, \
	LEG_RIGHT_KNEE_PITCH_ZERO, \
	LEG_RIGHT_ANKLE_PITCH_ZERO, \
	LEG_RIGHT_ANKLE_ROLL_ZERO \
};

void callback_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    goal_position [0] = int(msg->data[0]*(4096/(2*M_PI))*LEG_LEFT_YAW_CW + LEG_LEFT_YAW_ZERO);
    goal_position [1] = int(msg->data[1]*(4096/(2*M_PI))*LEG_LEFT_PITCH_CW + LEG_LEFT_PITCH_ZERO);
    goal_position [2] = int(msg->data[2]*(4096/(2*M_PI))*LEG_LEFT_ROLL_CW + LEG_LEFT_ROLL_ZERO);
    goal_position [3] = int(msg->data[3]*(4096/(2*M_PI))*LEG_LEFT_KNEE_PITCH_CW + LEG_LEFT_KNEE_PITCH_ZERO);
    goal_position [4] = int(msg->data[4]*(4096/(2*M_PI))*LEG_LEFT_ANKLE_PITCH_CW + LEG_LEFT_ANKLE_PITCH_ZERO);
    goal_position [5] = int(msg->data[5]*(4096/(2*M_PI))*LEG_LEFT_ANKLE_ROLL_CW + LEG_LEFT_ANKLE_ROLL_ZERO);
    goal_position [6] = int(msg->data[6]*(4096/(2*M_PI))*LEG_RIGHT_YAW_CW + LEG_RIGHT_YAW_ZERO);
    goal_position [7] = int(msg->data[7]*(4096/(2*M_PI))*LEG_RIGHT_PITCH_CW + LEG_RIGHT_PITCH_ZERO);
    goal_position [8] = int(msg->data[8]*(4096/(2*M_PI))*LEG_RIGHT_ROLL_CW + LEG_RIGHT_ROLL_ZERO);
    goal_position [9] = int(msg->data[9]*(4096/(2*M_PI))*LEG_RIGHT_KNEE_PITCH_CW + LEG_RIGHT_KNEE_PITCH_ZERO);
    goal_position [10]= int(msg->data[10]*(4096/(2*M_PI))*LEG_RIGHT_ANKLE_PITCH_CW + LEG_RIGHT_ANKLE_PITCH_ZERO);
    goal_position [11] = int(msg->data[11]*(4096/(2*M_PI))*LEG_RIGHT_ANKLE_ROLL_CW + LEG_RIGHT_ANKLE_ROLL_ZERO);
}

void callback_goal_pose_left(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    goal_position [0] = int(msg->data[0]*(4096/(2*M_PI))*LEG_LEFT_YAW_CW + LEG_LEFT_YAW_ZERO);
    goal_position [1] = int(msg->data[1]*(4096/(2*M_PI))*LEG_LEFT_PITCH_CW + LEG_LEFT_PITCH_ZERO);
    goal_position [2] = int(msg->data[2]*(4096/(2*M_PI))*LEG_LEFT_ROLL_CW + LEG_LEFT_ROLL_ZERO);
    goal_position [3] = int(msg->data[3]*(4096/(2*M_PI))*LEG_LEFT_KNEE_PITCH_CW + LEG_LEFT_KNEE_PITCH_ZERO);
    goal_position [4] = int(msg->data[4]*(4096/(2*M_PI))*LEG_LEFT_ANKLE_PITCH_CW + LEG_LEFT_ANKLE_PITCH_ZERO);
    goal_position [5] = int(msg->data[5]*(4096/(2*M_PI))*LEG_LEFT_ANKLE_ROLL_CW + LEG_LEFT_ANKLE_ROLL_ZERO);
}
void callback_goal_pose_right(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    goal_position [6] = int(msg->data[6]*(4096/(2*M_PI))*LEG_RIGHT_YAW_CW + LEG_RIGHT_YAW_ZERO);
    goal_position [7] = int(msg->data[7]*(4096/(2*M_PI))*LEG_RIGHT_PITCH_CW + LEG_RIGHT_PITCH_ZERO);
    goal_position [8] = int(msg->data[8]*(4096/(2*M_PI))*LEG_RIGHT_ROLL_CW + LEG_RIGHT_ROLL_ZERO);
    goal_position [9] = int(msg->data[9]*(4096/(2*M_PI))*LEG_RIGHT_KNEE_PITCH_CW + LEG_RIGHT_KNEE_PITCH_ZERO);
    goal_position [10]= int(msg->data[10]*(4096/(2*M_PI))*LEG_RIGHT_ANKLE_PITCH_CW + LEG_RIGHT_ANKLE_PITCH_ZERO);
    goal_position [11] = int(msg->data[11]*(4096/(2*M_PI))*LEG_RIGHT_ANKLE_ROLL_CW + LEG_RIGHT_ANKLE_ROLL_ZERO);
}

int main(int argc, char** argv)
 {     
    //Initilize joint_states_publisher
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    sensor_msgs::JointState joint_state_legs;
  
    joint_state_legs.name.resize(12);
    joint_state_legs.position.resize(12);
  
    joint_state_legs.name[0] ="left_joint_leg_yaw";
    joint_state_legs.name[1] ="left_joint_leg_pitch";
    joint_state_legs.name[2] ="left_joint_leg_roll";
    joint_state_legs.name[3] ="left_joint_knee_pitch";
    joint_state_legs.name[4] ="left_joint_ankle_pitch";
    joint_state_legs.name[5] ="left_joint_ankle_roll";
  
    joint_state_legs.name[6] ="right_joint_leg_yaw";
    joint_state_legs.name[7] ="right_joint_leg_pitch";
    joint_state_legs.name[8] ="right_joint_leg_roll";
    joint_state_legs.name[9] ="right_joint_knee_pitch";
    joint_state_legs.name[10] ="right_joint_ankle_pitch";
    joint_state_legs.name[11] ="right_joint_ankle_roll";

    goal_position [0] = LEG_LEFT_YAW_ZERO;
	goal_position [1] = LEG_LEFT_PITCH_ZERO;
	goal_position [2] = LEG_LEFT_ROLL_ZERO;
	goal_position [3] = LEG_LEFT_KNEE_PITCH_ZERO;
	goal_position [4] = LEG_LEFT_ANKLE_PITCH_ZERO;
	goal_position [5] = LEG_LEFT_ANKLE_ROLL_ZERO;
	goal_position [6] = LEG_RIGHT_YAW_ZERO;
	goal_position [7] = LEG_RIGHT_PITCH_ZERO;
	goal_position [8] = LEG_RIGHT_ROLL_ZERO;
	goal_position [9] = LEG_RIGHT_KNEE_PITCH_ZERO;
	goal_position [10] = LEG_RIGHT_ANKLE_PITCH_ZERO;
	goal_position [11] = LEG_RIGHT_ANKLE_ROLL_ZERO;
 
    std::cout << "INITIALIZING LEGS NODE..." << std::endl;
    ros::init(argc, argv, "legs");
    ros::NodeHandle n0;
    ros::Rate loop(30);
    ros::Subscriber subGoalPose0 = n0.subscribe("legs_goal_pose", 1, callback_goal_pose);

    std::cout << "INITIALIZING LEFT LEG NODE..." << std::endl;
    ros::init(argc, argv, "left_leg");
    ros::NodeHandle n1;
    ros::Subscriber subGoalPose1 = n1.subscribe("left_leg_goal_pose", 1, callback_goal_pose_left);

    std::cout << "INITIALIZING RIGHT LEG NODE..." << std::endl;
    ros::init(argc, argv, "right_leg");
    ros::NodeHandle n2;
    ros::Subscriber subGoalPose2 = n2.subscribe("right_leg_goal_pose", 1, callback_goal_pose_right);

    uint16_t dxl_current_pos_sim [12] = \
    { \
		LEG_LEFT_YAW_ZERO, \
		LEG_LEFT_PITCH_ZERO, \
		LEG_LEFT_ROLL_ZERO, \
		LEG_LEFT_KNEE_PITCH_ZERO, \
		LEG_LEFT_ANKLE_PITCH_ZERO, \
		LEG_LEFT_ANKLE_ROLL_ZERO, \
		LEG_RIGHT_YAW_ZERO, \
		LEG_RIGHT_PITCH_ZERO, \
		LEG_RIGHT_ROLL_ZERO, \
		LEG_RIGHT_KNEE_PITCH_ZERO, \
		LEG_RIGHT_ANKLE_PITCH_ZERO, \
		LEG_RIGHT_ANKLE_ROLL_ZERO \
	};

    while(ros::ok())
    {
 
      joint_state_legs.header.stamp = ros::Time::now();
      joint_state_legs.position[0] = ((int)(dxl_current_pos_sim [0]) - LEG_LEFT_YAW_ZERO)*LEG_LEFT_YAW_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[1] = ((int)(dxl_current_pos_sim [1]) - LEG_LEFT_PITCH_ZERO)*LEG_LEFT_PITCH_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[2] = ((int)(dxl_current_pos_sim [2]) - LEG_LEFT_ROLL_ZERO)*LEG_LEFT_ROLL_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[3] = ((int)(dxl_current_pos_sim [3]) - LEG_LEFT_KNEE_PITCH_ZERO)*LEG_LEFT_KNEE_PITCH_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[4] = ((int)(dxl_current_pos_sim [4]) - LEG_LEFT_ANKLE_PITCH_ZERO)*LEG_LEFT_ANKLE_PITCH_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[5] = ((int)(dxl_current_pos_sim [5]) - LEG_LEFT_ANKLE_ROLL_ZERO)*LEG_LEFT_ANKLE_ROLL_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[6] = ((int)(dxl_current_pos_sim [6]) - LEG_RIGHT_YAW_ZERO)*LEG_RIGHT_YAW_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[7] = ((int)(dxl_current_pos_sim [7]) - LEG_RIGHT_PITCH_ZERO)*LEG_RIGHT_PITCH_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[8] = ((int)(dxl_current_pos_sim [8]) - LEG_RIGHT_ROLL_ZERO)*LEG_RIGHT_ROLL_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[9] = ((int)(dxl_current_pos_sim [9]) - LEG_RIGHT_KNEE_PITCH_ZERO)*LEG_RIGHT_KNEE_PITCH_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[10] = ((int)(dxl_current_pos_sim [10]) - LEG_RIGHT_ANKLE_PITCH_ZERO)*LEG_RIGHT_ANKLE_PITCH_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[11] = ((int)(dxl_current_pos_sim [11]) - LEG_RIGHT_ANKLE_ROLL_ZERO)*LEG_RIGHT_ANKLE_ROLL_CW * (2*M_PI / 4096.0);

      //Send the joint state and transform
      joint_pub.publish(joint_state_legs);

      int error [12];

      for(int i = 0;i<12;i++)
      {
      	error [i] = goal_position [i] - dxl_current_pos_sim [i];
      	if( error [i] > SERVO_MX_STEP_SIMUL) error [i] = SERVO_MX_STEP_SIMUL;
      	if(error [i] < -SERVO_MX_STEP_SIMUL) error [i] = -SERVO_MX_STEP_SIMUL;
      	dxl_current_pos_sim [i] += error [i];
  	  }

      //This logic works but uses lots of lines of code
/*
      // 0      if((dxl_current_pos_sim_0 - goal_position_0) > SERVO_MX_STEP_SIMUL)
        dxl_current_pos_sim_0 -= SERVO_MX_STEP_SIMUL;

      if((dxl_current_pos_sim_0 - goal_position_0) < -SERVO_MX_STEP_SIMUL)
        dxl_current_pos_sim_0 += SERVO_MX_STEP_SIMUL;

      if(abs(dxl_current_pos_sim_0 - goal_position_0) <= SERVO_MX_STEP_SIMUL && (dxl_current_pos_sim_0 - goal_position_0) > 0)
        dxl_current_pos_sim_0 -= abs(dxl_current_pos_sim_0 - goal_position_0);

      if(abs(dxl_current_pos_sim_0 - goal_position_0) <= SERVO_MX_STEP_SIMUL && (dxl_current_pos_sim_0 - goal_position_0) < 0)
        dxl_current_pos_sim_0 += abs(dxl_current_pos_sim_0 - goal_position_0);
*/
      
      ros::spinOnce();
      loop.sleep();
    }
}


