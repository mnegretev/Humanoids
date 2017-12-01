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

uint16_t goal_position_0;
uint16_t goal_position_1;
uint16_t goal_position_2;
uint16_t goal_position_3;
uint16_t goal_position_4;
uint16_t goal_position_5;
uint16_t goal_position_6;
uint16_t goal_position_7;
uint16_t goal_position_8;
uint16_t goal_position_9;
uint16_t goal_position_10;
uint16_t goal_position_11;
bool new_goal_position = false;

float conv (float b)
{
  float radbit = 0.0; //Variable para conversión rad a bit
  radbit = 651.739492*b+2047.5; //Conversión de rad a bit
  radbit = round(radbit); //Redondea el valor de bit
  uint16_t entero = 0; //Variable de la posición deseada
  entero = int(radbit); //Convierte al flotante en entero
  return entero;
}

void callback_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    goal_position_0 = conv(msg->data[0]);
    goal_position_1 = conv(msg->data[1]);
    goal_position_2 = conv(msg->data[2]);
    goal_position_3 = conv(msg->data[3]);
    goal_position_4 = conv(msg->data[4]);
    goal_position_5 = conv(msg->data[5]);
    goal_position_6 = conv(msg->data[6]);
    goal_position_7 = conv(msg->data[7]);
    goal_position_8 = conv(msg->data[8]);
    goal_position_9 = conv(msg->data[9]);
    goal_position_10= conv(msg->data[10]);
    goal_position_11 = conv(msg->data[11]);
    
    new_goal_position = true;
}

int main(int argc, char** argv)
 {     
    //Inicializamos el publicador de simulación
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n1;
    ros::Publisher joint_pub = n1.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state_legs;
   
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "trunk";
 
    //odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = 0.0;//cos(angle)*2;
    odom_trans.transform.translation.y = 0.0;//sin(angle)*2;
    odom_trans.transform.translation.z = 0.0;//.7;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
  
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
 

    std::cout << "INITIALIZING LEFT LEG NODE BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "legs");
    ros::NodeHandle n;
    ros::Rate loop(1000);
    ros::Subscriber subGoalPose = n.subscribe("goal_pose_legs", 1, callback_goal_pose);

    
	std::cout << "LegLeft.->Serial port successfully openned" << std::endl;
	std::cout << "LegLeft.->Baudrate successfully set " << std::endl;


    int problem_counter = 0;
    int total_counter = 0;

    uint16_t dxl_current_pos_sim_0 = 0;
    uint16_t dxl_current_pos_sim_1 = 0;
    uint16_t dxl_current_pos_sim_2 = 0;
    uint16_t dxl_current_pos_sim_3 = 0;
    uint16_t dxl_current_pos_sim_4 = 0;
    uint16_t dxl_current_pos_sim_5 = 0;
    uint16_t dxl_current_pos_sim_6 = 0;
    uint16_t dxl_current_pos_sim_7 = 0;
    uint16_t dxl_current_pos_sim_8 = 0;
    uint16_t dxl_current_pos_sim_9 = 0;
    uint16_t dxl_current_pos_sim_10 = 0;
    uint16_t dxl_current_pos_sim_11 = 0;


    while(ros::ok())
    {
 
      std::cout << "Current position 0: " << (int)dxl_current_pos_sim_0 << std::endl;
      std::cout << "Current position 1: " << (int)dxl_current_pos_sim_1 << std::endl;
      std::cout << "Current position 2: " << (int)dxl_current_pos_sim_2 << std::endl;
      std::cout << "Current position 3: " << (int)dxl_current_pos_sim_3 << std::endl;
      std::cout << "Current position 4: " << (int)dxl_current_pos_sim_4 << std::endl;
      std::cout << "Current position 5: " << (int)dxl_current_pos_sim_5 << std::endl;
      std::cout << "Current position 6: " << (int)dxl_current_pos_sim_6 << std::endl;
      std::cout << "Current position 7: " << (int)dxl_current_pos_sim_7 << std::endl;
      std::cout << "Current position 8: " << (int)dxl_current_pos_sim_8 << std::endl;
      std::cout << "Current position 9: " << (int)dxl_current_pos_sim_9 << std::endl;
      std::cout << "Current position 10: " << (int)dxl_current_pos_sim_10 << std::endl;
      std::cout << "Current position 11: " << (int)dxl_current_pos_sim_11 << std::endl;


      joint_state_legs.header.stamp = ros::Time::now();
      joint_state_legs.position[0] = ((int)(dxl_current_pos_sim_0) - LEG_LEFT_YAW_ZERO)*LEG_LEFT_YAW_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[1] = ((int)(dxl_current_pos_sim_1) - LEG_LEFT_PITCH_ZERO)*LEG_LEFT_PITCH_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[2] = ((int)(dxl_current_pos_sim_2) - LEG_LEFT_ROLL_ZERO)*LEG_LEFT_ROLL_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[3] = ((int)(dxl_current_pos_sim_3) - LEG_LEFT_KNEE_PITCH_ZERO)*LEG_LEFT_KNEE_PITCH_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[4] = ((int)(dxl_current_pos_sim_4) - LEG_LEFT_ANKLE_PITCH_ZERO)*LEG_LEFT_ANKLE_PITCH_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[5] = ((int)(dxl_current_pos_sim_5) - LEG_LEFT_ANKLE_ROLL_ZERO)*LEG_LEFT_ANKLE_ROLL_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[6] = ((int)(dxl_current_pos_sim_6) - LEG_RIGHT_YAW_ZERO)*LEG_RIGHT_YAW_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[7] = ((int)(dxl_current_pos_sim_7) - LEG_RIGHT_PITCH_ZERO)*LEG_RIGHT_PITCH_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[8] = ((int)(dxl_current_pos_sim_8) - LEG_RIGHT_ROLL_ZERO)*LEG_RIGHT_ROLL_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[9] = ((int)(dxl_current_pos_sim_9) - LEG_RIGHT_KNEE_PITCH_ZERO)*LEG_RIGHT_KNEE_PITCH_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[10] = ((int)(dxl_current_pos_sim_10) - LEG_RIGHT_ANKLE_PITCH_ZERO)*LEG_RIGHT_ANKLE_PITCH_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[11] = ((int)(dxl_current_pos_sim_11) - LEG_RIGHT_ANKLE_ROLL_ZERO)*LEG_RIGHT_ANKLE_ROLL_CW * (2*M_PI / 4096.0);


      // (moving in a circle with radius=2)
      odom_trans.header.stamp = ros::Time::now();

      // //send the joint state and transform
      joint_pub.publish(joint_state_legs);
      broadcaster.sendTransform(odom_trans);

      // 0
      if((dxl_current_pos_sim_0 - goal_position_0) > 0)
      {
      	dxl_current_pos_sim_0--;
      }

      if((dxl_current_pos_sim_0 - goal_position_0) < 0)
      {
      	dxl_current_pos_sim_0++;
      }

      // 1
      if((dxl_current_pos_sim_1 - goal_position_1) > 0)
      {
      	dxl_current_pos_sim_1--;
      }

      if((dxl_current_pos_sim_1 - goal_position_1) < 0)
      {
      	dxl_current_pos_sim_1++;
      }

      // 2
      if((dxl_current_pos_sim_2 - goal_position_2) > 0)
      {
      	dxl_current_pos_sim_2--;
      }

      if((dxl_current_pos_sim_2 - goal_position_2) < 0)
      {
      	dxl_current_pos_sim_2++;
      }

      // 3
      if((dxl_current_pos_sim_3 - goal_position_3) > 0)
      {
      	dxl_current_pos_sim_3--;
      }

      if((dxl_current_pos_sim_3 - goal_position_3) < 0)
      {
      	dxl_current_pos_sim_3++;
      }

      // 4
      if((dxl_current_pos_sim_4 - goal_position_4) > 0)
      {
      	dxl_current_pos_sim_4--;
      }

      if((dxl_current_pos_sim_4 - goal_position_4) < 0)
      {
      	dxl_current_pos_sim_4++;
      }

      // 5
      if((dxl_current_pos_sim_5 - goal_position_5) > 0)
      {
      	dxl_current_pos_sim_5--;
      }

      if((dxl_current_pos_sim_5 - goal_position_5) < 0)
      {
      	dxl_current_pos_sim_5++;
      }

      // 6
      if((dxl_current_pos_sim_6 - goal_position_6) > 0)
      {
      	dxl_current_pos_sim_6--;
      }

      if((dxl_current_pos_sim_6 - goal_position_6) < 0)
      {
      	dxl_current_pos_sim_6++;
      }

      // 7
      if((dxl_current_pos_sim_7 - goal_position_7) > 0)
      {
      	dxl_current_pos_sim_7--;
      }

      if((dxl_current_pos_sim_7 - goal_position_7) < 0)
      {
      	dxl_current_pos_sim_7++;
      }

      // 8
      if((dxl_current_pos_sim_8 - goal_position_8) > 0)
      {
      	dxl_current_pos_sim_8--;
      }

      if((dxl_current_pos_sim_8 - goal_position_8) < 0)
      {
      	dxl_current_pos_sim_8++;
      }

      // 9
      if((dxl_current_pos_sim_9 - goal_position_9) > 0)
      {
      	dxl_current_pos_sim_9--;
      }

      if((dxl_current_pos_sim_9 - goal_position_9) < 0)
      {
      	dxl_current_pos_sim_9++;
      }

      // 10
      if((dxl_current_pos_sim_10 - goal_position_10) > 0)
      {
      	dxl_current_pos_sim_10--;
      }

      if((dxl_current_pos_sim_10 - goal_position_10) < 0)
      {
      	dxl_current_pos_sim_10++;
      }

      // 11
      if((dxl_current_pos_sim_11 - goal_position_11) > 0)
      {
      	dxl_current_pos_sim_11--;
      }

      if((dxl_current_pos_sim_11 - goal_position_11) < 0)
      {
      	dxl_current_pos_sim_11++;
      }

      
       
      if(new_goal_position)
	{
	  new_goal_position = false;
	}
      ros::spinOnce();
      loop.sleep();
    }
}


