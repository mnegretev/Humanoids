#include "ros/ros.h" 
#include "std_msgs/Float32.h"
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

uint16_t goal_position;
bool new_goal_position = false;

float conv2 (float b)
{
  float radbit = 0.0; //Variable para conversión rad a bit
  radbit = 651.739492*b+2047.5; //Conversión de rad a bit
  radbit = round(radbit); //Redondea el valor de bit
  uint16_t entero = 0; //Variable de la posición deseada
  entero = int(radbit); //Convierte al flotante en entero
  return entero;
}

void callback_goal_pose(const std_msgs::Float32::ConstPtr& msg)
{
    std::cout << "LegLeft.->New goal position: " << msg->data << std::endl;
    goal_position = conv2(msg->data);
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
    ros::Subscriber subGoalPose = n.subscribe("goal_pose", 1, callback_goal_pose);

    
	std::cout << "LegLeft.->Serial port successfully openned" << std::endl;
    
	std::cout << "LegLeft.->Cannot open serial port" << std::endl;
	
	std::cout << "LegLeft.->Baudrate successfully set " << std::endl;


    int problem_counter = 0;
    int total_counter = 0;

    uint16_t dxl_current_pos_sim = 0;

    while(ros::ok())
    {
 
      std::cout << "Current position: " << (int)dxl_current_pos_sim << std::endl;

      joint_state_legs.header.stamp = ros::Time::now();
      joint_state_legs.position[0] = ((int)(dxl_current_pos_sim) - LEG_LEFT_YAW_ZERO)*LEG_LEFT_YAW_CW * (2*M_PI / 4096.0);
      joint_state_legs.position[1] = 0;
      joint_state_legs.position[2] = 0;
      joint_state_legs.position[3] = 0;
      joint_state_legs.position[4] = 0;
      joint_state_legs.position[5] = 0;
      joint_state_legs.position[6] = 0;
      joint_state_legs.position[7] = 0;
      joint_state_legs.position[8] = 0;
      joint_state_legs.position[9] = 0;
      joint_state_legs.position[10] = 0;
      joint_state_legs.position[11] = 0;

      // (moving in a circle with radius=2)
      odom_trans.header.stamp = ros::Time::now();

      // //send the joint state and transform
      joint_pub.publish(joint_state_legs);
      broadcaster.sendTransform(odom_trans);

      if((dxl_current_pos_sim - goal_position) > 0)
      {
      	dxl_current_pos_sim--;
      }

      if((dxl_current_pos_sim - goal_position) < 0)
      {
      	dxl_current_pos_sim++;
      }
       
      if(new_goal_position)
	{
	  new_goal_position = false;
	}
      ros::spinOnce();
      loop.sleep();
    }
}


