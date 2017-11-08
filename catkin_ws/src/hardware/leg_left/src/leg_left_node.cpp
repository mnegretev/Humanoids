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
//#include <conio.h>

#define DEVICE_NAME      "/dev/ttyUSB0" //
#define BAUDRATE         57600
#define PROTOCOL_VERSION 1.0

#define SERVO_MX_RANGE_IN_BITS 4096
#define SERVO_MX_RANGE_IN_RADS 2*M_PI
#define SERVO_MX_BITS_PER_RAD  SERVO_MX_RANGE_IN_BITS/SERVO_MX_RANGE_IN_RADS

#define LEG_LEFT_YAW_ZERO     2048
#define LEG_LEFT_YAW_CW       1

uint16_t goal_position;
bool new_goal_position = false;

float conv (float a) //Convert an angle in radians to a value in bits for a MX servomotor
{
  float bitrad = 0.0; //Variable para conversión bit a rad
  bitrad = 2*M_PI*a/4095 - M_PI; //Conversión de bit a rad
  return bitrad;
}

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
    sensor_msgs::JointState joint_state;
   
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "trunk";
 
    //odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = 0.0;//cos(angle)*2;
    odom_trans.transform.translation.y = 0.0;//sin(angle)*2;
    odom_trans.transform.translation.z = 0.0;//.7;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
  
    joint_state.name.resize(20);
    joint_state.position.resize(20);
  
    joint_state.name[0] ="neck_joint_pitch";
    joint_state.name[1] ="neck_joint_yaw";
  
    joint_state.name[2] ="left_joint_leg_yaw";
    joint_state.name[3] ="left_joint_leg_pitch";
    joint_state.name[4] ="left_joint_leg_roll";
    joint_state.name[5] ="left_joint_knee_pitch";
    joint_state.name[6] ="left_joint_ankle_pitch";
    joint_state.name[7] ="left_joint_ankle_roll";
  
    joint_state.name[8] ="right_joint_leg_yaw";
    joint_state.name[9] ="right_joint_leg_pitch";
    joint_state.name[10] ="right_joint_leg_roll";
    joint_state.name[11] ="right_joint_knee_pitch";
    joint_state.name[12] ="right_joint_ankle_pitch";
    joint_state.name[13] ="right_joint_ankle_roll";
 
    joint_state.name[14] ="left_joint_shoulder_pitch";
    joint_state.name[15] ="left_joint_shoulder_roll";
    joint_state.name[16] ="left_joint_elbow_pitch";
 
    joint_state.name[17] ="right_joint_shoulder_pitch";
    joint_state.name[18] ="right_joint_shoulder_roll";
    joint_state.name[19] ="right_joint_elbow_pitch";

    std::cout << "INITIALIZING LEFT LEG NODE BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "leg_left");
    ros::NodeHandle n;
    ros::Rate loop(10);

    ros::Subscriber subGoalPose = n.subscribe("goal_pose", 1, callback_goal_pose);

    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if(portHandler->openPort())
	std::cout << "LegLeft.->Serial port successfully openned" << std::endl;
    else
    {
	std::cout << "LegLeft.->Cannot open serial port" << std::endl;
	return -1;
    }
    if(portHandler->setBaudRate(BAUDRATE))
	std::cout << "LegLeft.->Baudrate successfully set to " << BAUDRATE << std::endl;
    else
    {
	std::cout << "LegLeft.->Cannot set baud rate" << std::endl;
	return -1;
    }

    int problem_counter = 0;
    int total_counter = 0;

    uint16_t dxl_current_pos;
    uint8_t  dxl_error;
    packetHandler->write2ByteTxRx(portHandler, 1, 14, 1023, &dxl_error); //Máximo par
    while(ros::ok())
    {
	
      

      joint_state.header.stamp = ros::Time::now();
      joint_state.position[0] = conv(dxl_current_pos);
      joint_state.position[1] = 0;
      joint_state.position[2] = 0;
      joint_state.position[3] = 0;
      joint_state.position[4] = 0;
      joint_state.position[5] = 0;
      joint_state.position[6] = 0;
      joint_state.position[7] = 0;
      joint_state.position[8] = 0;
      joint_state.position[9] = 0;
      joint_state.position[10] = 0;
      joint_state.position[11] = 0;
      joint_state.position[12] = 0;
      joint_state.position[13] = 0;
      joint_state.position[14] = 0;
      joint_state.position[15] = 0;
      joint_state.position[16] = 0;
      joint_state.position[17] = 0;
      joint_state.position[18] = 0;
      joint_state.position[19] = 0;

      // (moving in a circle with radius=2)
       odom_trans.header.stamp = ros::Time::now();

      // //send the joint state and transform
       joint_pub.publish(joint_state);
       broadcaster.sendTransform(odom_trans);

       
	int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, 1, 36, &dxl_current_pos, &dxl_error);
	if(dxl_comm_result != COMM_SUCCESS)
	    packetHandler->printTxRxResult(dxl_comm_result);
	//  problem_counter++;
	else if (dxl_error != 0)
	    packetHandler->printRxPacketError(dxl_error);
	//problem_counter++;
	std::cout << "Current position: " << (int)dxl_current_pos << std::endl;
	total_counter++;
	//std::cout << "Problem rate: " << problem_counter << " out of " << total_counter << std::endl;
	if(new_goal_position)
	{
	    new_goal_position = false;
	    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, 24, 1, &dxl_error); //Habilita par
	    packetHandler->write2ByteTxRx(portHandler, 1, 30, goal_position, &dxl_error); //Escribe la posición deseada
	}
	ros::spinOnce();
	loop.sleep();
    }
}
