#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#define DEVICE_NAME      "/dev/ttyUSB0" //
#define BAUDRATE         57600
#define PROTOCOL_VERSION 1.0

uint16_t goal_position;
bool new_goal_position = false;

void callback_goal_pose(const std_msgs::Float32::ConstPtr& msg)
{
    std::cout << "LegLeft.->New goal position: " << msg->data << std::endl;
    goal_position = (uint16_t)msg->data;
    new_goal_position = true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LEFT LEG NODE BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "leg_left");
    ros::NodeHandle n;
    ros::Rate loop(30);

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
    while(ros::ok())
    {
	
       
	uint16_t dxl_current_pos;
	uint8_t  dxl_error;
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
	    packetHandler->write2ByteTxRx(portHandler, 1, 30, goal_position, &dxl_error);
	}
	ros::spinOnce();
	loop.sleep();
    }
}
