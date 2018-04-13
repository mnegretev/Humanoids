#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16MultiArray.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"

#define ID_LEG_LEFT_HIP_YAW               8
#define ID_LEG_LEFT_HIP_ROLL             10
#define ID_LEG_LEFT_HIP_PITCH            12
#define ID_LEG_LEFT_KNEE_PITCH           14
#define ID_LEG_LEFT_ANKLE_PITCH          16
#define ID_LEG_LEFT_ANKLE_ROLL           18

#define ID_LEG_RIGHT_HIP_YAW              7
#define ID_LEG_RIGHT_HIP_ROLL             9
#define ID_LEG_RIGHT_HIP_PITCH           11
#define ID_LEG_RIGHT_KNEE_PITCH          13
#define ID_LEG_RIGHT_ANKLE_PITCH         15
#define ID_LEG_RIGHT_ANKLE_ROLL          17

#define ID_ARM_LEFT_SHOULDER_PITCH        2
#define ID_ARM_LEFT_SHOULDER_ROLL         4
#define ID_ARM_LEFT_ELBOW_PITCH           6

#define ID_ARM_RIGHT_SHOULDER_PITCH       1
#define ID_ARM_RIGHT_SHOULDER_ROLL        3
#define ID_ARM_RIGHT_ELBOW_PITCH          5

#define ID_NECK_YAW                       19
#define ID_HEAD_PITCH                     20



#define ZERO_LEG_LEFT_HIP_YAW           2048
#define ZERO_LEG_LEFT_HIP_ROLL          2048
#define ZERO_LEG_LEFT_HIP_PITCH         2048
#define ZERO_LEG_LEFT_KNEE_PITCH        1180
#define ZERO_LEG_LEFT_ANKLE_PITCH       2048
#define ZERO_LEG_LEFT_ANKLE_ROLL        2048

#define ZERO_LEG_RIGHT_HIP_YAW          2048
#define ZERO_LEG_RIGHT_HIP_ROLL         2048
#define ZERO_LEG_RIGHT_HIP_PITCH        2048
#define ZERO_LEG_RIGHT_KNEE_PITCH       2890
#define ZERO_LEG_RIGHT_ANKLE_PITCH      2048
#define ZERO_LEG_RIGHT_ANKLE_ROLL       2048

#define ZERO_ARM_LEFT_SHOULDER_PITCH    2048
#define ZERO_ARM_LEFT_SHOULDER_ROLL     2540
#define ZERO_ARM_LEFT_ELBOW_PITCH       2048

#define ZERO_ARM_RIGHT_SHOULDER_PITCH   1905
#define ZERO_ARM_RIGHT_SHOULDER_ROLL    1407
#define ZERO_ARM_RIGHT_ELBOW_PITCH      2048

#define ZERO_NECK_YAW                   2048
#define ZERO_HEAD_PITCH                 2048



#define CW_LEG_LEFT_HIP_YAW               -1
#define CW_LEG_LEFT_HIP_ROLL               1
#define CW_LEG_LEFT_HIP_PITCH              1
#define CW_LEG_LEFT_KNEE_PITCH             1
#define CW_LEG_LEFT_ANKLE_PITCH           -1
#define CW_LEG_LEFT_ANKLE_ROLL            -1
				         	 
#define CW_LEG_RIGHT_HIP_YAW              -1
#define CW_LEG_RIGHT_HIP_ROLL              1
#define CW_LEG_RIGHT_HIP_PITCH            -1
#define CW_LEG_RIGHT_KNEE_PITCH           -1
#define CW_LEG_RIGHT_ANKLE_PITCH           1
#define CW_LEG_RIGHT_ANKLE_ROLL           -1
				         	 
#define CW_ARM_LEFT_SHOULDER_PITCH         1
#define CW_ARM_LEFT_SHOULDER_ROLL         -1
#define CW_ARM_LEFT_ELBOW_PITCH           -1
				         	 
#define CW_ARM_RIGHT_SHOULDER_PITCH       -1
#define CW_ARM_RIGHT_SHOULDER_ROLL        -1
#define CW_ARM_RIGHT_ELBOW_PITCH           1
				      
#define CW_NECK_YAW                        1
#define CW_HEAD_PITCH                     -1


#define SERVO_MX_BITS_PER_RAD  651.898646904
#define SERVO_MX_RADS_PER_BIT    0.001533981
#define SERVO_PROTOCOL_VERSION           1.0

#define ADDR_MX_CURRENT_POSITION         36
#define ADDR_MX_GOAL_POSITION            30


uint16_t leg_left_ids[6] =
{
    ID_LEG_LEFT_HIP_YAW    ,
    ID_LEG_LEFT_HIP_ROLL   , 
    ID_LEG_LEFT_HIP_PITCH  , 
    ID_LEG_LEFT_KNEE_PITCH ,
    ID_LEG_LEFT_ANKLE_PITCH,
    ID_LEG_LEFT_ANKLE_ROLL ,
};
uint16_t leg_right_ids[6]=
{
    ID_LEG_RIGHT_HIP_YAW    ,
    ID_LEG_RIGHT_HIP_ROLL   , 
    ID_LEG_RIGHT_HIP_PITCH  , 
    ID_LEG_RIGHT_KNEE_PITCH ,
    ID_LEG_RIGHT_ANKLE_PITCH,
    ID_LEG_RIGHT_ANKLE_ROLL ,
};
uint16_t arm_left_ids[3]=
{
    ID_ARM_LEFT_SHOULDER_PITCH ,
    ID_ARM_LEFT_SHOULDER_ROLL  ,
    ID_ARM_LEFT_ELBOW_PITCH    ,
};
uint16_t arm_right_ids[3]=
{
    ID_ARM_RIGHT_SHOULDER_PITCH ,
    ID_ARM_RIGHT_SHOULDER_ROLL  ,
    ID_ARM_RIGHT_ELBOW_PITCH    ,
};
uint16_t head_ids[2]=
{
    ID_NECK_YAW  ,
    ID_HEAD_PITCH,
};

uint16_t servos_ids[20] =
{
    ID_LEG_LEFT_HIP_YAW    ,
    ID_LEG_LEFT_HIP_ROLL   , 
    ID_LEG_LEFT_HIP_PITCH  , 
    ID_LEG_LEFT_KNEE_PITCH ,
    ID_LEG_LEFT_ANKLE_PITCH,
    ID_LEG_LEFT_ANKLE_ROLL ,
    ID_LEG_RIGHT_HIP_YAW    ,
    ID_LEG_RIGHT_HIP_ROLL   , 
    ID_LEG_RIGHT_HIP_PITCH  , 
    ID_LEG_RIGHT_KNEE_PITCH ,
    ID_LEG_RIGHT_ANKLE_PITCH,
    ID_LEG_RIGHT_ANKLE_ROLL ,
    ID_ARM_LEFT_SHOULDER_PITCH ,
    ID_ARM_LEFT_SHOULDER_ROLL  ,
    ID_ARM_LEFT_ELBOW_PITCH    ,
    ID_ARM_RIGHT_SHOULDER_PITCH ,
    ID_ARM_RIGHT_SHOULDER_ROLL  ,
    ID_ARM_RIGHT_ELBOW_PITCH    ,
    ID_NECK_YAW  ,
    ID_HEAD_PITCH,
};


uint16_t leg_left_position_zero_bits[6] =
{
    ZERO_LEG_LEFT_HIP_YAW    ,
    ZERO_LEG_LEFT_HIP_ROLL   , 
    ZERO_LEG_LEFT_HIP_PITCH  , 
    ZERO_LEG_LEFT_KNEE_PITCH ,
    ZERO_LEG_LEFT_ANKLE_PITCH,
    ZERO_LEG_LEFT_ANKLE_ROLL ,
};
uint16_t leg_right_position_zero_bits[6] =
{
    ZERO_LEG_RIGHT_HIP_YAW    ,
    ZERO_LEG_RIGHT_HIP_ROLL   , 
    ZERO_LEG_RIGHT_HIP_PITCH  , 
    ZERO_LEG_RIGHT_KNEE_PITCH ,
    ZERO_LEG_RIGHT_ANKLE_PITCH,
    ZERO_LEG_RIGHT_ANKLE_ROLL ,
};
uint16_t arm_left_position_zero_bits[3]=
{
    ZERO_ARM_LEFT_SHOULDER_PITCH ,
    ZERO_ARM_LEFT_SHOULDER_ROLL  ,
    ZERO_ARM_LEFT_ELBOW_PITCH    ,
};
uint16_t arm_right_position_zero_bits[3]=
{
    ZERO_ARM_RIGHT_SHOULDER_PITCH ,
    ZERO_ARM_RIGHT_SHOULDER_ROLL  ,
    ZERO_ARM_RIGHT_ELBOW_PITCH    ,
};
uint16_t head_position_zero_bits[2]=
{
    ZERO_NECK_YAW  ,
    ZERO_HEAD_PITCH,
};

uint16_t servos_position_zero[20] =
{
    ZERO_LEG_LEFT_HIP_YAW    ,
    ZERO_LEG_LEFT_HIP_ROLL   , 
    ZERO_LEG_LEFT_HIP_PITCH  , 
    ZERO_LEG_LEFT_KNEE_PITCH ,
    ZERO_LEG_LEFT_ANKLE_PITCH,
    ZERO_LEG_LEFT_ANKLE_ROLL ,
    ZERO_LEG_RIGHT_HIP_YAW    ,
    ZERO_LEG_RIGHT_HIP_ROLL   , 
    ZERO_LEG_RIGHT_HIP_PITCH  , 
    ZERO_LEG_RIGHT_KNEE_PITCH ,
    ZERO_LEG_RIGHT_ANKLE_PITCH,
    ZERO_LEG_RIGHT_ANKLE_ROLL ,
    ZERO_ARM_LEFT_SHOULDER_PITCH ,
    ZERO_ARM_LEFT_SHOULDER_ROLL  ,
    ZERO_ARM_LEFT_ELBOW_PITCH    ,
    ZERO_ARM_RIGHT_SHOULDER_PITCH ,
    ZERO_ARM_RIGHT_SHOULDER_ROLL  ,
    ZERO_ARM_RIGHT_ELBOW_PITCH    ,
    ZERO_NECK_YAW  ,
    ZERO_HEAD_PITCH,
};


uint16_t leg_left_is_clockwise[6] =
{
    CW_LEG_LEFT_HIP_YAW    ,
    CW_LEG_LEFT_HIP_ROLL   , 
    CW_LEG_LEFT_HIP_PITCH  , 
    CW_LEG_LEFT_KNEE_PITCH ,
    CW_LEG_LEFT_ANKLE_PITCH,
    CW_LEG_LEFT_ANKLE_ROLL ,
};
uint16_t leg_right_is_clockwise[6]=
{
    CW_LEG_RIGHT_HIP_YAW    ,
    CW_LEG_RIGHT_HIP_ROLL   , 
    CW_LEG_RIGHT_HIP_PITCH  , 
    CW_LEG_RIGHT_KNEE_PITCH ,
    CW_LEG_RIGHT_ANKLE_PITCH,
    CW_LEG_RIGHT_ANKLE_ROLL ,
};
uint16_t arm_left_is_clockwise[3]=
{
    CW_ARM_LEFT_SHOULDER_PITCH ,
    CW_ARM_LEFT_SHOULDER_ROLL  ,
    CW_ARM_LEFT_ELBOW_PITCH    ,
};
uint16_t arm_right_is_clockwise[3]=
{
    CW_ARM_RIGHT_SHOULDER_PITCH ,
    CW_ARM_RIGHT_SHOULDER_ROLL  ,
    CW_ARM_RIGHT_ELBOW_PITCH    ,
};
uint16_t head_is_clockwise[2]=
{
    CW_NECK_YAW  ,
    CW_HEAD_PITCH,
};

uint16_t servos_is_clockwise[20] =
{
    CW_LEG_LEFT_HIP_YAW    ,
    CW_LEG_LEFT_HIP_ROLL   , 
    CW_LEG_LEFT_HIP_PITCH  , 
    CW_LEG_LEFT_KNEE_PITCH ,
    CW_LEG_LEFT_ANKLE_PITCH,
    CW_LEG_LEFT_ANKLE_ROLL ,
    CW_LEG_RIGHT_HIP_YAW    ,
    CW_LEG_RIGHT_HIP_ROLL   , 
    CW_LEG_RIGHT_HIP_PITCH  , 
    CW_LEG_RIGHT_KNEE_PITCH ,
    CW_LEG_RIGHT_ANKLE_PITCH,
    CW_LEG_RIGHT_ANKLE_ROLL ,
    CW_ARM_LEFT_SHOULDER_PITCH ,
    CW_ARM_LEFT_SHOULDER_ROLL  ,
    CW_ARM_LEFT_ELBOW_PITCH    ,
    CW_ARM_RIGHT_SHOULDER_PITCH ,
    CW_ARM_RIGHT_SHOULDER_ROLL  ,
    CW_ARM_RIGHT_ELBOW_PITCH    ,
    CW_NECK_YAW  ,
    CW_HEAD_PITCH,
};



int main(int argc, char** argv)
{     
    std::cout << "INITIALIZING CM730 NODE BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "cm730");
    ros::NodeHandle n;
    ros::Rate loop(30);
/*
    ros::Subscriber subLegsGoalPose     = n.subscribe("legs_goal_pose", 1, callback_goal_pose);
    ros::Subscriber subLegLeftGoalPose  = n.subscribe("leg_left_goal_pose", 1, callback_goal_pose_left);
    ros::Subscriber subLegRightGoalPose = n.subscribe("leg_right_goal_pose", 1, callback_goal_pose_right);
    ros::Subscriber subLegsGoalTorque   = n.subscribe("legs_goal_torque", 1, callback_goal_torque);
    ros::Subscriber subLegLeftGoalTorque   = n.subscribe("legs_left_goal_torque", 1, callback_goal_torque_left);
    ros::Subscriber subLegRightGoalTorque   = n.subscribe("legs_right_goal_torque", 1, callback_goal_torque_right);*/
    
    ros::Publisher  pub_joint_states = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    sensor_msgs::JointState msg_joint_states;
  
    msg_joint_states.name.resize(20);
    msg_joint_states.position.resize(20);
  
    msg_joint_states.name[0]  ="left_hip_yaw";   
    msg_joint_states.name[1]  ="left_hip_roll";  
    msg_joint_states.name[2]  ="left_hip_pitch"; 
    msg_joint_states.name[3]  ="left_knee_pitch";
    msg_joint_states.name[4]  ="left_ankle_pitch";
    msg_joint_states.name[5]  ="left_ankle_roll";
  
    msg_joint_states.name[6]  ="right_hip_yaw";
    msg_joint_states.name[7]  ="right_hip_roll";
    msg_joint_states.name[8]  ="right_hip_pitch";
    msg_joint_states.name[9]  ="right_knee_pitch";
    msg_joint_states.name[10] ="right_ankle_pitch";
    msg_joint_states.name[11] ="right_ankle_roll";

    msg_joint_states.name[12] ="left_shoulder_pitch";  
    msg_joint_states.name[13] ="left_shoulder_roll";   
    msg_joint_states.name[14] ="left_elbow_pitch";    
    			                              
    msg_joint_states.name[15] ="right_shoulder_pitch"; 
    msg_joint_states.name[16] ="right_shoulder_roll";  
    msg_joint_states.name[17] ="right_elbow_pitch";   

    msg_joint_states.name[18] ="neck_yaw";   
    msg_joint_states.name[19] ="head_pitch";
    
    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(SERVO_PROTOCOL_VERSION);

    if(portHandler->openPort())
	std::cout << "CM730.->Serial port successfully openned on port " << std::endl;
    else
    {
	std::cout << "CM730.->Cannot open serial port" << std::endl;
	return -1;
    }
    
    if(portHandler->setBaudRate(1000000))
	std::cout << "CM730.->Baudrate successfully set to " << std::endl;
    else
    {
	std::cout << "CM730.->Cannot set baud rate" << std::endl;
	return -1;
    }

    uint8_t  dxl_error       = 0;
    int      dxl_comm_result = COMM_TX_FAIL;
    uint16_t dxl_current_pos;

    while(ros::ok())
    {
	for(int i = 0; i < 20; i++)
	{
	    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, servos_ids[i], ADDR_MX_CURRENT_POSITION,
							   &dxl_current_pos, &dxl_error);
	    if(dxl_comm_result == COMM_SUCCESS && dxl_error == 0)
		msg_joint_states.position[i] = (dxl_current_pos - servos_position_zero[i])*SERVO_MX_RADS_PER_BIT *
		    servos_is_clockwise[i];
	    else if(dxl_comm_result != COMM_SUCCESS)
		std::cout << "CM730.->Communication error while trying to read servo id " << servos_ids[i] << std::endl;
	    else
		std::cout << "CM730.->Warning! Error in servo id "<< servos_ids[i] << " with code "<< int(dxl_error) << std::endl;
	}

	msg_joint_states.header.stamp = ros::Time::now();
	pub_joint_states.publish(msg_joint_states);
	
	ros::spinOnce();
	loop.sleep();
    }

    portHandler->closePort();
}
