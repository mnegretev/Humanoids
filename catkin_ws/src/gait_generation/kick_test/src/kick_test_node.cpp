#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "kick_test/kick.h"


using namespace std;



void allocate_publishers(vector<string> &joint_name , vector<float> &joint_position, ros::Publisher& pubHeadPositions,
						 ros::Publisher& pubLegsPositions, ros::Publisher& pubLeftArmPositions, ros::Publisher& pubRightArmPositions)
{
	std_msgs::Float32MultiArray head_msg;
	std_msgs::Float32MultiArray legs_msg;
	std_msgs::Float32MultiArray left_msg;
	std_msgs::Float32MultiArray right_msg;

	head_msg.data.resize(2);
	legs_msg.data.resize(12); 
	left_msg.data.resize(3);
	right_msg.data.resize(3);

	for (int index = 0; index < 20; ++index)
	{
		if (joint_name[index] == "neck_yaw")
                head_msg.data[0] = joint_position[index];
		if (joint_name[index] == "head_pitch")
                head_msg.data[1] = joint_position[index];

		if (joint_name[index] == "left_shoulder_pitch")
            left_msg.data[0] = joint_position[index];
		if (joint_name[index] == "left_shoulder_roll")
            left_msg.data[1] = joint_position[index];
		if (joint_name[index] == "left_elbow_pitch")
            left_msg.data[2] = joint_position[index];

		if (joint_name[index] == "right_shoulder_pitch")
            right_msg.data[0] = joint_position[index];
		if (joint_name[index] == "right_shoulder_roll")
            right_msg.data[1] = joint_position[index];
		if (joint_name[index] == "right_elbow_pitch")
            right_msg.data[2] = joint_position[index];

		if (joint_name[index] == "left_hip_yaw")
			legs_msg.data[0] = joint_position[index];
		if (joint_name[index] == "left_hip_roll")
			legs_msg.data[1] = joint_position[index];
		if (joint_name[index] == "left_hip_pitch")
			legs_msg.data[2] = joint_position[index];
		if (joint_name[index] == "left_knee_pitch")
			legs_msg.data[3] = joint_position[index];
		if (joint_name[index] == "left_ankle_pitch")
			legs_msg.data[4] = joint_position[index];
		if (joint_name[index] == "left_ankle_roll")
			legs_msg.data[5] = joint_position[index];
		if (joint_name[index] == "right_hip_yaw")
			legs_msg.data[6] = joint_position[index];
		if (joint_name[index] == "right_hip_roll")
			legs_msg.data[7] = joint_position[index];
		if (joint_name[index] == "right_hip_pitch")
			legs_msg.data[8] = joint_position[index];
		if (joint_name[index] == "right_knee_pitch")
			legs_msg.data[9] = joint_position[index];
		if (joint_name[index] == "right_ankle_pitch")
			legs_msg.data[10] = joint_position[index];
		if (joint_name[index] == "right_ankle_roll")
			legs_msg.data[11] = joint_position[index];
		}

		pubHeadPositions.publish(head_msg);
		pubLegsPositions.publish(legs_msg);
		pubLeftArmPositions.publish(left_msg);
		pubRightArmPositions.publish(right_msg);
}



int main(int argc, char **argv)
{
	cout<<"Starting kick_test_node by Luis Näva....."<<endl;
	ros::init(argc, argv, "kick_test_node");
	ros::NodeHandle nh;

	ros::ServiceClient client = nh.serviceClient<kick_test::kick>("/joints_states_profiles");
	ros::Publisher pubHeadPositions = nh.advertise<std_msgs::Float32MultiArray>("/harware/head_goal_pose",10);
	ros::Publisher pubLegsPositions = nh.advertise<std_msgs::Float32MultiArray>("/hardware/legs_goal_pose", 10);
	ros::Publisher pubLeftArmPositions  = nh.advertise<std_msgs::Float32MultiArray>("/hardware/arm_left_goal_pose",  10);
	ros::Publisher pubRightArmPositions = nh.advertise<std_msgs::Float32MultiArray>("/hardware/arm_right_goal_pose", 10);
	ros::Rate loop(0.5);	
	
	kick_test::kick srv;

	if( argc == 2)
		srv.request.kick_mode = argv[1];
	else
		srv.request.kick_mode = "left";

	vector<string> joint_name;
	vector<float>  joint_position;
	joint_name.resize(20);
	joint_position.resize(20);

	int robot_state = 0, states_number;
	float delay;

	do
	{
		srv.request.robot_state = robot_state;

		if(client.call(srv))
		{
			cout<<"Robot state: "<<srv.request.robot_state<<endl;
			joint_name = srv.response.name;
			joint_position = srv.response.position;
			states_number = srv.response.states;
			delay = srv.response.delay;
		}
		else
		{ 
			ROS_ERROR("Failed to call service kick_server.py");
			return 1;
		}		

		allocate_publishers(joint_name, joint_position, pubHeadPositions, pubLegsPositions, pubLeftArmPositions, pubRightArmPositions);
		
		robot_state++;
		loop.sleep();
		//ros::spinOnce();
	}while(robot_state < states_number);

	return 0;
}
