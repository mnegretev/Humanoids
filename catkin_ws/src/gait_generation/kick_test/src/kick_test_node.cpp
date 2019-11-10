#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "kick_test/getPose.h"
#include "kick_test/speedProfile.h"

#define sampling_freq 50

#define SM_GET_PREFEF_POSE        0
#define SM_GETTING_CURRENT_POSE  10
#define SM_COMPUTE_PROFILES      20
#define SM_PUBLISH_PROFILES      30
#define SM_FINISH_TEST           40
#define SM_JUST_PUBLISH          50

using namespace std;


int state = SM_GET_PREFEF_POSE;
bool proceed_to_compute = false;

vector<float> current_head_position;
vector<float> current_legs_position;
vector<float> current_left_arm_position;
vector<float> current_right_arm_position;

bool compute_profiles(ros::ServiceClient& clt_speed_profile, vector<float>& head_goal_position, vector<float>& legs_goal_position, 
	                  vector<float>& left_arm_goal_position,  vector<float>& right_arm_goal_position, vector<vector<float> >& head_positions, 
	                  vector<vector<float> >& legs_positions, vector<vector<float> >& left_arm_positions, vector<vector<float> >& right_arm_positions)
{
	kick_test::speedProfile srv;
	
	srv.request.dt = 1.0/sampling_freq;
    srv.request.tf = 0.7;

    head_positions.resize(2);
 	legs_positions.resize(12);
    left_arm_positions.resize(3);
    right_arm_positions.resize(3);


	for(int id=0; id < 20; id++)
	{
		if(id < 12){
			srv.request.p0 = current_legs_position[id];
			srv.request.pf = legs_goal_position[id];

			if(clt_speed_profile.call(srv))
				legs_positions[id] = srv.response.profiled_positions.data;
			else
				return false;
		}
		
		if(id >= 12 && id < 15){
			srv.request.p0 = current_left_arm_position[id-12];
			srv.request.pf = left_arm_goal_position[id-12];

			if(clt_speed_profile.call(srv))
				left_arm_positions[id-12] = srv.response.profiled_positions.data;
			else
				return false;
		}
		if(id >= 15 && id < 18){
			srv.request.p0 = current_right_arm_position[id-15];
			srv.request.pf = right_arm_goal_position[id-15];

			if(clt_speed_profile.call(srv))
				right_arm_positions[id-15] = srv.response.profiled_positions.data;
			else
				return false;
		}
		if(id >= 18){
			srv.request.p0 = current_head_position[id-18];
			srv.request.pf = head_goal_position[id-18];

			if(clt_speed_profile.call(srv))
				head_positions[id-18] = srv.response.profiled_positions.data;
			else
				return false;
		}
	}//From for
	return true;
}

void current_positions_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
	current_head_position.resize(2);
	current_legs_position.resize(12);
	current_left_arm_position.resize(3);
	current_right_arm_position.resize(3);

	for(int id=0; id<20; id++)
	{
		if(id < 12)
			current_legs_position[id] = msg->position[id]; 
		if(id >= 12 && id < 15)
			current_left_arm_position[id-12] = msg->position[id];
		if(id >= 15 && id < 18)
			current_right_arm_position[id-15] = msg->position[id];
		if(id >= 18)
			current_head_position[id-18] = msg->position[id];
	}
	
	proceed_to_compute = true;
}

bool get_pose(ros::ServiceClient& clt_get_pose, string &kick_mode, int &robot_pose, int &number_poses,  vector<float>& head_goal_position,  
	vector<float>& legs_goal_position, vector<float>& left_arm_goal_position,  vector<float>& right_arm_goal_position)
{
	kick_test::getPose srv;
	srv.request.kick_mode = kick_mode;
	srv.request.robot_pose = robot_pose;

	head_goal_position.resize(2);
	legs_goal_position.resize(12);
	left_arm_goal_position.resize(3);
	right_arm_goal_position.resize(3);


	if(clt_get_pose.call(srv))
	{
		cout<<"--------------- Robot state: "<<srv.request.robot_pose<<" ---------------"<<endl;

		number_poses = srv.response.number_poses;
		head_goal_position = srv.response.head_position.data;		
		legs_goal_position = srv.response.legs_position.data;		
		left_arm_goal_position = srv.response.left_arm_position.data;		
		right_arm_goal_position = srv.response.right_arm_position.data;		
	}
	else
		return false;	

	return true;
}

int main(int argc, char **argv)
{
	cout<<"Starting kick_test_node by Luis Näva....."<<endl;
	ros::init(argc, argv, "kick_test_node");
	ros::NodeHandle nh;

	ros::ServiceClient clt_get_pose = nh.serviceClient<kick_test::getPose>("/kick_test/get_pose");
	ros::ServiceClient clt_speed_profile = nh.serviceClient<kick_test::speedProfile>("/kick_test/get_speed_profile");
	ros::Publisher pubHeadPositions = nh.advertise<std_msgs::Float32MultiArray>("/hardware/head_goal_pose", 10);
	ros::Publisher pubLegsPositions = nh.advertise<std_msgs::Float32MultiArray>("/hardware/legs_goal_pose", 10);
	ros::Publisher pubLeftArmPositions  = nh.advertise<std_msgs::Float32MultiArray>("/hardware/arm_left_goal_pose",  10);
	ros::Publisher pubRightArmPositions = nh.advertise<std_msgs::Float32MultiArray>("/hardware/arm_right_goal_pose", 10);
	ros::Subscriber subCurrentJointPositions = nh.subscribe("/joint_states", 1, current_positions_callback);
	ros::Rate loop(sampling_freq);	

	//system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");

	string kick_mode;
	if( argc == 2)
		kick_mode = argv[1];
	else
		kick_mode = "left";

	vector<float> head_goal_position;
	vector<float> legs_goal_position;
	vector<float> left_arm_goal_position;
	vector<float> right_arm_goal_position;

	vector<vector<float> > head_positions;
	vector<vector<float> > legs_positions;
	vector<vector<float> > left_arm_positions;
	vector<vector<float> > right_arm_positions;

	int robot_pose = 0;
	int number_poses=1;
	int time_k = 0;

	std_msgs::Float32MultiArray head_msg, legs_msgs, left_arm_msg, right_arm_msg;
	head_msg.data.resize(2);
	legs_msgs.data.resize(12);
	left_arm_msg.data.resize(3);
	right_arm_msg.data.resize(3);
	

	while(ros::ok())
	{

		switch(state)
		{
		case SM_GET_PREFEF_POSE:
			cout<<"SM_GET_PREFEF_POSE"<<endl;
			if(robot_pose < number_poses)
			{
				if(!get_pose(clt_get_pose, kick_mode, robot_pose, number_poses, head_goal_position, legs_goal_position, 
									                                    left_arm_goal_position, right_arm_goal_position))
					ROS_ERROR("Failed to call service get_pose_server.py");
				state = SM_GETTING_CURRENT_POSE;
			}
			else	
				state = SM_FINISH_TEST;
			break;

		case SM_GETTING_CURRENT_POSE:
			cout<<"SM_GETTING_CURRENT_POSE"<<endl;
			if(proceed_to_compute)
				state = SM_COMPUTE_PROFILES;
			break;

		case SM_JUST_PUBLISH:
			cout<<"SM_JUST_PUBLISH"<<endl;
			for(int i=0; i<2; i++)
				head_msg.data[i] = head_goal_position[i];
			for(int i=0;i<12; i++)
				legs_msgs.data[i] = legs_goal_position[i];
			for(int i=0;i<3; i++)
			{
				left_arm_msg.data[i] = left_arm_goal_position[i];
				right_arm_msg.data[i] = right_arm_goal_position[i];
			}

			pubHeadPositions.publish(head_msg);
			pubLegsPositions.publish(legs_msgs);
			pubLeftArmPositions.publish(left_arm_msg);
			pubRightArmPositions.publish(right_arm_msg);
			break; 

		case SM_COMPUTE_PROFILES:
			cout<<"SM_COMPUTE_PROFILES"<<endl;
			time_k = 0;
			proceed_to_compute = false;

			if(!compute_profiles(clt_speed_profile, head_goal_position, legs_goal_position, left_arm_goal_position, right_arm_goal_position,
																	head_positions,	legs_positions, left_arm_positions, right_arm_positions))
				ROS_ERROR("Failed to call service speed_profile_server.py");
			state = SM_PUBLISH_PROFILES;
			break;

		case SM_PUBLISH_PROFILES:
			cout<<"SM_PUBLISH_PROFILES"<<endl;
			if(time_k < head_positions[0].size())
			{
				for(int i=0; i<2; i++){
					head_msg.data[i] = head_positions[i][time_k];
				}
				for(int i=0;i<12; i++)
					legs_msgs.data[i] = legs_positions[i][time_k];
				for(int i=0;i<3; i++)
				{
					left_arm_msg.data[i] = left_arm_positions[i][time_k];
					right_arm_msg.data[i] = right_arm_positions[i][time_k];
				}

				pubHeadPositions.publish(head_msg);
				pubLegsPositions.publish(legs_msgs);
				//pubLeftArmPositions.publish(left_arm_msg);
				//pubRightArmPositions.publish(right_arm_msg);

				time_k++;
			}
			else
			{
				robot_pose++;
				state = SM_GET_PREFEF_POSE;
			}
			break;

			case SM_FINISH_TEST:
				cout<<"SM_FINISH_TEST"<<endl;
				robot_pose = 0;
				time_k=0;
				break;

		default:
			cout<<"The state machine number is not valid"<<endl;
		}

		ros::spinOnce();
		loop.sleep();

	}//From while(ros::ok())

	return 0;
}
