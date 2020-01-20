#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "get_up_test/getPose.h"
#include "get_up_test/speedProfile.h"

#define sampling_freq 60

#define SM_GET_PREFEF_POSE        0
#define SM_COMPUTE_PROFILES      10
#define SM_PUBLISH_PROFILES      20
#define SM_FINISH_TEST           30


using namespace std;


int state = SM_GET_PREFEF_POSE;
float tf;

vector<float> current_joint_position(20, 0);


bool compute_profiles(ros::ServiceClient& clt_speed_profile, vector<float>& joint_goal_position, 
										                  vector<vector<float> >& joint_profile_positions)
{
	get_up_test::speedProfile srv;
	
	srv.request.dt = 1.0/sampling_freq;
    srv.request.tf = tf;

	joint_profile_positions.resize(20);


	for(int id=0; id < 20; id++){
		srv.request.p0 = current_joint_position[id];
		srv.request.pf = joint_goal_position[id];

		if(clt_speed_profile.call(srv))
			joint_profile_positions[id] = srv.response.profiled_positions.data;
			
		else
			return false;

		current_joint_position[id] = joint_goal_position[id];
	}

	return true;
}

bool get_pose(ros::ServiceClient& clt_get_pose, string &get_up_mode, int &robot_pose, 
							 int &number_poses, vector<float>& joint_goal_position)
{
	get_up_test::getPose srv;
	srv.request.get_up_mode = get_up_mode;
	srv.request.robot_pose = robot_pose;

	joint_goal_position.resize(20);
	if(clt_get_pose.call(srv)){
		cout<<"--------------- Robot state: "<<srv.request.robot_pose<<" ---------------"<<endl;
		number_poses = srv.response.number_poses;
		joint_goal_position = srv.response.joint_goal_position.data;
		tf = srv.response.delay;
	}
	else
		return false;	

	return true;
}

int main(int argc, char **argv)
{
	cout<<"Starting get_up_test_node by Luis Näva....."<<endl;
	ros::init(argc, argv, "get_up_test_node");
	ros::NodeHandle nh;

	ros::ServiceClient clt_get_pose = nh.serviceClient<get_up_test::getPose>("/get_up_test/get_pose");
	ros::ServiceClient clt_speed_profile = nh.serviceClient<get_up_test::speedProfile>("/get_up_test/get_speed_profile");
	ros::Publisher pubHeadPositions = nh.advertise<std_msgs::Float32MultiArray>("/hardware/head_goal_pose", 10);
	ros::Publisher pubLegsPositions = nh.advertise<std_msgs::Float32MultiArray>("/hardware/legs_goal_pose", 10);
	ros::Publisher pubLeftArmPositions  = nh.advertise<std_msgs::Float32MultiArray>("/hardware/arm_left_goal_pose",  10);
	ros::Publisher pubRightArmPositions = nh.advertise<std_msgs::Float32MultiArray>("/hardware/arm_right_goal_pose", 10);
	ros::Rate loop(sampling_freq);	

	system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");

	string get_up_mode;
	if( argc == 2)
		get_up_mode = argv[1];
	else
		get_up_mode = "prone";

	vector<float> joint_goal_position;
	vector<vector<float> > joint_profile_positions;

	int robot_pose = 1;
	int number_poses=10;
	int time_k = 0;

	std_msgs::Float32MultiArray head_msg, legs_msg, left_arm_msg, right_arm_msg;
	head_msg.data.resize(2);
	legs_msg.data.resize(12);
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
				if(!get_pose(clt_get_pose, get_up_mode, robot_pose, number_poses, joint_goal_position))
					ROS_ERROR("Failed to call service get_pose_server.py");
				state = SM_COMPUTE_PROFILES;
			}
			else	
				state = SM_FINISH_TEST;
			break; 

		case SM_COMPUTE_PROFILES:
			cout<<"SM_COMPUTE_PROFILES"<<endl;
			time_k = 0;

			if(!compute_profiles(clt_speed_profile, joint_goal_position, joint_profile_positions))
				ROS_ERROR("Failed to call service speed_profile_server.py");
			state = SM_PUBLISH_PROFILES;
			break;

		case SM_PUBLISH_PROFILES:
			if(time_k < joint_profile_positions[0].size())
			{
				if(time_k==0)
					cout<<"SM_PUBLISH_PROFILES"<<endl;
				
				for(int id=0; id<20; id++){
					if(id < 12)
						legs_msg.data[id] = joint_profile_positions[id][time_k];
					if(id >= 12 && id < 15)
						left_arm_msg.data[id-12] = joint_profile_positions[id][time_k];
					if(id >= 15 && id < 18)
						right_arm_msg.data[id-15] = joint_profile_positions[id][time_k];
					if(id >= 18)
						head_msg.data[id-18] = joint_profile_positions[id][time_k];
				}

				pubHeadPositions.publish(head_msg);
				pubLegsPositions.publish(legs_msg);
				pubLeftArmPositions.publish(left_arm_msg);
				pubRightArmPositions.publish(right_arm_msg);

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
				return 1;
				break;

		default:
			cout<<"The state machine number is not valid"<<endl;
		}

		ros::spinOnce();
		loop.sleep();

	}//From while(ros::ok())

	return 0;
}
