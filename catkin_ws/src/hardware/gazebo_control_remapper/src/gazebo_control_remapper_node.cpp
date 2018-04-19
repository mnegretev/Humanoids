#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"

std_msgs::Float64 msgLeftHipYaw     ;
std_msgs::Float64 msgLeftHipRoll    ;
std_msgs::Float64 msgLeftHipPitch   ;
std_msgs::Float64 msgLeftKneePitch  ;
std_msgs::Float64 msgLeftAnklePitch ;
std_msgs::Float64 msgLeftAnkleRoll  ;
std_msgs::Float64 msgRightHipYaw    ;
std_msgs::Float64 msgRightHipRoll   ;
std_msgs::Float64 msgRightHipPitch  ;
std_msgs::Float64 msgRightKneePitch ;
std_msgs::Float64 msgRightAnklePitch;
std_msgs::Float64 msgRightAnkleRoll ;
std_msgs::Float64 msgLeftShoulPitch ;
std_msgs::Float64 msgLeftShoulRoll  ;
std_msgs::Float64 msgLeftElbowPitch ;
std_msgs::Float64 msgRightShoulPitch;
std_msgs::Float64 msgRightShoulRoll ;
std_msgs::Float64 msgRightElbowPitch;
std_msgs::Float64 msgNeckYaw        ;
std_msgs::Float64 msgNeckPitch      ;

void callback_legs_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 12)
    {
	std::cout << "ControlRemapper.->Error!!: goal position for both legs must be a 12-value array." << std::endl;
	return;
    }

    msgLeftHipYaw     .data = msg->data[0];
    msgLeftHipRoll    .data = msg->data[1];
    msgLeftHipPitch   .data = msg->data[2];
    msgLeftKneePitch  .data = msg->data[3];
    msgLeftAnklePitch .data = msg->data[4];
    msgLeftAnkleRoll  .data = msg->data[5];
    msgRightHipYaw    .data = msg->data[6];
    msgRightHipRoll   .data = msg->data[7];
    msgRightHipPitch  .data = msg->data[8];
    msgRightKneePitch .data = msg->data[9];
    msgRightAnklePitch.data = msg->data[10];
    msgRightAnkleRoll .data = msg->data[11];
}

void callback_leg_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 6)
    {
	std::cout << "ControlRemapper.->Error!!: goal position for left leg must be a 6-value array." << std::endl;
	return;
    }
    msgLeftHipYaw     .data = msg->data[0];
    msgLeftHipRoll    .data = msg->data[1];
    msgLeftHipPitch   .data = msg->data[2];
    msgLeftKneePitch  .data = msg->data[3];
    msgLeftAnklePitch .data = msg->data[4];
    msgLeftAnkleRoll  .data = msg->data[5];
}

void callback_leg_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 6)
    {
	std::cout << "ControlRemapper.->Error!!: goal position for right leg must be a 6-value array." << std::endl;
	return;
    }
    msgRightHipYaw    .data = msg->data[0];
    msgRightHipRoll   .data = msg->data[1];
    msgRightHipPitch  .data = msg->data[2];
    msgRightKneePitch .data = msg->data[3];
    msgRightAnklePitch.data = msg->data[4];
    msgRightAnkleRoll .data = msg->data[5];
}

void callback_arms_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 6)
    {
	std::cout << "ControlRemapper.->Error!!: goal position for both arms must be a 6-value array." << std::endl;
	return;
    }

    msgLeftShoulPitch .data = msg->data[0];
    msgLeftShoulRoll  .data = msg->data[1];
    msgLeftElbowPitch .data = msg->data[2];
    msgRightShoulPitch.data = msg->data[3];
    msgRightShoulRoll .data = msg->data[4];
    msgRightElbowPitch.data = msg->data[5];
}

void callback_arm_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 3)
    {
	std::cout << "ControlRemapper.->Error!!: goal position for left arm must be a 3-value array." << std::endl;
	return;
    }
    msgLeftShoulPitch .data = msg->data[0];
    msgLeftShoulRoll  .data = msg->data[1];
    msgLeftElbowPitch .data = msg->data[2];
}

void callback_arm_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 3)
    {
	std::cout << "ControlRemapper.->Error!!: goal position for right leg must be a 3-value array." << std::endl;
	return;
    }
    msgRightShoulPitch.data = msg->data[0];
    msgRightShoulRoll .data = msg->data[1];
    msgRightElbowPitch.data = msg->data[2];
}

void callback_head_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 2)
    {
	std::cout << "ControlRemapper.->Error!! goal position for head must be a 2-value array" << std::endl;
	return;
    }
    msgNeckYaw  .data = msg->data[0];
    msgNeckPitch.data = msg->data[1];
}


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING GAZEBO CONTROL MAPPER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "gazebo_control_mapper");
    ros::NodeHandle n;
    ros::Rate loop(30);

    ros::Subscriber sub_legs_goal_pose      = n.subscribe("legs_goal_pose", 1, callback_legs_goal_pose);
    ros::Subscriber sub_leg_left_goal_pose  = n.subscribe("leg_left_goal_pose", 1, callback_leg_left_goal_pose);
    ros::Subscriber sub_leg_right_goal_pose = n.subscribe("leg_right_goal_pose", 1, callback_leg_right_goal_pose);
    ros::Subscriber sub_arms_goal_pose      = n.subscribe("arms_goal_pose", 1, callback_arms_goal_pose);
    ros::Subscriber sub_arm_left_goal_pose  = n.subscribe("arm_left_goal_pose", 1, callback_arm_left_goal_pose);
    ros::Subscriber sub_arm_right_goal_pose = n.subscribe("arm_right_goal_pose", 1, callback_arm_right_goal_pose);
    ros::Subscriber sub_head_goal_pose      = n.subscribe("head_goal_pose", 1, callback_head_goal_pose);

    ros::Publisher pubLeftHipYaw      = n.advertise<std_msgs::Float64>("/nimbro/left_hip_yaw_position_controller/command",1);
    ros::Publisher pubLeftHipRoll     = n.advertise<std_msgs::Float64>("/nimbro/left_hip_roll_position_controller/command",1);
    ros::Publisher pubLeftHipPitch    = n.advertise<std_msgs::Float64>("/nimbro/left_hip_pitch_position_controller/command",1);
    ros::Publisher pubLeftKneePitch   = n.advertise<std_msgs::Float64>("/nimbro/left_knee_pitch_position_controller/command",1);
    ros::Publisher pubLeftAnklePitch  = n.advertise<std_msgs::Float64>("/nimbro/left_ankle_pitch_position_controller/command",1);
    ros::Publisher pubLeftAnkleRoll   = n.advertise<std_msgs::Float64>("/nimbro/left_ankle_roll_position_controller/command",1); 
    ros::Publisher pubRightHipYaw     = n.advertise<std_msgs::Float64>("/nimbro/right_hip_yaw_position_controller/command",1);    
    ros::Publisher pubRightHipRoll    = n.advertise<std_msgs::Float64>("/nimbro/right_hip_roll_position_controller/command",1);   
    ros::Publisher pubRightHipPitch   = n.advertise<std_msgs::Float64>("/nimbro/right_hip_pitch_position_controller/command",1);  
    ros::Publisher pubRightKneePitch  = n.advertise<std_msgs::Float64>("/nimbro/right_knee_pitch_position_controller/command",1); 
    ros::Publisher pubRightAnklePitch = n.advertise<std_msgs::Float64>("/nimbro/right_ankle_pitch_position_controller/command",1);
    ros::Publisher pubRightAnkleRoll  = n.advertise<std_msgs::Float64>("/nimbro/right_ankle_roll_position_controller/command",1);
    ros::Publisher pubLeftShoulPitch  = n.advertise<std_msgs::Float64>("/nimbro/right_shoulder_pitch_position_controller", 1);
    ros::Publisher pubLeftShoulRoll   = n.advertise<std_msgs::Float64>("/nimbro/right_shoulder_roll_position_controller",1);  
    ros::Publisher pubLeftElbowPitch  = n.advertise<std_msgs::Float64>("/nimbro/right_elbow_pitch_position_controller", 1); 
    ros::Publisher pubRightShoulPitch = n.advertise<std_msgs::Float64>("/nimbro/left_shoulder_pitch_position_controller", 1);
    ros::Publisher pubRightShoulRoll  = n.advertise<std_msgs::Float64>("/nimbro/left_shoulder_roll_position_controller", 1);
    ros::Publisher pubRightElbowPitch = n.advertise<std_msgs::Float64>("/nimbro/left_elbow_pitch_position_controller", 1);
    ros::Publisher pubNeckYaw  	      = n.advertise<std_msgs::Float64>("/nimbro/neck_yaw_position_controller", 1);
    ros::Publisher pubNeckPitch	      = n.advertise<std_msgs::Float64>("/nimbro/head_pitch_position_controller", 1);

    while(ros::ok())
    {
	pubLeftHipYaw     .publish(msgLeftHipYaw     );
	pubLeftHipRoll    .publish(msgLeftHipRoll    );
	pubLeftHipPitch   .publish(msgLeftHipPitch   );
	pubLeftKneePitch  .publish(msgLeftKneePitch  );
	pubLeftAnklePitch .publish(msgLeftAnklePitch );
	pubLeftAnkleRoll  .publish(msgLeftAnkleRoll  );
	pubRightHipYaw    .publish(msgRightHipYaw    );
	pubRightHipRoll   .publish(msgRightHipRoll   );
	pubRightHipPitch  .publish(msgRightHipPitch  );
	pubRightKneePitch .publish(msgRightKneePitch );
        pubRightAnklePitch.publish(msgRightAnklePitch);
	pubRightAnkleRoll .publish(msgRightAnkleRoll );
	pubLeftShoulPitch .publish(msgLeftShoulPitch );
	pubLeftShoulRoll  .publish(msgLeftShoulRoll  );
	pubLeftElbowPitch .publish(msgLeftElbowPitch );
	pubRightShoulPitch.publish(msgRightShoulPitch);
	pubRightShoulRoll .publish(msgRightShoulRoll );
	pubRightElbowPitch.publish(msgRightElbowPitch);
	pubNeckYaw        .publish(msgNeckYaw        );
	pubNeckPitch      .publish(msgNeckPitch      );
	
	ros::spinOnce();
	loop.sleep();
    }
}
