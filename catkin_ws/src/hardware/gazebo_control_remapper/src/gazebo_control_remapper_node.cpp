#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"

ros::Publisher pubLeftHipYaw        ;
ros::Publisher pubLeftHipRoll       ;
ros::Publisher pubLeftHipPitch      ;
ros::Publisher pubLeftKneePitch     ;
ros::Publisher pubLeftAnklePitch    ;
ros::Publisher pubLeftAnkleRoll     ;
ros::Publisher pubRightHipYaw       ;
ros::Publisher pubRightHipRoll      ;
ros::Publisher pubRightHipPitch     ;
ros::Publisher pubRightKneePitch    ;
ros::Publisher pubRightAnklePitch   ;
ros::Publisher pubRightAnkleRoll    ;
ros::Publisher pubLeftShoulderPitch ;
ros::Publisher pubLeftShoulderRoll  ;
ros::Publisher pubLeftElbowPitch    ;
ros::Publisher pubRightShoulderPitch;
ros::Publisher pubRightShoulderRoll ;
ros::Publisher pubRightElbowPitch   ;
ros::Publisher pubNeckYaw           ;
ros::Publisher pubNeckPitch         ;
std_msgs::Float64 msgLeftHipYaw        ;
std_msgs::Float64 msgLeftHipRoll       ;
std_msgs::Float64 msgLeftHipPitch      ;
std_msgs::Float64 msgLeftKneePitch     ;
std_msgs::Float64 msgLeftAnklePitch    ;
std_msgs::Float64 msgLeftAnkleRoll     ;
std_msgs::Float64 msgRightHipYaw       ;
std_msgs::Float64 msgRightHipRoll      ;
std_msgs::Float64 msgRightHipPitch     ;
std_msgs::Float64 msgRightKneePitch    ;
std_msgs::Float64 msgRightAnklePitch   ;
std_msgs::Float64 msgRightAnkleRoll    ;
std_msgs::Float64 msgLeftShoulderPitch ;
std_msgs::Float64 msgLeftShoulderRoll  ;
std_msgs::Float64 msgLeftElbowPitch    ;
std_msgs::Float64 msgRightShoulderPitch;
std_msgs::Float64 msgRightShoulderRoll ;
std_msgs::Float64 msgRightElbowPitch   ;
std_msgs::Float64 msgNeckYaw           ;
std_msgs::Float64 msgNeckPitch         ;

void callback_legs_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 12)
    {
	std::cout << "ControlRemapper.->Error!!: goal position for both legs must be a 12-value array." << std::endl;
	return;
    }
    msgRightHipYaw      .data = msg->data[0];
    msgRightHipRoll     .data = msg->data[1];
    msgRightHipPitch    .data = msg->data[2];
    msgRightKneePitch   .data = msg->data[3];
    msgRightAnklePitch  .data = msg->data[4];
    msgRightAnkleRoll   .data = msg->data[5];
    msgLeftHipYaw       .data = msg->data[6];
    msgLeftHipRoll      .data = msg->data[7];
    msgLeftHipPitch     .data = msg->data[8];
    msgLeftKneePitch    .data = msg->data[9];
    msgLeftAnklePitch   .data = msg->data[10];
    msgLeftAnkleRoll    .data = msg->data[11];

    pubLeftHipYaw        .publish(msgLeftHipYaw        );
    pubLeftHipRoll       .publish(msgLeftHipRoll       );
    pubLeftHipPitch      .publish(msgLeftHipPitch      );
    pubLeftKneePitch     .publish(msgLeftKneePitch     );
    pubLeftAnklePitch    .publish(msgLeftAnklePitch    );
    pubLeftAnkleRoll     .publish(msgLeftAnkleRoll     );
    pubRightHipYaw       .publish(msgRightHipYaw       );
    pubRightHipRoll      .publish(msgRightHipRoll      );
    pubRightHipPitch     .publish(msgRightHipPitch     );
    pubRightKneePitch    .publish(msgRightKneePitch    );
    pubRightAnklePitch   .publish(msgRightAnklePitch   );
    pubRightAnkleRoll    .publish(msgRightAnkleRoll    );
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
    pubLeftHipYaw        .publish(msgLeftHipYaw        );
    pubLeftHipRoll       .publish(msgLeftHipRoll       );
    pubLeftHipPitch      .publish(msgLeftHipPitch      );
    pubLeftKneePitch     .publish(msgLeftKneePitch     );
    pubLeftAnklePitch    .publish(msgLeftAnklePitch    );
    pubLeftAnkleRoll     .publish(msgLeftAnkleRoll     );
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
    pubRightHipYaw       .publish(msgRightHipYaw       );
    pubRightHipRoll      .publish(msgRightHipRoll      );
    pubRightHipPitch     .publish(msgRightHipPitch     );
    pubRightKneePitch    .publish(msgRightKneePitch    );
    pubRightAnklePitch   .publish(msgRightAnklePitch   );
    pubRightAnkleRoll    .publish(msgRightAnkleRoll    );
}

void callback_arms_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 6)
    {
	std::cout << "ControlRemapper.->Error!!: goal position for both arms must be a 6-value array." << std::endl;
	return;
    }

    msgLeftShoulderPitch .data = msg->data[0];
    msgLeftShoulderRoll  .data = msg->data[1];
    msgLeftElbowPitch    .data = msg->data[2];
    msgRightShoulderPitch.data = msg->data[3];
    msgRightShoulderRoll .data = msg->data[4];
    msgRightElbowPitch   .data = msg->data[5];
    pubLeftShoulderPitch .publish(msgLeftShoulderPitch );
    pubLeftShoulderRoll  .publish(msgLeftShoulderRoll  );
    pubLeftElbowPitch    .publish(msgLeftElbowPitch    );
    pubRightShoulderPitch.publish(msgRightShoulderPitch);
    pubRightShoulderRoll .publish(msgRightShoulderRoll );
    pubRightElbowPitch   .publish(msgRightElbowPitch   );
}

void callback_arm_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 3)
    {
	std::cout << "ControlRemapper.->Error!!: goal position for left arm must be a 3-value array." << std::endl;
	return;
    }
    msgLeftShoulderPitch.data = msg->data[0];
    msgLeftShoulderRoll .data = msg->data[1];
    msgLeftElbowPitch   .data = msg->data[2];
    pubLeftShoulderPitch .publish(msgLeftShoulderPitch );
    pubLeftShoulderRoll  .publish(msgLeftShoulderRoll  );
    pubLeftElbowPitch    .publish(msgLeftElbowPitch    );
}

void callback_arm_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 3)
    {
	std::cout << "ControlRemapper.->Error!!: goal position for right leg must be a 3-value array." << std::endl;
	return;
    }
    msgRightShoulderPitch.data = msg->data[0];
    msgRightShoulderRoll .data = msg->data[1];
    msgRightElbowPitch   .data = msg->data[2];
    pubRightShoulderPitch.publish(msgRightShoulderPitch);
    pubRightShoulderRoll .publish(msgRightShoulderRoll );
    pubRightElbowPitch   .publish(msgRightElbowPitch   );
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
    pubNeckYaw           .publish(msgNeckYaw           );
    pubNeckPitch         .publish(msgNeckPitch         );
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

    pubLeftHipYaw         = n.advertise<std_msgs::Float64>("/nimbro/left_hip_yaw_position_controller/command",1);
    pubLeftHipRoll        = n.advertise<std_msgs::Float64>("/nimbro/left_hip_roll_position_controller/command",1);
    pubLeftHipPitch       = n.advertise<std_msgs::Float64>("/nimbro/left_hip_pitch_position_controller/command",1);
    pubLeftKneePitch      = n.advertise<std_msgs::Float64>("/nimbro/left_knee_pitch_position_controller/command",1);
    pubLeftAnklePitch     = n.advertise<std_msgs::Float64>("/nimbro/left_ankle_pitch_position_controller/command",1);
    pubLeftAnkleRoll      = n.advertise<std_msgs::Float64>("/nimbro/left_ankle_roll_position_controller/command",1); 
    pubRightHipYaw        = n.advertise<std_msgs::Float64>("/nimbro/right_hip_yaw_position_controller/command",1);    
    pubRightHipRoll       = n.advertise<std_msgs::Float64>("/nimbro/right_hip_roll_position_controller/command",1);   
    pubRightHipPitch      = n.advertise<std_msgs::Float64>("/nimbro/right_hip_pitch_position_controller/command",1);  
    pubRightKneePitch     = n.advertise<std_msgs::Float64>("/nimbro/right_knee_pitch_position_controller/command",1); 
    pubRightAnklePitch    = n.advertise<std_msgs::Float64>("/nimbro/right_ankle_pitch_position_controller/command",1);
    pubRightAnkleRoll     = n.advertise<std_msgs::Float64>("/nimbro/right_ankle_roll_position_controller/command",1);
    pubLeftShoulderPitch  = n.advertise<std_msgs::Float64>("/nimbro/left_shoulder_pitch_position_controller/command", 1);
    pubLeftShoulderRoll   = n.advertise<std_msgs::Float64>("/nimbro/left_shoulder_roll_position_controller/command",1);  
    pubLeftElbowPitch     = n.advertise<std_msgs::Float64>("/nimbro/left_elbow_pitch_position_controller/command", 1); 
    pubRightShoulderPitch = n.advertise<std_msgs::Float64>("/nimbro/right_shoulder_pitch_position_controller/command", 1);
    pubRightShoulderRoll  = n.advertise<std_msgs::Float64>("/nimbro/right_shoulder_roll_position_controller/command", 1);
    pubRightElbowPitch    = n.advertise<std_msgs::Float64>("/nimbro/right_elbow_pitch_position_controller/command", 1);
    pubNeckYaw            = n.advertise<std_msgs::Float64>("/nimbro/neck_yaw_position_controller/command", 1);
    pubNeckPitch          = n.advertise<std_msgs::Float64>("/nimbro/head_pitch_position_controller/command", 1);

    while(ros::ok())
    {
	ros::spinOnce();
	ros::spinOnce();
	loop.sleep();
    }
}
