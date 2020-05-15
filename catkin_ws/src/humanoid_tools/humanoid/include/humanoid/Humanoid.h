#pragma once
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ctrl_msgs/CalculateIK.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <humanoid_msgs/speedProfile.h>
#include <humanoid_msgs/predefPoses.h>

class Humanoid {

    private: 
        static int   goal_steps;
        static int  step_number;
        static bool enable_stop;
        static int sampling_freq;
        static std::string movement;
        
        static ros::ServiceClient clt_speed_profile;
        static ros::ServiceClient clt_get_predef_poses;
        static ros::ServiceClient clt_calculate_ik_leg_left;
        static ros::ServiceClient clt_calculate_ik_leg_right;

        static ros::Publisher pub_robot_stop;
        static ros::Publisher pub_head_positions;
        static ros::Publisher pub_legs_positions;
        static ros::Publisher pub_left_arm_positions;
        static ros::Publisher pub_right_arm_positions;
        static ros::Publisher pub_leg_left_goal_pose;
        static ros::Publisher pub_leg_right_goal_pose;

        static ros::Subscriber sub_stop_by_topic;

        static std::vector<float> current_joint_position;
        static std::vector<float> left_kick_pose_duration;
        static std::vector<float> right_kick_pose_duration;
        static std::vector<float> prone_get_up_pose_duration;
        static std::vector<float> supine_get_up_pose_duration;
        static std::vector<float> ready_to_kick_pose_duration;
       
        static std::vector<std::vector<float> > leg_left_angles;    
        static std::vector<std::vector<float> > leg_right_angles;   
        static std::vector<std::vector<float> > left_kick_movements;
        static std::vector<std::vector<float> > right_kick_movements;
        static std::vector<std::vector<float> > prone_get_up_movements;
        static std::vector<std::vector<float> > supine_get_up_movements;
        static std::vector<std::vector<float> > ready_to_kick_movements;
        static std::vector<std::vector<float> > joint_profile_positions;

        static void loadPredefPoses(std::vector<std::vector<float> >&, std::vector<float>&);
        static void computeSpeedProfile(std::vector<std::vector<float> >&,std::vector<float>&, int);

        static void publishPositions();

        static void callback_stop_by_topic(const std_msgs::Bool::ConstPtr& msg);
        
    public:
        static void setNodeHandle(ros::NodeHandle* nh);

        static void stop();
        static void restart();
        static void getCurrentPose();
        static void loadWalkPositions();

        static void walk();
        static void setStepsNumber(int);

        static void leftKick();
        static void rightKick();
        static void proneGetUp();
        static void supineGetUp();
        static void readyToKick();
};
