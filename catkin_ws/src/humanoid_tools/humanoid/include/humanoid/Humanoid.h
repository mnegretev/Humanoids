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
        static int sampling_freq;
        static bool enable_stop;
        static std::string movement;
        static ros::ServiceClient clt_get_predef_poses;
        static ros::ServiceClient clt_speed_profile;

        static ros::Publisher pub_robot_stop;
        static ros::Publisher pub_head_positions;
        static ros::Publisher pub_legs_positions;
        static ros::Publisher pub_left_arm_positions;
        static ros::Publisher pub_right_arm_positions;

        static std::vector<float> current_joint_position;
        static std::vector<float> left_kick_pose_duration;
        static std::vector<float> right_kick_pose_duration;
        static std::vector<float> prone_get_up_pose_duration;
        static std::vector<float> supine_get_up_pose_duration;
        static std::vector<float> ready_to_kick_pose_duration;
       
        static std::vector<std::vector<float> > left_kick_movements;
        static std::vector<std::vector<float> > right_kick_movements;
        static std::vector<std::vector<float> > prone_get_up_movements;
        static std::vector<std::vector<float> > supine_get_up_movements;
        static std::vector<std::vector<float> > ready_to_kick_movements;
        static std::vector<std::vector<float> > joint_profile_positions;

        static void computeSpeedProfile(std::vector<std::vector<float> >&,std::vector<float>&, int);
        static void loadPredefPoses(std::vector<std::vector<float> >&, std::vector<float>&);

        static void publishPositions();
        


    public:
        static void setNodeHandle(ros::NodeHandle* nh);

        static void stop();
        static void restart();
        static void getCurrentPose();

        static void leftKick();
        static void rightKick();
        static void proneGetUp();
        static void supineGetUp();
        static void readyToKick();
};
