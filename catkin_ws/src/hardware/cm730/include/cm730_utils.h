#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "servo_utils.h"

uint16_t servos_goal_position[20];  // PENDING TO BE READ
std::vector<Servo::servo_t> left_arm_servos;
std::vector<Servo::servo_t> right_arm_servos;
std::vector<Servo::servo_t> left_leg_servos;
std::vector<Servo::servo_t> right_leg_servos;
std::vector<Servo::servo_t> head_servos;

std::vector<std::string> left_arm_names
{
    "/left/arm/shoulder/pitch",
    "/left/arm/shoulder/roll",
    "/left/arm/elbow/pitch"
};

std::vector<std::string> right_arm_names
{
    "/right/arm/shoulder/pitch",
    "/right/arm/shoulder/roll",
    "/right/arm/elbow/pitch"
};

std::vector<std::string> left_leg_names
{
    "/left/leg/hip/yaw",
    "/left/leg/hip/roll",
    "/left/leg/hip/pitch",
    "/left/leg/knee/pitch",
    "/left/leg/ankle/pitch",
    "/left/leg/ankle/roll",
};

std::vector<std::string> right_leg_names
{
    "/right/leg/hip/yaw",
    "/right/leg/hip/roll",
    "/right/leg/hip/pitch",
    "/right/leg/knee/pitch"
    "/right/leg/ankle/pitch",
    "/right/leg/ankle/roll",
};

std::vector<std::string> head_names
{
    "/head/yaw",
    "/head/pitch"
};

namespace CM730
{
    class Node 
    {
    private: 
        ros::NodeHandle n;
        ros::Rate       loop{30};
        ros::Subscriber sub_legs_goal_pose; 
        ros::Subscriber sub_leg_left_goal_pose;  
        ros::Subscriber sub_leg_right_goal_pose;
        ros::Subscriber sub_arms_goal_pose;
        ros::Subscriber sub_arm_left_goal_pose;
        ros::Subscriber sub_arm_right_goal_pose; 
        ros::Subscriber sub_head_goal_pose;
    
        ros::Publisher pub_legs_current_pose;
        ros::Publisher pub_leg_left_current_pose;
        ros::Publisher pub_leg_right_current_pose;
        ros::Publisher pub_arms_current_pose;
        ros::Publisher pub_arm_left_current_pose;
        ros::Publisher pub_arm_right_current_pose;
        ros::Publisher pub_head_current_pose;
        ros::Publisher pub_joint_current_angles;
        ros::Publisher pub_joint_states;

        Servo::CommHandler comm;

    public:
        Node(std::string port_name);
        bool startNode();
        static void callback_legs_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        static void callback_leg_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        static void callback_leg_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg); 
        static void callback_arms_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        static void callback_arm_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        static void callback_arm_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        static void callback_head_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        bool fillServoParameters(std::vector<std::string>& servo_names, std::vector<Servo::servo_t>& servo_list);
    };
}
