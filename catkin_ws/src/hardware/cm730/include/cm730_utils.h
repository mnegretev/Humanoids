#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "servo_utils.h"

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
        
        Servo::CommHandler comm{"/dev/ttyUSB0"};

    public:
        CM730Node(std::string node_name);
        bool startNode(std::string port_name);
        void callback_legs_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void callback_leg_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void callback_leg_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg); 
        void callback_arms_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void callback_arm_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void callback_arm_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void callback_head_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg);
        bool fillServoParameters(std::vector<std::string>& servo_names, std::vector<Servo::servo_t>& servo_list)
    }
}
