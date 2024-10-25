#include "servo_utils.h"

namespace CM730
{
    Node::Node(std::string node_name)
    {
        sub_legs_goal_pose          = n.subscribe("legs_goal_pose",      1, callback_legs_goal_pose);
        sub_leg_left_goal_pose      = n.subscribe("leg_left_goal_pose",  1, callback_leg_left_goal_pose);
        sub_leg_right_goal_pose     = n.subscribe("leg_right_goal_pose", 1, callback_leg_right_goal_pose);
        sub_arms_goal_pose          = n.subscribe("arms_goal_pose",      1, callback_arms_goal_pose);
        sub_arm_left_goal_pose      = n.subscribe("arm_left_goal_pose",  1, callback_arm_left_goal_pose);
        sub_arm_right_goal_pose     = n.subscribe("arm_right_goal_pose", 1, callback_arm_right_goal_pose);
        sub_head_goal_pose          = n.subscribe("head_goal_pose",      1, callback_head_goal_pose);
        
        pub_legs_current_pose       = n.advertise<std_msgs::Float32MultiArray>("legs_current_pose",       1);
        pub_leg_left_current_pose   = n.advertise<std_msgs::Float32MultiArray>("leg_left_current_pose",   1);
        pub_leg_right_current_pose  = n.advertise<std_msgs::Float32MultiArray>("leg_right_current_pose",  1);
        pub_arms_current_pose       = n.advertise<std_msgs::Float32MultiArray>("arms_current_pose",       1);
        pub_arm_left_current_pose   = n.advertise<std_msgs::Float32MultiArray>("arm_left_current_pose",   1);
        pub_arm_right_current_pose  = n.advertise<std_msgs::Float32MultiArray>("arm_right_current_pose",  1);
        pub_head_current_pose       = n.advertise<std_msgs::Float32MultiArray>("head_current_pose",       1);
        pub_joint_current_angles    = n.advertise<std_msgs::Float32MultiArray>("joint_current_angles",    1);
        pub_joint_states            = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    }

    bool Node::startNode(std::string port_name)
    {
        if(!fillServoParameters(left_arm_names, left_arm_servos)) return false;
        if(!fillServoParameters(right_arm_names, right_arm_servos)) return false;
        if(!fillServoParameters(left_leg_names, left_leg_servos)) return false;
        if(!fillServoParameters(right_leg_names, right_leg_servos)) return false;
        if(!fillServoParameters(head_names, head_servos)) return 0;
        
        Servo::CommHandler comm("/dev/ttyUSB0");

        {int counter{0};
        while( !comm.startComm() )
        {
            counter++;
            ros::Duration(1.0).sleep();
            if(counter > 10)
            {
                ROS_ERROR("Shutting down after attempting to open port. Shutting down");
                return -1;
            };
        }}
    }

    void Node::callback_legs_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 12)
        {
           std::cout << "CM730.->Error!!: goal position for both legs must be a 12-value array." << std::endl;
           return;
        }
        for(int i=0, j=0; i < 12; i++, j++)
            servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] +
                servos_position_zero[j]);
    }

    void Node::callback_leg_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 6)
        {
           std::cout << "CM730.->Error!!: goal position for left leg must be a 6-value array." << std::endl;
           return;
        }
        for(int i=0, j=0; i < 6; i++, j++)
            servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] + 
                servos_position_zero[j]);
    }

    void Node::callback_leg_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 6)
        {
            std::cout << "CM730.->Error!!: goal position for right leg must be a 6-value array." << std::endl;
            return;
        }
        for(int i=0, j=6; i < 6; i++, j++)
            servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] +
                servos_position_zero[j]);
    }

    void Node::callback_arms_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 6)
        {
            std::cout << "CM730.->Error!!: goal position for both arms must be a 6-value array." << std::endl;
            return;
        }
        for(int i=0, j=12; i < 6; i++, j++)
            servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] +
                servos_position_zero[j]);
    }

    void Node::callback_arm_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 3)
        {
            std::cout << "CM730.->Error!!: goal position for left arm must be a 3-value array." << std::endl;
            return;
        }
        for(int i=0, j=12; i < 3; i++, j++)
            servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] +
                servos_position_zero[j]);
}

    void Node::callback_arm_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 3)
        {
            std::cout << "CM730.->Error!!: goal position for right leg must be a 3-value array." << std::endl;
            return;
        }
        for(int i=0, j=15; i < 3; i++, j++)
            servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] +
                servos_position_zero[j]);
    }
    
    void Node::callback_head_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 2)
        {
            std::cout << "CM730.->Error!! goal position for head must be a 2-value array" << std::endl;
            return;
        }
        for(int i=0, j=18; i < 2; i++, j++)
            servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] +
                servos_position_zero[j]);
    }

    bool Node::fillServoParameters(std::vector<std::string>& servo_names, std::vector<Servo::servo_t>& servo_list)
    {
        for(auto name: servo_names)
        {
            Servo::servo_t servo;
            std::string param_id_str    {name + "/id"};
            std::string param_cw_str    {name + "/cw"};
            std::string param_zero_str  {name + "/zero"};
            std::string param_enabled_str {name + "/enabled"};

            int id, zero;
            if (!ros::param::get(param_id_str, id))
            {
                ROS_ERROR("Missing param in config file: %s", param_id_str.c_str());
                return false;
            }
            servo.id = id;
            if (!ros::param::get(param_cw_str, servo.cw))
            {
                ROS_ERROR("Missing param in config file: %s", param_cw_str.c_str());
                return false;
            }
            if (!ros::param::get(param_zero_str, zero))
            {
                ROS_ERROR("Missing param in config file: %s", param_zero_str.c_str());
                return false;
            }
            servo.zero = zero;
            if (!ros::param::get(param_enabled_str, servo.enabled))
            {
                ROS_ERROR("Missing param in config file: %s", param_enabled_str.c_str());
                return false;
            }
            servo_list.push_back(servo);
        }
        return true;
    }
}