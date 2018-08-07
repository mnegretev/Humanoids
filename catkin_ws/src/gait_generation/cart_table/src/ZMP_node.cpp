#include "ros/ros.h" 
#include "std_msgs/Float32MultiArray.h"
#include "ctrl_msgs/CalculateIK.h"

int main(int argc, char** argv)
 {     
    //Initilize joint_states_publisher
    std::cout << "INITIALIZING ZMP NODE..." << std::endl;
    ros::init(argc, argv, "ZMP");
    ros::NodeHandle n;
    ros::Rate loop(30);
    
    ros::service::waitForService("/control/ik_leg_left", 15);
    ros::ServiceClient clt_link_props = n.serviceClient<ctrl_msgs::CalculateIK>("/control/ik_leg_left");
    ctrl_msgs::CalculateIK srv_props;
    ros::Publisher  joint_pub = n.advertise<std_msgs::Float32MultiArray>("/hardware/leg_left_goal_pose", 1);

    float i = 0;

     while(ros::ok())
    {

    i = i + M_PI/128;
    srv_props.request.x = -0.07 * sin(i) + 0.004;    
    srv_props.request.y = 0.055 + 0.03;
    srv_props.request.z = 0.05 * fabs(sin(i/2)) - 0.577;
    srv_props.request.roll = 0;
    srv_props.request.pitch = 0;
    srv_props.request.yaw = 0;

    clt_link_props.call(srv_props);
    std_msgs::Float32MultiArray msg;
    msg.data = srv_props.response.joint_values;

    joint_pub.publish(msg);

    }

    
 }
