#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Float32MultiArray.h>
#include<sensor_msgs/JointState.h>

#define vertical_threshold     0.15
#define horizontal_threshold   0.20

#define angle_view   0.680678

using namespace std;


float  goal_pan;
float goal_tilt;

ros::Publisher head_pub;

void angles_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{

    goal_pan  += 0.06 * (msg->data[0]);
    goal_tilt += 0.06 * (msg->data[1]); 
    
}


int main(int argc, char **argv)
{
    cout<<"Starting follow_colour_node by Luis Näva..."<<endl;
    ros::init(argc, argv, "follow_colour_node");
    ros::NodeHandle nh;

    ros::Rate loop(50);

    system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");

    ros::Subscriber angles_sub = nh.subscribe("/vision/get_ball_position/vision_angles", 1000 , angles_callback);
                    head_pub   = nh.advertise<std_msgs::Float32MultiArray>("/hardware/head_goal_pose", 1);

    std_msgs::Float32MultiArray head_msg;
    head_msg.data.resize(2);

    while(ros::ok())
    {
     
        head_msg.data[0] = goal_pan ;
        head_msg.data[1] = goal_tilt;
    
        head_pub.publish(head_msg);    

        loop.sleep();
        ros::spinOnce();
    
    } 
    return 0;
}
