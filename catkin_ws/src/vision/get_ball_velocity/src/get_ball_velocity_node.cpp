#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>



using namespace std;

float  x,  y;
float dx, dy;
float vx, vy;
float dt, t0, tf;

void position_callback(const std_msgs::Float32MultiArray::ConstPtr msg)
{
    cout<<"["<<msg->data[0]<<", "<<msg->data[1]<<"]"<<endl;

    dx = msg->data[0] - x ;
    dy = msg->data[1] - y ;

    ros::Time t0 = ros::Time::now();
    
    x = msg->data[0];
    y = msg->data[1];



}


int main(int argc, char ** argv)
{
    cout<<"Starting get_ball_velocity_node by Luis Näva"<<endl;
    ros::init(argc, argv, "get_ball_velocity_node");
    ros::NodeHandle nh;

    ros::Subscriber position_sub = nh.subscribe("/vision/get_ball_position/ball_position", 1000, position_callback);
    
    while(ros::ok())
    {
        

        dt = tf - t0;
  
        vx = dx / dt;
        vy = dy / dt;

        cout<<"Velocity: ["<<vx<<", "<<vy<<"]"<<endl;
        ros::Time tf = ros::Time::now();
        ros::spinOnce();
        
    }
    


    return 0;
}
