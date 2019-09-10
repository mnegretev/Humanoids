#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>



using namespace std;

double  x,  y;
double vx, vy;
double dx = 0, dy = 0;
double t0, tf, dt = 0;


ros::Publisher pub_vel;


void position_callback(const std_msgs::Float32MultiArray::ConstPtr msg)
{
//    cout<<"["<<msg->data[0]<<", "<<msg->data[1]<<"]"<<endl;
    cout<<"-------------------------------------------"<<endl;
    cout<<endl;

    std_msgs::Float32MultiArray vel_msg;
    vel_msg.data.resize(2);
    
    dx = msg->data[0] - x ;
    dy = msg->data[1] - y ;
    dt = tf - t0;
    
    vx = dx / dt;
    vy = dy / dt;
    
    vel_msg.data[0] = vx ;
    vel_msg.data[1] = vy ;
    
    pub_vel.publish(vel_msg);

    cout<<"dx: "<<dx<<" dy: "<<dy<<endl;
    cout<<"Velocity: ["<<vx<<", "<<vy<<"]"<<endl;
    cout<<"dt: "<<dt<<endl;
 
    x = msg->data[0];
    y = msg->data[1];
    t0 = tf;

}


int main(int argc, char ** argv)
{
    cout<<"Starting get_ball_velocity_node by Luis Näva"<<endl;
    ros::init(argc, argv, "get_ball_velocity_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_position = nh.subscribe("/vision/get_ball_position/ball_position", 1000, position_callback);
                    pub_vel      = nh.advertise<std_msgs::Float32MultiArray>("/vision/get_ball_velocity/ball_velocity", 1);    
    
    
    while(ros::ok())
    {
        tf = ros::Time::now().toSec();        
        ros::spinOnce();
    }
    


    return 0;
}
