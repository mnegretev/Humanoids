#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<std_msgs/Float32MultiArray.h>

#define ball_radius  0.03156

using namespace std;

ros::Publisher position_pub;

double px, py,  pz;
double roll, pitch, yaw;
double x, y, theta, psi;
double e_x , e_y, v;

double x_,y_;
double vx    , vy;
double dx=0  , dy=0;
double t0, ti, dt=0;

void angles_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std_msgs::Float32MultiArray kinematics_msg;
    kinematics_msg.data.resize(4);

    psi   =  msg->data[1] + pitch;
    theta =  msg->data[0] +  yaw ;
       

    e_x =  -1.3848*pow(pitch, 4) + 4.63*pow(pitch,3) - 5.691*pow(pitch,2) + 3.055*pitch - 0.6504;
    
    x = px - pz * tan(1.5708 + psi) * cos(theta) - ball_radius * cos(theta) / tan(psi) + e_x;
    v = sqrt( pow(x, 2) + pow(pz, 2) );    
    y = v * tan(theta) + e_y;

    dx = x - x_;
    dy = y - y_;
    dt = ti - t0;

    vx = dx / dt;
    vy = dy / dt;

    //Data for position (x,y)
    kinematics_msg.data[0] = x;
    kinematics_msg.data[1] = y;
    //Data for velocity (vx,vy)
    kinematics_msg.data[2] = vx;
    kinematics_msg.data[3] = vy; 
    

    position_pub.publish(kinematics_msg);

    x_ = x;
    y_ = y;
    t0 = ti;
}


int main(int argc, char **argv)
{
    cout<<"Starting ball_kinematics_node by Luis Näva..."<<endl;
    ros::init(argc, argv, "ball_kinematics_node");
    ros::NodeHandle nh;
    
    position_pub = nh.advertise<std_msgs::Float32MultiArray>("/vision/ball_kinematics/ball_kinematics", 1);    
    ros::Subscriber sub = nh.subscribe("/vision/ball_kinematics/vision_angles", 1000 ,  angles_callback);

    tf::TransformListener listener;

    

    while(nh.ok())
    {
        tf::StampedTransform transform;
        
        try{   
            listener.waitForTransform("/right_foot_plane_link", "/camera_optical", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/right_foot_plane_link", "/camera_optical", ros::Time(0), transform);
            transform.getBasis().getRPY(roll, pitch, yaw);

            px = transform.getOrigin().x();
            py = transform.getOrigin().y();
            pz = transform.getOrigin().z();

        }
        catch (tf::TransformException ex) {ROS_ERROR("%s",ex.what());}
       
        ti = ros::Time::now().toSec();
        ros::spinOnce();    
    }

    return 0;
}  


