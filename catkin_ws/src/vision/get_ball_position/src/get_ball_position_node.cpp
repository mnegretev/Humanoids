#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<std_msgs/Float32MultiArray.h>

#define ball_radius  0.03156

using namespace std;

ros::Publisher position_pub;

double px, py,  pz;
double roll, pitch, yaw;
double x, y, theta, psi;
double e_x , e_y;

void angles_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{

    std_msgs::Float32MultiArray position_msg;
    position_msg.data.resize(2);

    psi   =  msg->data[1] + pitch;
    theta =  msg->data[0] +  yaw ;
       

    e_x =  -1.3848*pow(pitch, 4) + 4.63*pow(pitch,3) - 5.691*pow(pitch,2) + 3.055*pitch - 0.6504;

      
    x = px - pz * tan(1.5708 + pitch) * cos(yaw) - ball_radius * cos(yaw) / tan(pitch) + e_x;
    y = py - pz * tan(1.5708 + pitch) * sin(yaw) - ball_radius * sin(yaw) / tan(pitch);
//    x = px - pz * tan(1.5708 + psi) * cos(theta) - ball_radius * cos(theta) / tan(psi) + e_x;
//    y = py - pz * tan(1.5708 + psi) * sin(theta) - ball_radius * sin(theta) / tan(psi);



    cout<<"Pitch: "<<pitch<<endl;

    position_msg.data[0] = x;
    position_msg.data[1] = y;


    position_pub.publish(position_msg);
}


int main(int argc, char **argv)
{
    cout<<"Starting get_ball_position_node by Luis Näva..."<<endl;
    ros::init(argc, argv, "get_ball_position_node");
    ros::NodeHandle nh;
    
    position_pub = nh.advertise<std_msgs::Float32MultiArray>("/vision/get_ball_position/ball_position", 1);    
    ros::Subscriber sub = nh.subscribe("/vision/get_ball_position/vision_angles", 1000 ,  angles_callback);

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
       

        ros::spinOnce();    
    }

    return 0;
}  

