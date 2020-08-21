#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


//#define v0 5.336 
#define  g 9.81
#define Mg 0.3636

using namespace std;

int main(int argc, char** argv) {
	cout << "Starting move_ball_node by Luis Näva" << endl;
	ros::init(argc, argv, "move_ball_node");
	ros::NodeHandle nh;

	ros::Publisher move_ball_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Rate loop(30);
    
    float vel, t = 0;
    float v0 = atof(argv[1]);
    
    cout << "Moving ball with initial vel: " << v0 << " m/s" << endl;

	while(ros::ok()) {
		geometry_msgs::Twist ball_msg;

        vel = v0 - Mg * g * t; 

		ball_msg.linear.y = vel;

		move_ball_pub.publish(ball_msg);

        t += 0.0333;
		loop.sleep();
		ros::spinOnce();

        if(vel <= 0) break;
	}

	return 0;	
}
