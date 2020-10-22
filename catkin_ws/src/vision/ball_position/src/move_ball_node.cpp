#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>


#define  g 9.81
#define Mg 0.15

using namespace std;


bool log_out;

void stop_moving_callback(const std_msgs::BoolConstPtr& msg) { 
   log_out = msg->data; 
}


int main(int argc, char** argv) {
	cout << "Starting move_ball_node by Luis Näva" << endl;
	ros::init(argc, argv, "move_ball_node");
	ros::NodeHandle nh;

	float sample_freq = 50;

    ros::Subscriber stop_moving  = nh.subscribe("/robot_stop", 1, stop_moving_callback);
	ros::Publisher move_ball_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Rate loop(sample_freq);
    
    float velx, vely, t = 0;
    float vx0 = atof(argv[1]);
    float vy0 = atof(argv[2]);
    
    cout << "Moving ball with initial vel x: " << vx0 << "\tvel y: " << vy0 << " m/s" << endl;

    ros::Duration(1.0).sleep();

	while(ros::ok()) {
		geometry_msgs::Twist ball_msg;

        velx = vx0 - Mg * g * t; 
        vely = vy0 - Mg * g * t;

        if(velx < 0) velx = 0;
        
        //cout << "vely: " << vely << "\ttime: " << t << endl;

        ball_msg.linear.x = -velx;
		ball_msg.linear.y = vely;

		move_ball_pub.publish(ball_msg);

        t += 1 / sample_freq;
		loop.sleep();
		ros::spinOnce();

        if(vely < 0 || log_out) break;
	}

	return 0;	
}
