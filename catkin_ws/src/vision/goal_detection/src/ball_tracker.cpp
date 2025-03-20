#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <smach/state.h>
#include <smach/state_machine.h>
#include <vector>
#include <cmath>
#include <iostream>

double pan_robot = 0.0;
double tilt_robot = 0.0;

class StartingSearch : public smach::State {
private:
    ros::NodeHandle nh_;
    ros::Subscriber centroid_sub_;
    ros::Publisher head_pub_;
    
    
}