#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>
#include <vision_msgs/VisionObject.h>


const double PAN_MIN = -M_PI / 2.1;
const double PAN_MAX = M_PI / 2.1;
const double TILT_MIN = 0.5;
const double TILT_MAX = 0.7;
const double TILT_MIN_TRACK = 0.1;
const double TILT_MAX_TRACK = 0.9;

ros::Publisher tracker_pub;
//ros::Subscriber head_position_sub;
//ros::Subscriber filtered_centroid;
ros::Time last_detection_time;
double timeout = 5.0;  
float pan_robot = 0.0;
float tilt_robot = 0.5;
float last_search_pan ;
float last_search_tilt;
enum State { SEARCHING, TRACKING, LOST };
State current_state = SEARCHING;

geometry_msgs::Point32 getSearchTrajectory() {
    static double current_pan = PAN_MIN;
    static double current_tilt = TILT_MIN;
    static bool increasing_pan = true;
    
    geometry_msgs::Point32 search_point;
    
    if (increasing_pan) {
        current_pan += 0.05;
        if (current_pan >= PAN_MAX) {
            current_pan = PAN_MAX;
            increasing_pan = false;
            if (current_tilt < TILT_MAX) current_tilt += 0.1;
        }
    } else {
        current_pan -= 0.05;
        if (current_pan <= PAN_MIN) {
            current_pan = PAN_MIN;
            increasing_pan = true;
            if (current_tilt < TILT_MAX) current_tilt += 0.1;
        }
    }
    if (current_tilt >= TILT_MAX){
    current_tilt = TILT_MIN;}
    search_point.x = current_pan;
    search_point.y = current_tilt;
    last_search_pan = current_pan;
    last_search_tilt = current_tilt;
    return search_point;
}
float current_robot_pan = 0.0;
float current_robot_tilt = 0.5;
void stateMachine() {
    ros::Time current_time = ros::Time::now();
    double time_since_last_detection = (current_time - last_detection_time).toSec();
    std_msgs::Float32MultiArray head_goal_pose_msg;

    switch (current_state) {
        case SEARCHING:
            if (time_since_last_detection > timeout) {
                geometry_msgs::Point32 search_point = getSearchTrajectory();
                current_robot_pan = search_point.x;
                current_robot_tilt = search_point.y;
                head_goal_pose_msg.data = {current_robot_pan, current_robot_tilt};
                tracker_pub.publish(head_goal_pose_msg);
                ROS_INFO("Searching... pan: %.2f, tilt: %.2f", search_point.x, search_point.y);
            }
            break;

        case TRACKING:
            if (time_since_last_detection > timeout) {
                ROS_WARN("Lost ball, switching to LOST state");
                current_state = LOST;
            }
            break;

        case LOST:
            if (time_since_last_detection > timeout + 3.0) { 
                ROS_WARN("No ball detected, returning to SEARCHING state");
                current_state = SEARCHING;
            }
            break;
    }
}

void centroidCallback(const vision_msgs::VisionObject::ConstPtr& msg) {
    last_detection_time = ros::Time::now();
    std_msgs::Float32MultiArray head_goal_pose_msg;

    if (current_state != TRACKING) {
        ROS_INFO("Ball detected, switching to TRACKING state");
        current_state = TRACKING;
    }

    if (current_state == TRACKING){
        const int IMAGE_WIDTH = 640;
        const int IMAGE_HEIGHT = 360;

        float 
        center_x = (msg->x+(msg->width/2.0)), 
        center_y = (msg->y+(msg->height/2.0)),

        kpan=0.18/320,
        ktilt = 0.05/180,

        ex = center_x - (IMAGE_WIDTH/2),
        ey = center_y - (IMAGE_HEIGHT/2),
    
        pan_correction = -kpan * ex,
        tilt_correction = ktilt * ey;

        current_robot_pan += pan_correction;
        current_robot_tilt += tilt_correction;

        current_robot_pan = std::max((double)PAN_MIN, std::min((double)PAN_MAX, (double)current_robot_pan));
        current_robot_tilt = std::max((double)TILT_MIN_TRACK, std::min((double)TILT_MAX_TRACK, (double)current_robot_tilt));
        
        std::cout << "Ball Center: "<<center_x<<","<<center_y<<std::endl;
        std::cout<<"Current Head Pose Goal: "<<current_robot_pan<<","<<current_robot_tilt<<std::endl;
        std::cout<<"Error: "<<ex<<","<<ey<<std::endl;
        
        head_goal_pose_msg.data = {current_robot_pan, current_robot_tilt};
        tracker_pub.publish(head_goal_pose_msg);
    }
}

void headPositionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    current_robot_pan = msg->data[18];
    current_robot_tilt = msg->data[19];
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ball_tracker");
    ros::NodeHandle nh;
    last_detection_time = ros::Time::now();
    ros::Subscriber centroid_sub = nh.subscribe("/vision/ball", 1, centroidCallback);
    //filtered_centroid = nh.subscribe("/filtered_centroid",1, centroidCallback);
    tracker_pub = nh.advertise<std_msgs::Float32MultiArray>("/hardware/head_goal_pose", 1);

    ros::Rate loop_rate(10); // Se ejecuta a 10 Hz (sin timers)
    while (ros::ok()) {
        stateMachine();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
