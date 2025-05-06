#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>


const double PAN_MIN = -M_PI / 2.1;
const double PAN_MAX = M_PI / 2.1;
const double TILT_MIN = 0.0;
const double TILT_MAX = 1.0;

ros::Publisher tracker_pub;
ros::Time last_detection_time;
double timeout = 5.0;  
float pan_robot = 0.0, tilt_robot = 0.0;
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
            if (current_tilt < TILT_MAX) current_tilt += 0.2;
        }
    } else {
        current_pan -= 0.0001;
        if (current_pan <= PAN_MIN) {
            current_pan = PAN_MIN;
            increasing_pan = true;
            if (current_tilt < TILT_MAX) current_tilt += 0.2;
        }
    }

    search_point.x = current_pan;
    search_point.y = current_tilt;
    last_search_pan = current_pan;
    last_search_tilt = current_tilt;
    return search_point;
}

void stateMachine() {
    ros::Time current_time = ros::Time::now();
    double time_since_last_detection = (current_time - last_detection_time).toSec();
    std_msgs::Float32MultiArray head_goal_pose_msg;

    switch (current_state) {
        case SEARCHING:
            if (time_since_last_detection > timeout) {
                geometry_msgs::Point32 search_point = getSearchTrajectory();
                head_goal_pose_msg.data = {search_point.x, search_point.y};
                tracker_pub.publish(head_goal_pose_msg);
                ROS_INFO("Searching... pan: %.2f, tilt: %.2f", search_point.x, search_point.y);
            }
            break;

        case TRACKING:
            if (time_since_last_detection > timeout) {
                ROS_WARN("Lost goal, switching to LOST state");
                current_state = LOST;
            }
            break;

        case LOST:
            if (time_since_last_detection > timeout + 3.0) { 
                ROS_WARN("No goal detected, returning to SEARCHING state");
                current_state = SEARCHING;
            }
            break;
    }
}

void centroidCallback(const geometry_msgs::Point32::ConstPtr& msg) {
    last_detection_time = ros::Time::now();
    std_msgs::Float32MultiArray head_goal_pose_msg;
    pan_robot= last_search_pan;
    tilt_robot= last_search_tilt;
    float 
    center_x = msg->x, 
    center_y = msg ->y,
    kpan=0.05/320,
    ktilt = 0.025/240;

    int 
    Cx=320,
    Cy=240,
    ex = center_x-Cx, 
    ey = center_y-Cy;

    float
    pan = -kpan * ex,
    tilt = ktilt * ey;

    pan_robot += pan;
    tilt_robot += tilt;
    last_search_pan = pan_robot;
    last_search_tilt = tilt_robot;
    std::cout << "Center:"<<center_x<<","<<center_y<<std::endl;
    std::cout<<"Coordenadas:"<<pan_robot<<","<<tilt_robot<<std::endl;
    //std::cout<<"Error***:"<<ex<<","<<ey<<std::endl;
    head_goal_pose_msg.data = {pan_robot, tilt_robot};
    float 
    height = 0.7, 
    tilt_d = head_goal_pose_msg.data[1], 
    pan_d = head_goal_pose_msg.data[0];
    const double
    pi = 3.141592653589793;
    std::cout<<"Distance to the goal: "<< height / std::tan(tilt_d*1.155) <<std::endl;
    tracker_pub.publish(head_goal_pose_msg);

}

void headPositionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    pan_robot = msg->data[18];
    tilt_robot = msg->data[19];
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_tracker");
    ros::NodeHandle nh;
    last_detection_time = ros::Time::now();
    ros::Subscriber centroid_sub = nh.subscribe("/centroid_goal_publisher", 1, centroidCallback);
    tracker_pub = nh.advertise<std_msgs::Float32MultiArray>("/hardware/head_goal_pose", 1);

    ros::Rate loop_rate(10); // Se ejecuta a 10 Hz (sin timers)
    while (ros::ok()) {
        stateMachine();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
