#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32MultiArray.h"

std_msgs::Float32MultiArray msgArmLeftGoalPose;
std_msgs::Float32MultiArray msgArmRightGoalPose;

float acc_x, acc_y, acc_z;
double theta, phi, rho;

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    std::cout << "x: " << (float)msg->linear_acceleration.x << std::endl;
    std::cout << "y: " << (float)msg->linear_acceleration.y << std::endl;
    std::cout << "z: " << (float)msg->linear_acceleration.z << std::endl;
    
    acc_x = (float)msg->linear_acceleration.x;
    acc_y = (float)msg->linear_acceleration.y;
    acc_z = (float)msg->linear_acceleration.z;

    theta   = atan2(acc_y, acc_x);
    rho     = sqrt(acc_x*acc_x + acc_y*acc_y);
    phi     = atan(acc_z/theta);

    std::cout << "theta: " << theta << std::endl;
    std::cout << "rho: " << rho << std::endl;
    std::cout << "phi: " << phi << std::endl;

}

int main(int argc, char** argv){
    std::cout << "\n---- ARMS ARE AWAKE ------\n" << std::endl;
    ros::init(argc, argv, "arms_stabilizer");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("imu", 1, ImuCallback);
    
    ros::Publisher pubArmLeftGoalPose  = n.advertise<std_msgs::Float32MultiArray>("/hardware/arm_left_goal_pose", 1); 
    ros::Publisher pubArmRightGoalPose = n.advertise<std_msgs::Float32MultiArray>("/hardware/arm_right_goal_pose", 1);
    
    while(ros::ok()){
        if (acc_x > 5){
            msgArmLeftGoalPose.data = {M_PI/2,0,0};
            msgArmRightGoalPose.data = {M_PI/2,0,0};
        }
        else if (acc_x < -5){
            msgArmLeftGoalPose.data = {-M_PI/2,0,0};
            msgArmRightGoalPose.data = {-M_PI/2,0,0};
        }
        else{
            msgArmLeftGoalPose.data = {0,0,0};
            msgArmRightGoalPose.data = {0,0,0};
        }
        pubArmRightGoalPose.publish(msgArmRightGoalPose);
        pubArmLeftGoalPose.publish(msgArmLeftGoalPose);
        ros::spinOnce();
    }

    return 0;         
}