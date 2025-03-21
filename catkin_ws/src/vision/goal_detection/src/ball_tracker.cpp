#include <iostream>
#include <ros/ros.h>
#include <sensor_msg/Image.h>
#include <geometry_msgs/Point32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <sensor_msgs/image_encodings.h>

ros::Publisher ball_image_pub;
ros::Publisher hue_image_pub;
ros::Publisher centroid_pub;
cv::Point center (-1 -1);
cv::Point prev_center (-1 -1);

float calculate_distance(const cv::Point& center1, const cv::Point& center2){
    return std::sqrt(std::pow(center1.x - center2.x, 2) + std::pow(center1.y - center2.y, 2));
}
void callback_image(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what ());
        return ;
    }
    cv::Mat cv_image = cv_ptr->image;
    cv::Mat gray, blur;

    
}
