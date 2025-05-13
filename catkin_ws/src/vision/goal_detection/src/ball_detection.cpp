#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

ros::Publisher centroid_pub;
ros::Publisher ball_image_pub;
ros::Publisher hue_image_pub;
cv::Point center(-1, -1);
cv::Point prev_center(-1, -1);

float calculate_distance(const cv::Point& p1, const cv::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    std::string image_encoding;
    ros::param::get("image_encoding", image_encoding);
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, image_encoding);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat cv_image = cv_ptr->image;
    cv::Mat gray, blur;

    // Image conversion to space color 
    cv::Mat converted_image;
    bool use_hsv;
    ros::param::get("use_hsv", use_hsv);
    if (use_hsv) {
        cv::cvtColor(cv_image, converted_image, cv::COLOR_BGR2HSV);
    } else {
        cv::cvtColor(cv_image, converted_image, cv::COLOR_BGR2HLS);
    }

    int h_min, h_max, s_min, s_max, v_min, v_max,l_min,l_max;
    if (use_hsv){
    ros::param::get("hsv_h_min", h_min);
    ros::param::get("hsv_h_max", h_max);
    ros::param::get("hsv_s_min", s_min);
    ros::param::get("hsv_s_max", s_max);
    ros::param::get("hsv_v_min", v_min);
    ros::param::get("hsv_v_max", v_max);        
    }
    else{
    ros::param::get("hls_h_min", h_min);
    ros::param::get("hls_h_max", h_max);
    ros::param::get("hls_l_min", l_min);
    ros::param::get("hls_l_max", l_max);
    ros::param::get("hls_s_min", s_min);
    ros::param::get("hls_s_max", s_max);
    }
    
    cv::Scalar lower(h_min, s_min, v_min);
    cv::Scalar upper(h_max, s_max, v_max);
    
    cv::Mat mask;
    cv::inRange(converted_image, lower, upper, mask);
    
    cv::Mat masked_image;
    cv::bitwise_and(cv_image, cv_image, masked_image, mask);
    
    cv::cvtColor(masked_image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(7, 7), 0);

    //Circles params
    double minDist = 500;
    double param1 = 50, param2 = 50;
    int minRadius = 20, maxRadius = 50;
    
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(blur, circles, cv::HOUGH_GRADIENT, 1.5, minDist, param1, param2, minRadius, maxRadius);
    
    geometry_msgs::Point32 centroid_msg;
    bool ball_detected = false;
    float distance_threshold = 5.0;
    
    for (const auto& circle : circles) {
        cv::Point detected_center(cvRound(circle[0]), cvRound(circle[1]));
        int radius = cvRound(circle[2]);
        
        cv::circle(cv_image, detected_center, radius, cv::Scalar(0, 255, 0), 2);
        cv::circle(cv_image, detected_center, 2, cv::Scalar(0, 0, 255), 3);
        
        if (center.x == -1) {
            prev_center = center = detected_center;
        } else {
            prev_center = center;
            center = detected_center;
        }
        
        float distance = calculate_distance(prev_center, center);
        if (distance < distance_threshold) {
            ball_detected = true;
            centroid_msg.x = center.x;
            centroid_msg.y = center.y;
            centroid_pub.publish(centroid_msg);
        }
    }
    
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
    ball_image_pub.publish(output_msg);
    
    sensor_msgs::ImagePtr blur_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", blur).toImageMsg();
    hue_image_pub.publish(blur_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ball_detection");
    ros::NodeHandle nh;
    ros::Subscriber image_sub = nh.subscribe("/hardware/camera/image", 1, imageCallback);
    centroid_pub = nh.advertise<geometry_msgs::Point32>("/centroid_publisher", 1);
    ball_image_pub = nh.advertise<sensor_msgs::Image>("/ball_image", 1);
    hue_image_pub = nh.advertise<sensor_msgs::Image>("/hue_image", 1);
    
    ros::spin();
    return 0;
}
