#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

ros::Publisher pub_head_goal;
ros::Publisher centroid_pub;
ros::Publisher original_cv_image_pub;
ros::Publisher goal_image_pub;
ros::Publisher hue_image_pub;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat cv_image = cv_ptr->image;
    // Convert to HSV and mask
    cv::Mat hsv_image;
    cv::cvtColor(cv_image, hsv_image, cv::COLOR_RGB2HSV);

    cv::Mat gray_image;
    cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);

    cv::Mat mask;
    cv::inRange(hsv_image, cv::Scalar(0, 0, 0), cv::Scalar(179, 255, 80), mask);

    cv::Mat image_mask;
    cv::bitwise_and(cv_image, cv_image, image_mask, mask);

    // Gaussian Blur + Canny
    cv::Mat blur;
    cv::GaussianBlur(mask, blur, cv::Size(7,7), 0);
    cv::Mat edges;
    cv::Canny(blur, edges, 380, 430);

    cv::Mat cdstP = cv::Mat::ones(edges.size(), CV_8UC3);
    // Hough Lines Probabilistic
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180, 115, 200, 100);
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 2, cv::LINE_AA);
    }
    cv::Mat contour_output = cv::Mat::zeros(cdstP.size(), CV_8UC3);

    // Find contours
    cv::Mat gray;
    cv::cvtColor(cdstP, gray, cv::COLOR_RGB2GRAY);
    cv::Mat thresh;
    cv::threshold(gray, thresh, 50, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::drawContours(contour_output, contours, -1, cv::Scalar(0,0,250), 2, cv::LINE_AA);

    // Find target contour
    std::vector<cv::Point> target_contour;
    bool found = false;
    for (const auto& contour : contours) {
        cv::Moments M = cv::moments(contour);
        if (M.m00 == 0) continue;
        cv::Mat hu_moments;
        cv::HuMoments(M, hu_moments);
        std::cout<< hu_moments << std::endl;
        if (0.19 > hu_moments.at <double> (0) > 0.18  &&
            0.005 > hu_moments.at <double>( 1) > 0.004  ||
            6.8 > hu_moments.at <double> (0) > 2.2||
            7.6 > hu_moments.at <double> (1) > 5.7
            ) 
            {

            target_contour = contour;
            found = true;
            break;
        }
    }

    if (found) {
        cv::Moments M = cv::moments(target_contour);
        int cx = int(M.m10 / M.m00);
        int cy = int(M.m01 / M.m00);

        ROS_INFO("Centroid: (%d, %d)", cx, cy);

        cv::drawContours(contour_output, std::vector<std::vector<cv::Point>>{target_contour}, -1, cv::Scalar(255,0,0), 2, cv::LINE_AA);
        cv::circle(contour_output, cv::Point(cx, cy), 3, cv::Scalar(0,255,0), -1);

        int error_x = cx - 320;
        int error_y = cy - 240;

    //     float goal_pan = -1.0/10000.0 * error_x;
    //     float goal_tilt = 1.0/7000.0 * error_y;

    //     goal_pan = std::max(std::min(goal_pan, 1.5f), -1.5f);
    //     goal_tilt = std::max(std::min(goal_tilt, 1.0f), -1.0f);

        // Publish centroid
        geometry_msgs::Point32 centroid_msg;
        centroid_msg.x = cx;
        centroid_msg.y = cy;
        centroid_pub.publish(centroid_msg);
    }

    // Publish original image
    sensor_msgs::ImagePtr msg_pub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
    original_cv_image_pub.publish(msg_pub);

    sensor_msgs::ImagePtr goal_output = cv_bridge::CvImage(std_msgs::Header(), "bgr8", contour_output).toImageMsg();
    goal_image_pub.publish(goal_output);
    
    sensor_msgs::ImagePtr blur_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", blur).toImageMsg();
    hue_image_pub.publish(blur_msg);
}

int main(int argc, char** argv)
{
    std::cout<<"Initializing goal detector..."<<std::endl;
    ros::init(argc, argv, "goal_detector");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/hardware/camera/image", 1, imageCallback);
    pub_head_goal = nh.advertise<std_msgs::Float32MultiArray>("/hardware/head_goal_pose", 1);
    centroid_pub = nh.advertise<geometry_msgs::Point32>("/centroid_goal_publisher", 1);
    original_cv_image_pub = nh.advertise<sensor_msgs::Image>("/original_image", 1);
    goal_image_pub = nh.advertise<sensor_msgs::Image>("/ball_image", 1);
    hue_image_pub = nh.advertise<sensor_msgs::Image>("/hue_image", 1);

    ros::spin();
    return 0;
}
