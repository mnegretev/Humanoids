#include<ros/ros.h>
#include<ros/package.h>
#include<opencv2/opencv.hpp>
#include<std_msgs/UInt8MultiArray.h>
#include<std_msgs/Float32MultiArray.h>


using namespace std;

string file_pkg;
cv::FileStorage  fs;

cv::Mat hsv_min = cv::Mat_<int>::zeros(1, 3);
cv::Mat hsv_max = cv::Mat_<int>::zeros(1, 3);

cv::Mat   video_frame;
cv::Mat tracked_frame;
cv::Mat  ball_located;
cv::Mat dilated_frame;
cv::Mat kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(5,5) );

void callback_img(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{

    video_frame = cv::imdecode(msg->data,1); 
    cv::imshow("Camera visualizer", video_frame);  
    cv::cvtColor( video_frame, dilated_frame, CV_BGR2HSV);  
    cv::inRange(  dilated_frame, hsv_min, hsv_max, dilated_frame); 
    cv::erode( dilated_frame, dilated_frame, kernel, cv::Point(-1, -1), 2);

    video_frame.copyTo(tracked_frame);

        for(int j=0; j<dilated_frame.cols; j++)
            for(int i=0; i<dilated_frame.rows; i++)
                if((int)(dilated_frame).at<uchar>(i, j) == 0){
                    tracked_frame.at<cv::Vec3b>(i, j)[0] = 0;
                    tracked_frame.at<cv::Vec3b>(i, j)[1] = 0;
                    tracked_frame.at<cv::Vec3b>(i, j)[2] = 0;
                }
    cv::dilate( tracked_frame, ball_located, kernel, cv::Point(-1, -1), 1);   
    cv::imshow("Ball located",ball_located);
}


bool get_hsv_values()
{   
    fs.open(file_pkg + "/vision/hsv_values.xml", cv::FileStorage::READ);


    if(!fs.isOpened()){
        cout<<"There's no hsv_values.xml file..."<<endl;
        return false;
    }

    fs["hsv_min"] >> hsv_min;
    fs["hsv_max"] >> hsv_max;
    
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "Starting compressed ball visualizer by Luis Näva..." << std::endl;
    ros::init(argc, argv, "compres_ball_viz_node");
    ros::NodeHandle nh;

	file_pkg = ros::package::getPath("config_files");
    ros::Subscriber subCompressed = nh.subscribe("/hardware/camera/img_compressed", 1, callback_img);
 
    if(!get_hsv_values()){
        ROS_ERROR("Please track colour with: rosrun ball_tracker ball_tracker_node.");
        return -1;
    }

	while(ros::ok() && cv::waitKey(15)!=27)
    	ros::spinOnce();

    return 0;
}
