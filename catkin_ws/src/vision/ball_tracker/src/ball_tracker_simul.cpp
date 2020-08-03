#include<ros/ros.h>
#include<ros/package.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>


using namespace std;

string file_pkg;
cv::Mat   video_frame;
cv::Mat     hsv_frame;
cv::Mat    mask_frame;
cv::Mat  eroded_frame;
cv::Mat dilated_frame;
cv::Mat tracked_frame;
cv::Mat kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(5, 5));

int h_min = 0;;
int s_min = 0;;
int v_min = 0;;
int h_max = 179;;
int s_max = 255;;
int v_max = 255;;

vector<int> H,S,V;

cv::Mat hsv_min = cv::Mat_<int>::zeros(1, 3);
cv::Mat hsv_max = cv::Mat_<int>::zeros(1, 3);

void hsv_values(int, void*) { 
    //cout<<"h_min:"<<h_min<<" s_min:"<<s_min<<" v_min:"<<v_min<<" H_max:"<<h_max<<" S_max:"<<s_max<<" V_max:"<<v_max<<endl;
}

bool log_out;

void set_hsv_values( int event, int x, int y, int, void* param ) {

    if(event == CV_EVENT_MBUTTONDOWN){
        cv::FileStorage fs("src/config_files/vision/hsv_values.xml", cv::FileStorage::WRITE);
        hsv_min.at<int>(0,0) = h_min;
        hsv_min.at<int>(0,1) = s_min;
        hsv_min.at<int>(0,2) = v_min;

        hsv_max.at<int>(0,0) = h_max;
        hsv_max.at<int>(0,1) = s_max;
        hsv_max.at<int>(0,2) = v_max;

        fs << "hsv_min" << hsv_min;
        fs << "hsv_max" << hsv_max;
        fs.release();

        cout<<"hsv_min: ["<<h_min<<", "<<s_min<<", "<<v_min<<"]   hsv_max: ["<<h_max<<", "<<s_max
            <<", "<<v_max<<"]---> Added to hsv_values.xml"<<endl;

        log_out = true;
    }
}

void callback_img(const sensor_msgs::ImageConstPtr& msg) {
    
    cv::VideoCapture capture;    
    cv::namedWindow("video_frame",1);
    cv::setMouseCallback("video_frame", set_hsv_values, &video_frame);

    cv::createTrackbar( "h_min", "video_frame", &h_min, 179, hsv_values );
    cv::createTrackbar( "s_min", "video_frame", &s_min, 255, hsv_values );    
    cv::createTrackbar( "v_min", "video_frame", &v_min, 255, hsv_values );
    cv::createTrackbar( "H_max", "video_frame", &h_max, 179, hsv_values );
    cv::createTrackbar( "S_max", "video_frame", &s_max, 255, hsv_values );
    cv::createTrackbar( "V_max", "video_frame", &v_max, 255, hsv_values );

    video_frame = cv_bridge::toCvShare(msg, "bgr8")->image;


    cv::cvtColor( video_frame , hsv_frame, CV_BGR2HSV);
    cv::inRange(  hsv_frame   , cv::Scalar(h_min, s_min  , v_min), cv::Scalar(h_max, s_max, v_max), mask_frame);
    cv::erode(    mask_frame  , eroded_frame    , kernel , cv::Point(-1,-1), 1);
    cv::dilate(   eroded_frame, dilated_frame   , kernel , cv::Point(-1,-1), 1);

    putText(video_frame, "click on scroll to set values", cv::Point(5,50), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar( 143, 30, 90), 2);

    cv::imshow("video_frame", video_frame);
    cv::imshow("dilated_frame", dilated_frame);
    
    cv::waitKey(15)!=27;
}


int main(int argc, char**argv) {

    cout << "Starting ball_tracker_simul by Luis Näva..." << endl;
    ros::init(argc, argv, "ball_tracker_simul");
    ros::NodeHandle nh;
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/hardware/camera/image", 1, callback_img); 


    while(ros::ok() && cv::waitKey(15)!=27) {
        
       ros::spinOnce();

       if(log_out)
           break;
    }


    return 0;
}
