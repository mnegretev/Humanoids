#include <ros/ros.h>
#include <opencv2/opencv.hpp>   


using namespace std;


int h_min = 0;;
int s_min = 0;;
int v_min = 0;;
int h_max = 179;;
int s_max = 255;;
int v_max = 255;;


void set_hsv_values( int event, int x, int y, int, void* param )
{
    if(event == CV_EVENT_LBUTTONDOWN){
        cout<<"hsv_min: ["<<h_min<<", "<<s_min<<", "<<v_min<<"]     hsv_max: ["<<h_max<<", "<<s_max<<", "<<v_max<<"]"<<endl;
        ros::param::set("h_min", h_min);
        ros::param::set("s_min", s_min);
        ros::param::set("v_min", v_min);
        ros::param::set("h_max", h_max);
        ros::param::set("s_max", s_max);
        ros::param::set("v_max", v_max);
    }
}

void hsv_values(int, void*)
{
    //cout<<"h_min:"<<h_min<<" s_min:"<<s_min<<" v_min:"<<v_min<<" H_max:"<<h_max<<" S_max:"<<s_max<<" V_max:"<<v_max<<endl;
}


int main(int argc, char **argv)
{   
    cout<<"Starting ball_tracker_node by Luis Näva..."<<endl;
    ros::init(argc, argv, "ball_tracker_node");


    cv::Mat   video_frame;
    cv::Mat     hsv_frame;
    cv::Mat    mask_frame;
    cv::Mat  eroded_frame;
    cv::Mat dilated_frame;

    cv::Mat kernel=cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));

    cv::VideoCapture capture(1);
    capture.open(1);
    cv::namedWindow("video_frame",1);

    cv::setMouseCallback("video_frame", set_hsv_values, &video_frame); 

    cv::createTrackbar( "h_min", "video_frame", &h_min, 179, hsv_values );    
    cv::createTrackbar( "s_min", "video_frame", &s_min, 255, hsv_values );
    cv::createTrackbar( "v_min", "video_frame", &v_min, 255, hsv_values );
    cv::createTrackbar( "H_max", "video_frame", &h_max, 179, hsv_values );
    cv::createTrackbar( "S_max", "video_frame", &s_max, 255, hsv_values );
    cv::createTrackbar( "V_max", "video_frame", &v_max, 255, hsv_values );

    while ( ros::ok() && cv::waitKey(15)!=27 )
    {
        capture.read( video_frame );
        cv::cvtColor( video_frame , hsv_frame, CV_BGR2HSV);
        cv::inRange(  hsv_frame   , cv::Scalar(h_min, s_min  , v_min), cv::Scalar(h_max, s_max, v_max), mask_frame);
        cv::erode(    mask_frame  , eroded_frame    , kernel , cv::Point(-1,-1), 1);
        cv::dilate(   eroded_frame, dilated_frame   , kernel , cv::Point(-1,-1), 1);

        putText(video_frame, "click to set values", cv::Point(5,50), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar( 143, 30, 90), 2);
        cv::imshow("dilated_frame", dilated_frame);
        cv::imshow("video_frame", video_frame);
    }


    return 0;
}
