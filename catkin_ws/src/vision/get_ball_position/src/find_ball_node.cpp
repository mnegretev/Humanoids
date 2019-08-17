#include<ros/ros.h>
#include<ros/package.h>
#include<opencv2/opencv.hpp>
#include<std_msgs/Float32MultiArray.h>

#define vertical_view    0.280000
#define horizontal_view  0.460000

using namespace std;

string file_pkg;
int h_min, h_max, s_min, s_max, v_min, v_max;
vector<cv::Point> position;

cv::FileStorage  fs;
cv::Mat hsv_min = cv::Mat_<int>::zeros(1, 3);
cv::Mat hsv_max = cv::Mat_<int>::zeros(1, 3);

cv::Mat   intrinsic = cv::Mat_<float>::zeros(3, 3);
cv::Mat dist_coeffs = cv::Mat_<float>::zeros(1, 5);


bool get_camera_parameters()
{
    fs.open(file_pkg + "/vision/camera_parameters.xml", cv::FileStorage::READ);
    
    if(!fs.isOpened()){
        cout<<"There's no camera_parameters.xml file..."<<endl;
        return false;
    }

    fs["intrinsic"]   >> intrinsic;
    fs["distCoeffs"] >> dist_coeffs;
    
    return true;
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


int main(int argc, char **argv)
{
    cout<<"Starting find_ball_node by Luis Näva..."<<endl;
    ros::init(argc, argv, "find_ball_node");
    ros::NodeHandle nh;

    file_pkg = ros::package::getPath("config_files");
    ros::Publisher pub_angles = nh.advertise<std_msgs::Float32MultiArray>("/vision/get_ball_position/vision_angles", 1000);

    cv::Mat       video_frame;
    cv::Mat     tracked_frame;
    cv::Mat    target_located;
    cv::Mat video_undistorted;
    cv::Mat kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(5,5) );

    std_msgs::Float32MultiArray msg;
    msg.data.resize(2);
    ros::Rate loop(50);


    cv::VideoCapture capture;
    capture.open(1);

    if(!get_camera_parameters())
    {   
        ROS_ERROR("Please calibrate the camera with: rosrun camera_calibrator camera_calibrator_node.");
        return -1;
    }
 
    if(!get_hsv_values()){
        ROS_ERROR("Please track colour with: rosrun ball_tracker ball_tracker_node.");
        return -1;
    }


    while( ros::ok() && cv::waitKey(15)!=27 )
    {
        capture.read(video_frame);
        undistort(video_frame, video_undistorted, intrinsic, dist_coeffs);

        video_undistorted.copyTo(target_located);
        cv::cvtColor(video_undistorted,  tracked_frame   , CV_BGR2HSV);
        cv::inRange(  tracked_frame   ,  hsv_min         , hsv_max, tracked_frame); 
        cv::erode(    tracked_frame   ,  tracked_frame   , kernel , cv::Point(-1,-1), 1);
        cv::dilate(   tracked_frame   ,  tracked_frame   , kernel , cv::Point(-1,-1), 1);

        for(int j=0; j<tracked_frame.cols; j++)
            for(int i=0; i<tracked_frame.rows; i++)
                if((int)(tracked_frame).at<uchar>(i, j) == 0)
                {
                    target_located.at<cv::Vec3b>(i, j)[0] = 0;
                    target_located.at<cv::Vec3b>(i, j)[1] = 0;
                    target_located.at<cv::Vec3b>(i, j)[2] = 0;
                }
        
        cv::findNonZero(tracked_frame, position);
        cv::Scalar centroid = cv::mean( position);
        centroid[0] =  - horizontal_view * ( centroid[0] - 320 ) / 320;
        centroid[1] =    vertical_view   * ( centroid[1] - 240 ) / 240;
        
        msg.data[0] = centroid[0];
        msg.data[1] = centroid[1];


        cv::imshow("video_undistorted", video_undistorted);
        cv::imshow("target_located"   ,  target_located  );
        
        if( centroid[0] != horizontal_view   && centroid[1] != - vertical_view )    
            pub_angles.publish(msg);
       
       
        ros::spinOnce();
        loop.sleep();        
    }



    return 0;
}  
