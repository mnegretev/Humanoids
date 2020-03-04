#include<ros/ros.h>
#include<ros/package.h>
#include<opencv2/opencv.hpp>
#include<sensor_msgs/Image.h>
#include<std_msgs/UInt8MultiArray.h>
#include<std_msgs/Float32MultiArray.h>


using namespace std;

string file_pkg;

cv::FileStorage  fs;

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


int main(int argc, char **argv)
{
    cout<<"Starting camera_node by Luis Näva..."<<endl;
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    
    int camera = 2;
    if(argc == 2)
        camera = atoi(argv[1]);

    file_pkg = ros::package::getPath("config_files");
    ros::Publisher pubImage = nh.advertise<sensor_msgs::Image>("/hardware/camera/image", 1);
    ros::Publisher pubJpeg  = nh.advertise<std_msgs::UInt8MultiArray>("/hardware/camera/img_compressed", 1);
    ros::Rate loop(30);

    cv::Mat       video_frame;
    cv::VideoCapture capture;
    capture.open(camera);

    sensor_msgs::Image msgImage;
    std_msgs::UInt8MultiArray msgCompressed;

    int img_width  = 640;
    int img_height = 480;
    msgImage.header.frame_id = "camera_link";
    msgImage.data.resize(img_width*img_height*3);
    msgImage.height = img_height;
    msgImage.width  = img_width;
    msgImage.encoding = "bgr8";
    msgImage.step = img_width*3;
    std::vector<int> compressionParams(2);
    std::vector<uchar> compressedBuff;
    compressionParams[0] = cv::IMWRITE_JPEG_QUALITY;
    compressionParams[1] = 95;


    if(!get_camera_parameters())
    {   
        ROS_ERROR("Please calibrate the camera with: rosrun camera_calibrator camera_calibrator_node.");
        return -1;
    }

    while( ros::ok() && cv::waitKey(15)!=27 )
    {
        capture.read(video_frame);

        capture.retrieve(video_frame);
        msgImage.header.stamp = ros::Time::now();
        memcpy(msgImage.data.data(), video_frame.data, img_width*img_height*3);
        cv::imencode(".jpg", video_frame, msgCompressed.data, compressionParams);

		pubImage.publish(msgImage);
        pubJpeg.publish(msgCompressed);

        ros::spinOnce();
        loop.sleep();        
    }



    return 0;
}  
