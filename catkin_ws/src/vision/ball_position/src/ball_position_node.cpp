#include<ros/ros.h>
#include<ros/package.h>
#include<opencv2/opencv.hpp>
#include<tf/transform_listener.h>
#include<std_msgs/UInt8MultiArray.h>
#include<std_msgs/Float32MultiArray.h>

//#define ball_radius  0.03156
#define ball_radius 0.093106

using namespace std;


string file_pkg;
ros::Publisher pub_centroid;
ros::Publisher pub_ball_position;
    
cv::Scalar centroid;
cv::Mat hsv_min = cv::Mat_<int>::zeros(1, 3);
cv::Mat hsv_max = cv::Mat_<int>::zeros(1, 3);

cv::Mat   video_frame;
cv::Mat tracked_frame;
cv::Mat  ball_located;
vector<cv::Point> pixel_point;
cv::Mat kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(5,5) );

int camera_resolution[2] = {640, 480};
double vertical_view;
double horizontal_view;


double px, py,  pz;
double e_x , e_y, v;
double x, y, theta, psi;
double roll, pitch, yaw;


bool get_hsv_values()
{   
    cv::FileStorage  fs;
    fs.open(file_pkg + "/vision/hsv_values.xml", cv::FileStorage::READ);


    if(!fs.isOpened()){
        cout<<"There's no hsv_values.xml file..."<<endl;
        return false;
    }

    fs["hsv_min"] >> hsv_min;
    fs["hsv_max"] >> hsv_max;
    
    return true;
}

void get_centroid()
{   
    std_msgs::Float32MultiArray centroid_position;
    centroid_position.data.resize(2);
    
    //cout<<"x_pix:"<<centroid[0]<<" y_pix:"<<centroid[1]<<endl;
    centroid[0] =   ( centroid[0] - 0.5 * camera_resolution[0] ) / (0.5 * camera_resolution[0]);
    centroid[1] = - ( centroid[1] - 0.5 * camera_resolution[1] ) / (0.5 * camera_resolution[1]);

    centroid_position.data[0] = centroid[0];
    centroid_position.data[1] = centroid[1];


    if(centroid[0] != -1 && centroid[1] != -1 )    
        pub_centroid.publish(centroid_position);

    centroid[0] = horizontal_view * centroid[0];
    centroid[1] = vertical_view   * centroid[1];
}

void compute_ball_position()
{

    std_msgs::Float32MultiArray position_msg;
    position_msg.data.resize(2);

    psi   =  -centroid[1] + pitch;
    theta =  -centroid[0] +  yaw ;
    //cout<<"psi: "<<psi<<"\ttheta: "<<theta<<endl;

    e_x =  -1.3848*pow(pitch, 4) + 4.63*pow(pitch,3) - 5.691*pow(pitch,2) + 3.055*pitch - 0.6504;
    x = px - pz * tan(1.5708 + psi) * cos(theta)  -  ball_radius * cos(theta) / tan(psi)  +  e_x;
    v = sqrt( pow(x, 2) + pow(pz, 2) );    
    y = v * tan(theta) + e_y;

    position_msg.data[0] = x;
    position_msg.data[1] = y;
    if(centroid[0] != -horizontal_view && centroid[1] != -vertical_view)
    	pub_ball_position.publish(position_msg);   
}

void callback_img(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{

    video_frame = cv::imdecode(msg->data,1); 
    //GaussianBlur(video_frame, video_frame, cv::Size(15,15), 0);    
    cv::imshow( "Camera visualizer", video_frame);  
    cv::cvtColor(video_frame, tracked_frame, cv::COLOR_BGR2HSV);  
    cv::inRange( tracked_frame,  hsv_min, hsv_max, tracked_frame); 
    cv::erode(   tracked_frame, tracked_frame, kernel, cv::Point(-1, -1), 2);
    cv::dilate(  tracked_frame, ball_located, kernel, cv::Point(-1, -1), 1);   
    cv::imshow( "Ball located",ball_located);


    cv::findNonZero(ball_located, pixel_point);
    centroid = cv::mean(pixel_point);

    get_centroid();
    compute_ball_position();
}


int main(int argc, char** argv)
{
    std::cout << "Starting ball_position_node by Luis Näva..." << std::endl;
    ros::init(argc, argv, "ball_position_node");
    ros::NodeHandle nh("~");
    ros::Rate loop(30);

    file_pkg = ros::package::getPath("config_files");
    ros::Subscriber sub_compressed_image = nh.subscribe("/hardware/camera/img_compressed",  1,  callback_img);
    pub_centroid = nh.advertise<std_msgs::Float32MultiArray>("/vision/ball_position/centroid_position", 1000);
    pub_ball_position = nh.advertise<std_msgs::Float32MultiArray>("/vision/ball_position/ball_position",   1);    

    nh.param("vertical_view", vertical_view, 0.39);
    nh.param("horizontal_view", horizontal_view, 0.46);

    if(!get_hsv_values()){
        ROS_ERROR("Please track colour with: rosrun ball_tracker ball_tracker_node.");
        return -1;
    }


    tf::TransformListener listener;

    while(ros::ok() && cv::waitKey(15)!=27){
        
        tf::StampedTransform transform;

        listener.waitForTransform("/right_foot_plane_link", "/camera_optical", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform( "/right_foot_plane_link", "/camera_optical", ros::Time(0), transform);
        transform.getBasis().getRPY(roll, pitch, yaw);

        px = transform.getOrigin().x();
        py = transform.getOrigin().y();
        pz = transform.getOrigin().z();

        //cout<<"px: "<<px<<"\tpy: "<<py<<"\tpz: "<<pz<<endl;
        //cout<<"roll: "<<roll<<"\tpitch: "<<pitch<<"\tyaw: "<<yaw<<endl;
        ros::spinOnce();
    }

    return 0;
}
