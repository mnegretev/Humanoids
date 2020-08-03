#include<ros/ros.h>
#include<ros/package.h>
#include<opencv2/opencv.hpp>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<tf/transform_listener.h>
#include<std_msgs/Float32MultiArray.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#define ball_radius 0.095


using namespace std;


double px, py, pz;
double e_x , e_y, v;
double x, y, theta, psi;
double roll, pitch, yaw;

double vertical_view, horizontal_view;
int camera_resolution[2] = {640, 480};


string file_pkg;
ros::Publisher pub_centroid_angle;
ros::Publisher  pub_ball_position;

cv::Scalar centroid;
cv::Mat   video_frame;
cv::Mat tracked_frame;
cv::Mat  ball_located;
vector<cv::Point> pixel_point;
cv::Mat kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(5, 5));

cv::Mat hsv_min = cv::Mat_<int>::zeros(1, 3);
cv::Mat hsv_max = cv::Mat_<int>::zeros(1, 3);

bool get_hsv_values() {

    cv::FileStorage fs;
    fs.open(file_pkg + "/vision/hsv_values.xml", cv::FileStorage::READ);

    if(!fs.isOpened()) { 
        cout << "There's no hsv_values.xml file..." << endl;
        return false;
    }

    fs["hsv_min"] >> hsv_min;
    fs["hsv_max"] >> hsv_max;

    return true;
}

void get_centroid_angles() {

    std_msgs::Float32MultiArray centroid_angle;
    centroid_angle.data.resize(2);

    //cout << "x_pix->" << centroid[0] << "\ty_pix->" << centroid[1] << endl;
    centroid[0] =   horizontal_view * (centroid[0] - 0.5 * camera_resolution[0]) / (0.5 * camera_resolution[0]);
    centroid[1] = - vertical_view   * (centroid[1] - 0.5 * camera_resolution[1]) / (0.5 * camera_resolution[1]);

    centroid_angle.data[0] = centroid[0]; 
    centroid_angle.data[1] = centroid[1];

    if(centroid[0] != -1 && centroid[1] != -1)
        pub_centroid_angle.publish(centroid_angle);
}

void compute_ball_position() { 

    std_msgs::Float32MultiArray position_msg;
    position_msg.data.resize(2);

    psi   = -centroid[1] + pitch;
    theta = -centroid[0] +  yaw ;
    //cout<<"psi: "<<psi<<"\ttheta: "<<theta<<endl;
    x = px - pz * tan(1.5708 + psi) * cos(theta) - ball_radius * cos(theta) / tan(psi);
    y = py - pz * tan(1.5708 + psi) * sin(theta) - ball_radius * sin(theta) / tan(psi);

    position_msg.data[0] = x;
    position_msg.data[1] = y;

    if(centroid[0] != - horizontal_view && centroid[1] != - vertical_view)
        pub_ball_position.publish(position_msg);    
}

void callback_img(const sensor_msgs::ImageConstPtr& msg) {
    
    //ROS_INFO("height = %d, width = %d",msg->height, msg->width);
    
    video_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("Simulation view", video_frame);
    cv::cvtColor(video_frame, tracked_frame, CV_BGR2HSV);
    cv::inRange( tracked_frame,  hsv_min, hsv_max, tracked_frame);
    cv::erode(   tracked_frame, tracked_frame, kernel, cv::Point(-1, -1), 2);
    cv::dilate(  tracked_frame, ball_located, kernel, cv::Point(-1, -1), 1);
    cv::imshow( "Ball located",ball_located);

    cv::findNonZero(ball_located, pixel_point);
    centroid = cv::mean(pixel_point);

    get_centroid_angles();
   compute_ball_position();
}

int main(int argc, char**argv) {

    cout << "Starting ball_position_simul by Luis Näva..." << endl;
    ros::init(argc, argv, "ball_position_simul");
    ros::NodeHandle nh("~");
    
    file_pkg = ros::package::getPath("config_files");

    pub_centroid_angle = nh.advertise<std_msgs::Float32MultiArray>("/vision/ball_position/centroid_angle", 1000);
    pub_ball_position  = nh.advertise<std_msgs::Float32MultiArray>("/vision/ball_position/ball_position",   1);
    
    nh.param("vertical_view", vertical_view, 0.6981);
    nh.param("horizontal_view", horizontal_view, 0.6981);

    if(!get_hsv_values()){
        ROS_ERROR("Please track colour with: rosrun ball_tracker ball_trac    ker_node.");
        return -1;
    }        
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/hardware/camera/image", 1, callback_img); 


    tf::TransformListener listener;

    while(ros::ok() && cv::waitKey(15)!=27) {

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
