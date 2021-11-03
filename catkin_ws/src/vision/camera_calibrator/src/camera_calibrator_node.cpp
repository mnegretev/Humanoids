#include<ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    cout<<"Starting camera_calibrator_node by Luis Näva..."<<endl;
    ros::init(argc, argv, "camera_calibrator_node");

    int numBoards = 0;
    int numCornersHor;
    int numCornersVer;
    int i;
    printf("Enter number of corners along width: ");
    scanf("%d", &numCornersHor);

    printf("Enter number of corners along height: ");
    scanf("%d", &numCornersVer);

    printf("Enter number of boards: ");
    scanf("%d", &numBoards);

    cout<<"Press the key 'Space' to set the picture."<<endl;
    

    int numSquares = numCornersHor * numCornersVer;
    Size board_sz = Size(numCornersHor, numCornersVer);

    VideoCapture capture = VideoCapture(1);
  
    vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > image_points;
   
    vector<Point2f> corners;
    int successes=0;  

    Mat image;
    Mat gray_image;
    capture >> image;

    vector<Point3f> obj;
    for(int j=0;j<numSquares;j++)
    	obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));  

    while(successes<numBoards)
    {
        cvtColor(image, gray_image, COLOR_BGR2GRAY);
        bool found = findChessboardCorners(image, board_sz, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

        if(found)
        {
            cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS  | TermCriteria::MAX_ITER, 30, 0.1));
            drawChessboardCorners(gray_image, board_sz, corners, found);
        }
        imshow("Frame for training", gray_image);

        capture >> image;
        int key = waitKey(1);
        
        if(key==27)
            return 0;

        if(key==' ' && found!=0)
        { 
            image_points.push_back(corners);
            object_points.push_back(obj);

            cout<<"Pictures set: "<< successes + 1 <<endl;
            successes++;

            if(successes>=numBoards)
                break;
        }
    }         

    destroyWindow("Frame for training");


    Mat   intrinsic  = Mat_<float>::zeros(3, 3);
    Mat   distCoeffs = Mat_<float>::zeros(1, 5);
    
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;
    calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
    Mat imageUndistorted;

    FileStorage fs("src/config_files/vision/camera_parameters.xml", FileStorage::WRITE);
    fs << "intrinsic"  << intrinsic;
    fs << "distCoeffs" << distCoeffs;
    fs.release();

    cout<<"\nIntrinsic parameters:"<<endl; 
    cout<<intrinsic<<endl;
    cout<<"\nDistortion coefficients: " <<distCoeffs<<endl;

    while(1)
    {
        capture >> image;
        undistort(image, imageUndistorted, intrinsic, distCoeffs);

        imshow("Real image", image);
        imshow("Undistort image", imageUndistorted);
        waitKey(1);
    }        


    capture.release();


    return 0;   
}
    
