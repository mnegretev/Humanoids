#include "ros/ros.h"
#include <thread>
#include "undistort_point_service.h"

bool threadTerminating = false;

void threadFunction(void)
{
   try
   {
       undistort_point_service();
   }
   catch (std::runtime_error e)
   {
       std::cout << "Caught exception: " << e.what() << std::endl;
   }
   catch (...)
   {
       std::cout << "Caught unknown exception, terminating the program." << std::endl;
   }
    threadTerminating = true;
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "undistort_point_service");
    ros::NodeHandlePtr MLROSNodePtr = ros::NodeHandlePtr(new ros::NodeHandle);
    std::thread threadObj(threadFunction);

    ros::spin();
    if (threadTerminating) {
    threadObj.join();
    }

    return 0;
}
