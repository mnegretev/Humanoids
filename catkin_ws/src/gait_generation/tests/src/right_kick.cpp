#include<ros/ros.h>
#include<humanoid/Humanoid.h>

using namespace std;

int main(int argc, char **argv) {
    cout << "Starting right_kick test by Luis Näva..." << endl;
    ros::init(argc, argv, "right_kick");
    ros::NodeHandle nh;

    Humanoid::setNodeHandle(&nh);
    Humanoid::rightKick();

    cout << "Right kick test finished" << endl;


    return 0;
}
