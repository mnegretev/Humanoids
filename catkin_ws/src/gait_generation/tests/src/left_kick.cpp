#include<ros/ros.h>
#include<humanoid/Humanoid.h>

using namespace std;

int main(int argc, char **argv) {
    cout << "Starting left_kick test by Luis Näva..." << endl;
    ros::init(argc, argv, "left_kick");
    ros::NodeHandle nh;

    Humanoid::setNodeHandle(&nh);
    Humanoid::leftKick();

    cout << "Left kick test finished" << endl;
    return 0;
}
