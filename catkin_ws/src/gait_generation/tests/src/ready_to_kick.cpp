#include<ros/ros.h>
#include<humanoid/Humanoid.h>

using namespace std;

int main(int argc, char **argv) {
    cout << "Starting ready_to_kick test by Luis Näva..." << endl;
    ros::init(argc, argv, "ready_to_kick");
    ros::NodeHandle nh;

    Humanoid::setNodeHandle(&nh);
    Humanoid::readyToKick();

    cout << "Ready to kick test finished" << endl;

    return 0;
}
