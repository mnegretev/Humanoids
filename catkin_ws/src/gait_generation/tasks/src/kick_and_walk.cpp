#include <ros/ros.h>
#include <humanoid/Humanoid.h>

using namespace std;

int main(int argc, char** argv) {
    cout << "Starting kick_and_walk by Luis Näva..." << endl;
    ros::init(argc, argv, "kick_and_walk");
    ros::NodeHandle nh;

    Humanoid::setNodeHandle(&nh);
    Humanoid::loadWalkPositions();
    Humanoid::setStepsNumber(15);

    Humanoid::leftKick();
    Humanoid::walk();

    return 0;
}
