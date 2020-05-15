#include <ros/ros.h>
#include <humanoid/Humanoid.h>

using namespace std;

int main(int argc, char** argv) {
    cout << "Starting walk_and_kick by Luis Näva..." << endl;
    ros::init(argc, argv, "walk_and_kick");
    ros::NodeHandle nh;

    Humanoid::setNodeHandle(&nh);
    Humanoid::loadWalkPositions();
    Humanoid::setStepsNumber(15);

    Humanoid::walk();

    Humanoid::leftKick();

    return 0;
}
