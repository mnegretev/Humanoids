#include<ros/ros.h>
#include<humanoid/Humanoid.h>

using namespace std;

int main(int argc, char **argv) {
    cout << "Starting walk test by Luis Näva..." << endl;
    ros::init(argc, argv, "walk");
    ros::NodeHandle nh;

    Humanoid::setNodeHandle(&nh);

    Humanoid::loadWalkPositions();

    //Humanoid::setStepsNumber(10);

    Humanoid::walk();

    cout << "Walk test finished" << endl;
    return 0;
}
