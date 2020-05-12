#include<ros/ros.h>
#include<humanoid/Humanoid.h>

using namespace std;

int main(int argc, char **argv) {
    cout << "Starting prone_get_up test by Luis Näva..." << endl;
    ros::init(argc, argv, "prone_get_up");
    ros::NodeHandle nh;

    Humanoid::setNodeHandle(&nh);
    Humanoid::proneGetUp();

    cout << "prone_get_up test finished" << endl;


    return 0;
}
