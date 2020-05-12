#include<ros/ros.h>
#include<humanoid/Humanoid.h>

using namespace std;

int main(int argc, char **argv) {
    cout << "Starting supine_get_up test by Luis Näva..." << endl;
    ros::init(argc, argv, "supine_get_up");
    ros::NodeHandle nh;

    Humanoid::setNodeHandle(&nh);
    Humanoid::supineGetUp();

    cout << "Supine get up test finished" << endl;
    return 0;
}
