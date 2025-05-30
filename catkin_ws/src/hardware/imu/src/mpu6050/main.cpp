#include <ros/ros.h>
#include <exception>
#include "mpu6050/mpu6050_node.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpu6050_node");
  ros::NodeHandle nh;

  MPU6050Node mpu_node;

  try {
    mpu_node.init();
    mpu_node.run();
  } catch (std::runtime_error error) {
    ROS_FATAL("%s", error.what());
    ros::shutdown();
  }

  return 0;
}