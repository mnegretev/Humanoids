#include <ros/ros.h>
#include <exception>
#include "mpu6050/calibration.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpu6050_calibration_node");
  ros::NodeHandle nh;

  MPU6050CalibrationNode mpu_calibration_node;

  try {
    mpu_calibration_node.init();
    mpu_calibration_node.run();
  } catch (std::runtime_error error) {
    ROS_FATAL("%s", error.what());
    ros::shutdown();
  }

  return 0;
}
