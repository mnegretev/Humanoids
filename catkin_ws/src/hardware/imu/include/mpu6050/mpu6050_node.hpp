#ifndef MPU6050_DRIVER_MPU6050_NODE_HPP_
#define MPU6050_DRIVER_MPU6050_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "mpu6050/mpu6050.hpp"

/**
 * @brief Provide MPU6050 data to ROS nodes
 * 
 * This class intend to act as middle layer between the MPU6050 library and ROS
 * applications. Thus, this class basically apply the basics settings to
 * comunicate with the MPU6050 sensor and then get data from de sensor and make it
 * available from ROS interfaces 
 * 1
 */
class MPU6050Node {
 public:

  MPU6050Node();
  void init();
  void run();
  void publishMPUData();

  template <typename T>
  static void getParameterHelper(const ros::NodeHandle &nh, std::string param_name, T *param, T default_value) {
    if (!nh.param<T>(param_name, *param, default_value)) {
      ROS_WARN_STREAM_NAMED("mpu6050_ros_driver", "No " << param_name <<
      " parameter found in the parameter server. Using default parameter value: " << default_value);
    } else {
      ROS_INFO_STREAM("Parameter " << param_name << " found and it value is " << *param);
    }
  }

 protected:
    void loadParameters();
    void setMPUOffsets();
    ros::NodeHandle nh_;
    ros::Publisher mpu_data_pub_;
    MPU6050 mpu6050_;
    int mpu6050_addr_;
    std::string i2c_bus_uri_;
    float pub_rate_;
    std::string imu_frame_id_;
    std::vector<int> axes_offsets_;
};

#endif  // MPU6050_DRIVER_MPU6050_NODE_HPP_
