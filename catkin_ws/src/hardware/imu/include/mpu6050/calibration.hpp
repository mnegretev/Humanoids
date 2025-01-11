#ifndef MPU6050_DRIVER_MPU6050_CALIBRATION_NODE_HPP_
#define MPU6050_DRIVER_MPU6050_CALIBRATION_NODE_HPP_

#include <eigen3/Eigen/Dense>

#include <cstdint>
#include <string>

#include "ros/ros.h"

#include "mpu6050/mpu6050.hpp"
#include "mpu6050/mpu6050_node.hpp"

class MPU6050CalibrationNode : public MPU6050Node {

private:
    void loadParameters();
    void computeOffsets();
    void adjustOffsets();
    void publishOffsets();
    bool isCalibrationFinished();
    ros::NodeHandle nh_;
    ros::Publisher imu_offsets_pub_;
    float kp_;
    float ki_;
    float delta_;
    Eigen::Matrix<float, 3, 2> p_term_matrix_;
    Eigen::Matrix<float, 3, 2> i_term_matrix_;
    Eigen::Matrix<float, 3, 2> error_matrix_;
    Eigen::Matrix<float, 3, 2> offset_matrix_;

public:
    MPU6050CalibrationNode();
    void init();
    void printOffsets();
    void run();

};

#endif  // MPU6050_DRIVER_MPU6050_CALIBRATION_NODE_HPP_
