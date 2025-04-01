#ifndef LIPM_H
#define LIPM_H

#include <Eigen/Dense>
#include "ros/ros.h"

namespace LIPM
{

typedef struct state
{
    double x_0;
    double dx_0;
    double dy_0;
    double t;
} state_t ;

struct trajectory_t
{
    Eigen::Vector3d last_p_com;
    Eigen::MatrixXd q_left;
    Eigen::MatrixXd q_right;
};

class StepHandler {
private:
    const double y_feet_gap;
    const double static_height;
    const double walking_height;
    const double step_height;
    const double step_length;
    const double com_x_offset;
    const double com_y_offset;

    const double MODEL_SAMPLE_TIME;
    const double SERVO_SAMPLE_TIME;

    ros::ServiceClient left_ik_srv;
    ros::ServiceClient right_ik_srv;

    ros::Publisher  legs_goal_pose;

public:

    StepHandler(const double & gravity,
         const double & y_feet_gap, 
         const double & z_robot_walk,
         const double & z_robot_static,
         const double & step_length,
         const double & step_height,
         const double & com_x_offset,
         const double & com_y_offset,
         const double & lipm_sample_time,
         const double & servo_sample_time);

    Eigen::MatrixXd getFootSwingTrajectory( const Eigen::Vector3d & initial_foot_position,
                                            const Eigen::Vector3d & final_foot_position,
                                            const double & swing_height,
                                            const std::vector<double> & time_vector);
    
    Eigen::MatrixXd request_ik( const Eigen::MatrixXd & P, ros::ServiceClient & service);
    
    state_t findInitialConditionsLIPM(  const double step_length, 
                                        const double x_vel,
                                        const double y_foot_pos,
                                        const double z_model,
                                        const double G);

    trajectory_t get_right_start(const double displacement_ratio);
    trajectory_t get_left_start(const double displacement_ratio);

    trajectory_t get_right_first_step(const Eigen::Vector3d p_start, const Eigen::Vector3d p_end);
    trajectory_t get_left_first_step(const Eigen::Vector3d p_start, const Eigen::Vector3d p_end);

    trajectory_t get_right_full_step(const Eigen::Vector3d initial_l_foot_pos);
    trajectory_t get_left_full_step(const Eigen::Vector3d initial_r_foot_pos);
};

} //namespace LIPM

#endif // LIPM_H

