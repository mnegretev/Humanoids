#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "ctrl_msgs/CalculateIK.h"
#include "ctrl_msgs/CalculateDK.h"


Eigen::MatrixXd request_ik( const std::vector<Eigen::Vector3d>& P,
                            ros::ServiceClient & srv_client)
{
    Eigen::MatrixXd joint_values(P.size(), 6);
    ctrl_msgs::CalculateIK srv;
    for(auto vector: P)
    {
        srv.request.x = vector.x();
        srv.request.y = vector.y();
        srv.request.z = vector.z();
        srv.request.roll = 0.0;
        srv.request.pitch= 0.0;
        srv.request.yaw  = 0.0;
        if (srv_client.call(srv))
        {
            if(srv.response.joint_values.size() == 6)
            {
                 
            }
            else
            {

            }
        }
    }
    return joint_values;
}

// Solve LIPM initial condition to create a symmetric trajectory
state_t findInitialConditionsLIPM(  const double step_length, 
                                    const double x_vel,
                                    const double y_foot_pos,
                                    const double z_model,
                                    const double G)
{
    
    // Desired midstance and state
    double s = std::sqrt(z_model / G);
    double x_mid = 0;
    
    //Corresponding orbital energy is
    double E = -G / (2 * z_model) * std::pow(x_mid,2) + 0.5 * std::pow(x_vel,2);
    double x_0 = -step_length / 2;
    
    //Calculating initial velocity (dx0, dy0) and LIPM swing time
    double dx0 = std::sqrt(2 * (E + G / (2 * z_model) * (x_0 * x_0)));
    double singlesupport_t = 2 * std::asinh(step_length / 2 / (s * x_vel)) * s;
    double tf = singlesupport_t / 2;
    double dy0 = -y_foot_pos / s * std::sinh(tf / s) / std::cosh(tf / s);

    state_t lipm_initial_state =
    {
        x_0,  // street_no
        dx0,
        dy0,
        singlesupport_t
    };

    return lipm_initial_state;
}

Eigen::MatrixXd getFootSwingTrajectory( const Eigen::Vector3d initial_foot_position,
                                        const Eigen::Vector3d final_foot_position,
                                        const double swing_height,
                                        const std::vector<double> time_vector)
{
    double x_0 = initial_foot_position.x();
    double x_f = final_foot_position.x();

    double y_0 = initial_foot_position.y();
    double y_f = final_foot_position.y();

    //Calculating parabola coefficients for swinging foot
    //Using canonical equation  y = a(x-h)^2 + k
    double h = x_0 + (x_f - x_0)/2;
    double k = swing_height;
    double a = -k/std::pow((x_0-h), 2);
    double m_x = (x_f - x_0)/(time_vector.back() - time_vector.front());
    double m_y = (y_f - y_0)/(time_vector.back() - time_vector.front());

    auto x_t  = [&](double t) -> double {return x_0 + m_x*t; };
    auto y_t  = [&](double t) -> double {return y_0 + m_y*t; };
    auto z    = [&](double x) -> double {return a*std::pow((x-h),2) + k; };

    
    std::vector<Eigen::Vector3d> swing_foot_trajectory;
    for(auto i: time_vector)
    {
        Eigen::Vector3d aux(x_t(i), y_t(i), z(x_t(i)));
        swing_foot_trajectory.push_back(aux);
    }

    return swing_foot_trajectory;
}

trajectory_t get_right_start_pose(const double displacement_ratio)