#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include "ros/ros.h"

struct state_t
{
    double x_0;
    double dx_0;
    double dy_0;
    double t;
};


int main(int argc, char* argv[])
{
    ROS_INFO("INITIALIZING LIPM WALK NODE BY MIGUEL GARCIA");
    ros::init(argc, argv, "lipm_walk_node");
    ros::NodeHandle n;



    return 0;
}

// Solve LIPM initial condition to create a symmetric trajectory
state_t findInitialConditions(  const double step_length, 
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