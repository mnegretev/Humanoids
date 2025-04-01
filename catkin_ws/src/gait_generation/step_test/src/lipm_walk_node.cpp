#include "ros/ros.h"
#include "lipm.h"

int main(int argc, char* argv[])
{
    ROS_INFO("INITIALIZING LIPM WALK NODE BY MIGUEL GARCIA");
    ros::init(argc, argv, "lipm_walk_node");
    ros::NodeHandle nh;
    
    double  gravity_param, y_body_to_feet, z_robot_walk, z_robot_static, 
            step_height, step_length, robot_vel_x, com_x_offset, 
            com_y_offset, lipm_sample_time, servo_sample_time;
    
    std::map<std::string, double&> lipm_params =
    {
        {"/gait/lipm/gravity"           , gravity_param},
        {"/gait/lipm/y_body_to_feet"    , y_body_to_feet},
        {"/gait/lipm/z_robot_walk"      , z_robot_walk},
        {"/gait/lipm/z_robot_static"    , z_robot_static},
        {"/gait/lipm/step_height"       , step_height},
        {"/gait/lipm/step_length"       , step_length},
        {"/gait/lipm/com_x_offset"      , com_x_offset},
        {"/gait/lipm/com_y_offset"      , com_y_offset},
        {"/gait/lipm/sample_time"       , lipm_sample_time},
        {"/gait/lipm/servo_sample_time" , servo_sample_time},
    };

    try
    {
        for(auto &param: lipm_params)
        {
            if (nh.getParam(param.first, param.second)) {
                ROS_INFO("%s is:\t\t %f", param.first.c_str(), param.second);
            } else {
                std::stringstream ss;
                ss << "Param " << param.first << " not found. Aborting";
                throw std::invalid_argument(ss.str());
            }
        }
    }
    catch (const std::invalid_argument & e)
    {
        std::cerr << "Exception captured"<< e.what() << std::endl;
        return 0;
    }

    //All params found. Continue


    return 0;
}