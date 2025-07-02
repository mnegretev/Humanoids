#include "WalkHandler.hpp"
#include <std_msgs/Float32MultiArray.h>
#include "darwin_gait/WalkGains.h"
#include "IKWalk.hpp"
#include <ros/ros.h>

WalkNode::WalkNode(ros::NodeHandle& nh, int rate = 40) :
    nh_(nh), rate_(ros::Rate(rate))
{
    legs_pub_       = nh_.advertise<std_msgs::Float32MultiArray>("/hardware/legs_goal_pose", 1);
    arms_pub_       = nh_.advertise<std_msgs::Float32MultiArray>("/hardware/arms_goal_pose", 1);
    walk_server_    = nh_.advertiseService("walk_engine", &WalkNode::handleSetGains, this);
    
    ROS_INFO("========= Starting ROS Walk Node by M1Gu3l ==========");
}

bool WalkNode::start()
{
    if(! fillHumanoidParameters())
    {
        return false;
    }
    return true;
}

bool WalkNode::runWalk(
    double timeLength,
    double phase,
    double time)
{
    //Leg motor computed positions
    struct Rhoban::IKWalkOutputs outputs;
    std_msgs::Float32MultiArray msg;
    msg.data.clear();
    //Walk engine frequency
    double engineFrequency = 40.0;

    for (double t=0.0;t<=timeLength;t+=1.0/engineFrequency) {
        time += 1.0/engineFrequency;
        bool success = Rhoban::IKWalk::walk(
            params_, //Walk parameters
            1.0/engineFrequency, //Time step
            phase, //Current walk phase -will be updated)
            outputs); //Result target position (updated)
        if (!success) {
            //The requested position for left or right foot is not feasible
            //(phase is not updated)
            std::cout << time << " Inverse Kinematics error. Position not reachable." << std::endl;
            return false;
        } else {
            std::cout << time << " ";
            std::cout << phase << " ";
            msg.data =
            {
                /* LEFT */
                (float)outputs.left_hip_yaw,
                (float)outputs.left_hip_roll,
                (float)outputs.left_hip_pitch,
                (float)outputs.left_knee,
                (float)outputs.left_ankle_pitch,
                (float)outputs.left_ankle_roll,
                /* RIGHT */
                (float)outputs.right_hip_yaw,
                (float)outputs.right_hip_roll,
                (float)outputs.right_hip_pitch,
                (float)outputs.right_knee,
                (float)outputs.right_ankle_pitch,
                (float)outputs.right_ankle_roll
            };
            std::cout << std::endl;
            legs_pub_.publish(msg);
            rate_.sleep();
        }
    }
    return true;
}

bool WalkNode::fillHumanoidParameters()
{
    ROS_INFO("Attempting to read 'walk' parameters one by one...");

    // Humanoid Shape parameters
    if(!getParamWithLog("/walk/distHipToKnee",      params_.distHipToKnee, 0.0))          return false;
    if(!getParamWithLog("/walk/distKneeToAnkle",    params_.distKneeToAnkle, 0.0))      return false;
    if(!getParamWithLog("/walk/distAnkleToGround",  params_.distAnkleToGround, 0.0))  return false;
    if(!getParamWithLog("/walk/distFeetLateral",    params_.distFeetLateral, 0.0))      return false;

    //Walking cycle parameters
    if(!getParamWithLog("/walk/frequency",          params_.freq, 0.0)) return false;
    if(!getParamWithLog("/walk/enabledGain",        params_.enabledGain, 0.0)) return false;
    if(!getParamWithLog("/walk/supportPhaseRatio",  params_.supportPhaseRatio, 0.0)) return false;
    if(!getParamWithLog("/walk/footYOffset",        params_.footYOffset, 0.0)) return false;
    if(!getParamWithLog("/walk/stepGain",           params_.stepGain, 0.0)) return false;
    if(!getParamWithLog("/walk/riseGain",           params_.riseGain, 0.0)) return false;
    if(!getParamWithLog("/walk/turnGain",           params_.turnGain, 0.0)) return false;
    if(!getParamWithLog("/walk/lateralGain",        params_.lateralGain, 0.0)) return false;
    if(!getParamWithLog("/walk/trunkZOffset",       params_.trunkZOffset, 0.0)) return false;
    if(!getParamWithLog("/walk/swingGain",          params_.swingGain, 0.0)) return false;
    if(!getParamWithLog("/walk/swingRollGain",      params_.swingRollGain, 0.0)) return false;
    if(!getParamWithLog("/walk/swingPhase",         params_.swingPhase, 0.0)) return false;
    if(!getParamWithLog("/walk/stepUpVel",          params_.stepUpVel, 0.0)) return false;
    if(!getParamWithLog("/walk/stepDownVel",        params_.stepDownVel, 0.0)) return false;
    if(!getParamWithLog("/walk/riseUpVel",          params_.riseUpVel, 0.0)) return false;
    if(!getParamWithLog("/walk/riseDownVel",        params_.riseDownVel, 0.0)) return false;
    if(!getParamWithLog("/walk/swingPause",         params_.swingPause, 0.0)) return false;
    if(!getParamWithLog("/walk/swingVel",           params_.swingVel, 0.0)) return false;
    if(!getParamWithLog("/walk/trunkXOffset",       params_.trunkXOffset, 0.0)) return false;
    if(!getParamWithLog("/walk/trunkYOffset",       params_.trunkYOffset, 0.0)) return false;
    if(!getParamWithLog("/walk/trunkPitch",         params_.trunkPitch, 0.0)) return false;
    if(!getParamWithLog("/walk/trunkRoll",          params_.trunkRoll, 0.0)) return false;

    // Extra offsets on X, Y, Z for feet (index 26 onwards)
    if(!getParamWithLog("/walk/extraLeftX",         params_.extraLeftX, 0.0)) return false;
    if(!getParamWithLog("/walk/extraLeftY",         params_.extraLeftY, 0.0)) return false;
    if(!getParamWithLog("/walk/extraLeftZ",         params_.extraLeftZ, 0.0)) return false;
    if(!getParamWithLog("/walk/extraRightX",        params_.extraRightX, 0.0)) return false;
    if(!getParamWithLog("/walk/extraRightY",        params_.extraRightY, 0.0)) return false;
    if(!getParamWithLog("/walk/extraRightZ",        params_.extraRightZ, 0.0)) return false;

    // Extra angular offset of roll, pitch, yaw on left and right foot (index 32 onwards)
    if(!getParamWithLog("/walk/extraLeftYaw",       params_.extraLeftYaw, 0.0)) return false;
    if(!getParamWithLog("/walk/extraLeftPitch",     params_.extraLeftPitch, 0.0)) return false;
    if(!getParamWithLog("/walk/extraLeftRoll",      params_.extraLeftRoll, 0.0)) return false;
    if(!getParamWithLog("/walk/extraRightYaw",      params_.extraRightYaw, 0.0)) return false;
    if(!getParamWithLog("/walk/extraRightPitch",    params_.extraRightPitch, 0.0)) return false;
    if(!getParamWithLog("/walk/extraRightRoll",     params_.extraRightRoll, 0.0)) return false;

    ROS_INFO("WALKING PARAMS FETCHED SUCCESFULLY");
    return true;
}

bool WalkNode::handleSetGains(darwin_gait::WalkGains::Request& req,
                              darwin_gait::WalkGains::Response& res) {
    
    std_msgs::Float32MultiArray arms_msg;
    arms_msg.data.clear();
    std::vector<float> arms_data = {1.0, 0.3, -1.8707, 1.0, -0.3, -1.8707};
    arms_msg.data = arms_data;
    arms_pub_.publish(arms_msg);
    rate_.sleep();

    float phase = 0.0;
    float start_time = 0.0;

    params_.enabledGain = 0.0;
    params_.stepGain    = 0.0;
    params_.lateralGain = 0.0;
    params_.turnGain    = 0.0;
    
    if(!runWalk(1.0, phase, start_time))
    {
        res.success = false;
        return false;
    }

    params_.enabledGain = 1.0;
    params_.stepGain    = 0.0;
    params_.lateralGain = 0.0;
    params_.turnGain    = 0.0;
    
    if(!runWalk(2.0, phase, start_time))
    {
        res.success = false;
        return false;
    }

    params_.enabledGain = req.enabled_gain;
    params_.stepGain    = req.step_gain;
    params_.lateralGain = req.lateral_gain;
    params_.turnGain    = req.turn_gain;

    if(!runWalk(req.time, phase, start_time))
    {
        res.success = false;
        return false;
    }

    res.success = true;
    return true;
}

template <typename T>
bool getParamWithLog(const std::string& param_name, T& value, const T& default_value)
{
    if (ros::param::get(param_name, value))
    {
        ROS_INFO("  Read parameter: %s = %f", param_name.c_str(), static_cast<double>(value));
        return true;
    }
    else
    {
        value = default_value;
        ROS_WARN("  Failed to read parameter: %s. Using default value: %f", param_name.c_str(), static_cast<double>(default_value));
        return false;
    }
}