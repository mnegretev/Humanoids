#include <iostream>
#include "IKWalk.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <ros/ros.h>
/**
 * Run the walk for given among of time and update
 * phase and time state
 */
static void runWalk(
    const Rhoban::IKWalkParameters& params, 
    double timeLength, 
    double& phase, 
    double& time,
    ros::Publisher& pub,
    ros::Rate&      rate)
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
            params, //Walk parameters
            1.0/engineFrequency, //Time step
            phase, //Current walk phase -will be updated)
            outputs); //Result target position (updated)
        if (!success) {
            //The requested position for left or right foot is not feasible
            //(phase is not updated)
            std::cout << time << " Inverse Kinematics error. Position not reachable." << std::endl;
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
            pub.publish(msg);
            rate.sleep();
        }
    }
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

bool fillHumanoidParameters(struct Rhoban::IKWalkParameters& params)
{
    ROS_INFO("Attempting to read 'walk' parameters one by one...");

    // Humanoid Shape parameters
    if(!getParamWithLog("/walk/distHipToKnee", params.distHipToKnee, 0.0))          return false;
    if(!getParamWithLog("/walk/distKneeToAnkle", params.distKneeToAnkle, 0.0))      return false;
    if(!getParamWithLog("/walk/distAnkleToGround", params.distAnkleToGround, 0.0))  return false;
    if(!getParamWithLog("/walk/distFeetLateral", params.distFeetLateral, 0.0))      return false;

    //Walking cycle parameters
    if(!getParamWithLog("/walk/frequency",          params.freq, 0.0)) return false;
    if(!getParamWithLog("/walk/enabledGain",        params.enabledGain, 0.0)) return false;
    if(!getParamWithLog("/walk/supportPhaseRatio",  params.supportPhaseRatio, 0.0)) return false;
    if(!getParamWithLog("/walk/footYOffset",        params.footYOffset, 0.0)) return false;
    if(!getParamWithLog("/walk/stepGain",           params.stepGain, 0.0)) return false;
    if(!getParamWithLog("/walk/riseGain",           params.riseGain, 0.0)) return false;
    if(!getParamWithLog("/walk/turnGain",           params.turnGain, 0.0)) return false;
    if(!getParamWithLog("/walk/lateralGain",        params.lateralGain, 0.0)) return false;
    if(!getParamWithLog("/walk/trunkZOffset",       params.trunkZOffset, 0.0)) return false;
    if(!getParamWithLog("/walk/swingGain",          params.swingGain, 0.0)) return false;
    if(!getParamWithLog("/walk/swingRollGain",      params.swingRollGain, 0.0)) return false;
    if(!getParamWithLog("/walk/swingPhase",         params.swingPhase, 0.0)) return false;
    if(!getParamWithLog("/walk/stepUpVel",          params.stepUpVel, 0.0)) return false;
    if(!getParamWithLog("/walk/stepDownVel",        params.stepDownVel, 0.0)) return false;
    if(!getParamWithLog("/walk/riseUpVel",          params.riseUpVel, 0.0)) return false;
    if(!getParamWithLog("/walk/riseDownVel",        params.riseDownVel, 0.0)) return false;
    if(!getParamWithLog("/walk/swingPause",         params.swingPause, 0.0)) return false;
    if(!getParamWithLog("/walk/swingVel",           params.swingVel, 0.0)) return false;
    if(!getParamWithLog("/walk/trunkXOffset",       params.trunkXOffset, 0.0)) return false;
    if(!getParamWithLog("/walk/trunkYOffset",       params.trunkYOffset, 0.0)) return false;
    if(!getParamWithLog("/walk/trunkPitch",         params.trunkPitch, 0.0)) return false;
    if(!getParamWithLog("/walk/trunkRoll",          params.trunkRoll, 0.0)) return false;

    // Extra offsets on X, Y, Z for feet (index 26 onwards)
    if(!getParamWithLog("/walk/extraLeftX",         params.extraLeftX, 0.0)) return false;
    if(!getParamWithLog("/walk/extraLeftY",         params.extraLeftY, 0.0)) return false;
    if(!getParamWithLog("/walk/extraLeftZ",         params.extraLeftZ, 0.0)) return false;
    if(!getParamWithLog("/walk/extraRightX",        params.extraRightX, 0.0)) return false;
    if(!getParamWithLog("/walk/extraRightY",        params.extraRightY, 0.0)) return false;
    if(!getParamWithLog("/walk/extraRightZ",        params.extraRightZ, 0.0)) return false;

    // Extra angular offset of roll, pitch, yaw on left and right foot (index 32 onwards)
    if(!getParamWithLog("/walk/extraLeftYaw",       params.extraLeftYaw, 0.0)) return false;
    if(!getParamWithLog("/walk/extraLeftPitch",     params.extraLeftPitch, 0.0)) return false;
    if(!getParamWithLog("/walk/extraLeftRoll",      params.extraLeftRoll, 0.0)) return false;
    if(!getParamWithLog("/walk/extraRightYaw",      params.extraRightYaw, 0.0)) return false;
    if(!getParamWithLog("/walk/extraRightPitch",    params.extraRightPitch, 0.0)) return false;
    if(!getParamWithLog("/walk/extraRightRoll",     params.extraRightRoll, 0.0)) return false;

    ROS_INFO("WALKING PARAMS FETCHED SUCCESFULLY");
    return true;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "walk_node");
    ros::NodeHandle n;
    ros::Publisher legs_pub = n.advertise<std_msgs::Float32MultiArray>("/hardware/legs_goal_pose", 1);
    ros::Rate rate(40); // 1 Hz

    struct Rhoban::IKWalkParameters params;
    
    if(! fillHumanoidParameters(params))
    {
        return 0;
    }

    double phase = 0.0;
    double time = 0.0;

    while(ros::ok())
    {
            //The walk is stopped
        params.enabledGain = 0.0;
        params.stepGain = 0.0;
        params.lateralGain = 0.0;
        params.turnGain = 0.0;
        runWalk(params, 2.0, phase, time, legs_pub, rate);

        //The walk is started while walking on place
        params.enabledGain = 1.0;
        params.stepGain = 0.0;
        params.lateralGain = 0.0;
        params.turnGain = 0.0;
        runWalk(params, 10.0, phase, time, legs_pub, rate);
    }
    return 0;
}

