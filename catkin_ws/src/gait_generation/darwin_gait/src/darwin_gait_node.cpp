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

int main(int argc, char** argv)
{

    ros::init(argc, argv, "walk_node");
    ros::NodeHandle n;
    ros::Publisher legs_pub = n.advertise<std_msgs::Float32MultiArray>("/hardware/legs_goal_pose", 1);
    ros::Rate rate(40); // 1 Hz

    struct Rhoban::IKWalkParameters params;
    
    //if fillHumanoidParameters(struct Rhoban::IKWalkParameters params)

    /**
     * Model leg typical length between
     * each rotation axis
     */
    params.distHipToKnee = 0.093;
    params.distKneeToAnkle = 0.93;
    params.distAnkleToGround = 0.035;
    /**
     * Distance between the two feet in lateral
     * axis while in zero position
     */
    params.distFeetLateral = 0.072;
    /**
     * Complete (two legs) walk cycle frequency
     * in Hertz
     */
    params.freq = 1.7;
    /**
     * Global gain multiplying all time
     * dependant movement between 0 and 1.
     * Control walk enabled/disabled smoothing.
     * 0 is walk disabled.
     * 1 is walk fully enabled
     */
    params.enabledGain = 1.0;
    /**
     * Length of double support phase
     * in phase time
     * (between 0 and 1)
     * 0 is null double support and full single support
     * 1 is full double support and null single support
     */
    params.supportPhaseRatio = 0.2;
    /**
     * Lateral offset on default foot 
     * position in meters (foot lateral distance)
     * 0 is default
     * > 0 is both feet external offset
     */
    params.footYOffset = 0.025;
    /**
     * Forward length of each foot step
     * in meters
     * >0 goes forward
     * <0 goes backward
     * (dynamic parameter)
     */
    params.stepGain = 0.0;
    /**
     * Vertical rise height of each foot
     * in meters (positive)
     */
    params.riseGain = 0.05;
    /**
     * Angular yaw rotation of each 
     * foot for each step in radian.
     * 0 does not turn
     * >0 turns left
     * <0 turns right
     * (dynamic parameter)
     */
    params.turnGain = 0.0;
    /**
     * Lateral length of each foot step
     * in meters.
     * >0 goes left
     * <0 goes right
     * (dynamic parameter)
     */
    params.lateralGain = 0.0;
    /**
     * Vertical foot offset from trunk 
     * in meters (positive)
     * 0 is in init position
     * > 0 set the robot lower to the ground
     */
    params.trunkZOffset = 0.02;
    /**
     * Lateral trunk oscillation amplitude
     * in meters (positive)
     */
    params.swingGain = 0.02;
    /**
     * Lateral angular oscillation amplitude
     * of swing trunkRoll in radian
     */
    params.swingRollGain = 0.0;
    /**
     * Phase shift of lateral trunk oscillation
     * between 0 and 1
     */
    params.swingPhase = 0.25;
    /**
     * Foot X-Z spline velocities
     * at ground take off and ground landing.
     * Step stands for X and rise stands for Z
     * velocities.
     * Typical values ranges within 0 and 5.
     * >0 for DownVel is having the foot touching the
     * ground with backward velocity.
     * >0 for UpVel is having the foot going back
     * forward with non perpendicular tangent.
     */
    params.stepUpVel = 4.0;
    params.stepDownVel = 4.0;
    params.riseUpVel = 4.0;
    params.riseDownVel = 4.0;
    /**
     * Time length in phase time
     * where swing lateral oscillation
     * remains on the same side
     * between 0 and 0.5
     */
    params.swingPause = 0.0;
    /**
     * Swing lateral spline velocity (positive).
     * Control the "smoothness" of swing trajectory.
     * Typical values are between 0 and 5.
     */
    params.swingVel = 4.0;
    /**
     * Forward trunk-foot offset 
     * with respect to foot in meters
     * >0 moves the trunk forward
     * <0 moves the trunk backward
     */
    params.trunkXOffset = 0.1;
    /**
     * Lateral trunk-foot offset
     * with respect to foot in meters
     * >0 moves the trunk on the left
     * <0 moves the trunk on the right
     */
    params.trunkYOffset = 0.0;
    /**
     * Trunk angular rotation
     * around Y in radian
     * >0 bends the trunk forward
     * <0 bends the trunk backward
     */
    params.trunkPitch = 0.15;
    /**
     * Trunk angular rotation
     * around X in radian
     * >0 bends the trunk on the right
     * <0 bends the trunk on the left
     */
    params.trunkRoll = 0.0;
    /**
     * Add extra offset on X, Y and Z
     * direction on left and right feet
     * in meters
     * (Can be used for example to implement 
     * dynamic kick)
     */
    params.extraLeftX = 0.0;
    params.extraLeftY = 0.0;
    params.extraLeftZ = 0.0;
    params.extraRightX = 0.0;
    params.extraRightY = 0.0;
    params.extraRightZ = 0.0;
    /**
     * Add extra angular offset on
     * Yaw, Pitch and Roll rotation of 
     * left and right foot in radians
     */
    params.extraLeftYaw = 0.0;
    params.extraLeftPitch = 0.0;
    params.extraLeftRoll = 0.0;
    params.extraRightYaw = 0.0;
    params.extraRightPitch = 0.0;
    params.extraRightRoll = 0.0;

    double phase = 0.0;
    double time = 0.0;

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
    runWalk(params, 2.0, phase, time, legs_pub, rate);

    //Walk forward
    params.enabledGain = 1.0;
    params.stepGain = 0.02;
    params.lateralGain = 0.0;
    params.turnGain = 0.0;
    runWalk(params, 2.0, phase, time, legs_pub, rate);

    //Walk on the left with lateral steps
    params.enabledGain = 1.0;
    params.stepGain = 0.0;
    params.lateralGain = 0.02;
    params.turnGain = 0.0;
    runWalk(params, 2.0, phase, time, legs_pub, rate);
    
    //Turn on the right
    params.enabledGain = 1.0;
    params.stepGain = 0.0;
    params.lateralGain = 0.0;
    params.turnGain = -0.1;
    runWalk(params, 2.0, phase, time, legs_pub, rate);

    //Stop the walk
    params.enabledGain = 0.0;
    params.stepGain = 0.0;
    params.lateralGain = 0.0;
    params.turnGain = 0.0;
    runWalk(params, 2.0, phase, time, legs_pub, rate);

    return 0;
}

