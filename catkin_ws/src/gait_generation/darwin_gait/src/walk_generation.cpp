#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv){

  ROS_INFO("Starting darwin_walk_generation node from Miguel")
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  
  struct Rhoban::IKWalkParameters params;
  
  if(fillWalkParameters(params)) return -1;
  
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}


bool fillWalkParameters(const struct Rhoban::IKWalkParameters& params)
{
        if(!ros::param::get("/walk/dist_hip_to_knee", params.distHipToKnee))
        {
            ROS_ERROR("Missing param in config file: /walk/dist_hip_to_knee");
            return false;
        }
        if(!ros::param::get("/walk/dist_knee_to_ankle", params.distKneeToAnkle))
        {
            ROS_ERROR("Missing param in config file: /walk/dist_knee_to_ankle");
            return false;
        }
        if(!ros::param::get("/walk/dist_ankle_to_ground", params.distAnkleToGround))
        {
            ROS_ERROR("Missing param in config file: /walk/dist_ankle_to_ground");
            return false;
        }
        if(!ros::param::get("/walk/dist_feet_lateral", params.distFeetLateral))
        {
            ROS_ERROR("Missing param in config file: /walk/dist_feet_lateral");
            return false;
        }
        if(!ros::param::get("/walk/frequency", params.freq))
        {
            ROS_ERROR("Missing param in config file: /walk/frequency");
            return false;
        }
        if(!ros::param::get("/walk/enabled_gain", params.enabledGain))
        {
            ROS_ERROR("Missing param in config file: /walk/enabled_gain");
            return false;
        }
        if(!ros::param::get("/walk/support_phase_ratio", params.supportPhaseRatio))
        {
            ROS_ERROR("Missing param in config file: /walk/support_phase_ratio");
            return false;
        }
        if(!ros::param::get("/walk/foot_y_offset", params.footYOffset))
        {
            ROS_ERROR("Missing param in config file: /walk/foot_y_offset");
            return false;
        }
        if(!ros::param::get("/walk/step_gain", params.stepGain))
        {
            ROS_ERROR("Missing param in config file /walk/step_gain");
            return false;
        }
        if(!ros::param::get("/walk/rise_gain", params.riseGain))
        {
            ROS_ERROR("Missing param in config file: /walk/rise_gain");
            return false;
        }
        if(!ros::param::get("/walk/turn_gain", params.turnGain))
        {
            ROS_ERROR("Missing param in config file: /walk/turn_gain");
            return false;
        }
        if(!ros::param::get("/walk/lateral_gain", params.lateralGain))
        {
            ROS_ERROR("Missing param in config file: /walk/lateral_gain");
            return false;
        }
        if(!ros::param::get("/walk/trunk_z_offset", params.trunkZOffset))
        {
            ROS_ERROR("Missing param in config file: /walk/trunk_z_offset");
            return false;
        }
        if(!ros::param::get("/walk/swing_gain", params.swingGain))
        {
            ROS_ERROR("Missing param in config file: /walk/swing_gain");
            return false;
        }
        if(!ros::param::get("/walk/swing_roll_gain", params.swingRollGain))
        {
            ROS_ERROR("Missing param in config file: ");
            return false;
        }
        if(!ros::param::get("/walk/swing_phase", params.swingPhase))
        {
            ROS_ERROR("Missing param in config file: /walk/swing_phase");
            return false;
        }
        if(!ros::param::get("/walk/step_up_vel", params.stepUpVel))
        {
            ROS_ERROR("Missing param in config file: /walk/step_up_vel");
            return false;
        }
        if(!ros::param::get("/walk/step_down_vel", params.stepDownVel))
        {
            ROS_ERROR("Missing param in config file: /walk/step_down_vel");
            return false;
        }
        if(!ros::param::get("/walk/rise_up_vel", params.riseUpVel))
        {
            ROS_ERROR("Missing param in config file: /walk/rise_up_vel");
            return false;
        }
        if(!ros::param::get("/walk/swing_pause", params.swingPause))
        {
            ROS_ERROR("Missing param in config file: /walk/swingPause")
            return false;
        }
        if(!ros::param::get("/walk/swing_vel", params.swingVel))
        {
            ROS_ERROR("Missing param in config file: /walk/swing_vel");
            return false:
        }
        if(!ros::param::get("/walk/trunk_x_offset", params.trunkXOffset))
        {
            ROS_ERROR("Missing param in config file: /walk/trunk_x_offset");
            return false;
        }
        if(!ros::param::get("/walk/trunk_y_offset", params.trunkYOffset))
        {
            ROS_ERROR("Missing param in config file: /walk/trunk_y_offset");
            return false;
        }
        if(!ros::param::get("/walk/trunk_pitch", params.trunkPitch))
        {
            ROS_ERROR("Missing param in config file: /walk/trunk_pitch");
            return false;
        }
        if(!ros::param::get("/walk/trunk_roll", params.trunkRoll))
        {
            ROS_ERROR("Missing param in config file: /walk/trunk_roll");
            return false;
        }
        if(!ros::param::get("/walk/extra_left_x", params.extraLeftX))
        {
            ROS_ERROR("Missing param in config file: /walk/extra_left_x");
            return false;
        }
        if(!ros::param::get("/walk/extra_left_y", params.extraLeftY))
        {
            ROS_ERROR("Missing param in config file: /walk/extra_left_y");
            return false;
        }
        if(!ros::param::get("/walk/extra_left_z", params.extraLeftZ))
        {
            ROS_ERROR("Missing param in config file: /walk/extra_left_z");
            return false;
        }
         if(!ros::param::get("/walk/extra_right_x", params.extraRightX))
        {
            ROS_ERROR("Missing param in config file: /walk/extra_right_x")
            return false;
        }
        if(!ros::param::get("/walk/extra_right_y", params.extraRightY))
        {
            ROS_ERROR("Missing param in config file: /walk/extra_right_y");
            return false;
        }
        if(!ros::param::get("/walk/extra_right_z", params.extraRightZ))
        {
            ROS_ERROR("Missing param in config file: /walk/extra_right_z");
            return false;
        }
        if(!ros::param::get("/walk/extra_left_yaw", params.extraLeftYaw))
        {
            ROS_ERROR("Missing param in config file: /walk/extra_left_yaw");
            return false;
        }
        if(!ros::param::get("/walk/extra_left_pitch", params.extraLeftPitch))
        {
            ROS_ERROR("Missing param in config file: /walk/extra_left_pitch");
            return false;
        }
        if(!ros::param::get("/walk/extra_left_roll", params.extraLeftRoll))
        {
            ROS_ERROR("Missing param in config file: /walk/extra_left_roll");
            return false;
        }
        if(!ros::param::get("/walk/extra_right_yaw", params.extraRightYaw))
        {
            ROS_ERROR("Missing param in config file: /walk/extra_right_yaw")
            return false;
        }
        if(!ros::param::get("/walk/extra_right_pitch", params.extraRightPitch))
        {
            ROS_ERROR("Missing param in config file: /walk/extra_right_pitch");
            return false;
        }
        if(!ros::param::get("/walk/extra_right_roll", params.extraRightRoll))
        {
            ROS_ERROR("Missing param in config file: /walk/extra_right_roll");
            return false;
        }
}
