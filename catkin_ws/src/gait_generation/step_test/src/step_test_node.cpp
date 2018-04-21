#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "control_msgs/CalculateIK.h"
#include "control_msgs/CalculateDK.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING STEP TEST NODE BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "step_test");
    ros::NodeHandle n;
    ros::Rate loop(40);

    ros::Publisher pubLegLeftGoalPose  = n.advertise<std_msgs::Float32MultiArray>("/hardware/leg_left_goal_pose", 1); 
    ros::Publisher pubLegRightGoalPose = n.advertise<std_msgs::Float32MultiArray>("/hardware/leg_right_goal_pose", 1);
    ros::Publisher pubArmLeftGoalPose  = n.advertise<std_msgs::Float32MultiArray>("/hardware/arm_left_goal_pose", 1); 
    ros::Publisher pubArmRightGoalPose = n.advertise<std_msgs::Float32MultiArray>("/hardware/arm_right_goal_pose", 1);
    ros::Publisher pubHeadGoalPose     = n.advertise<std_msgs::Float32MultiArray>("/hardware/head_goal_pose", 1);
    ros::ServiceClient cltCalculateIKLegLeft  = n.serviceClient<control_msgs::CalculateIK>("/control/ik_leg_left"); 
    ros::ServiceClient cltCalculateIKLegRight = n.serviceClient<control_msgs::CalculateIK>("/control/ik_leg_right");
    ros::ServiceClient cltCalculateDKLegLeft  = n.serviceClient<control_msgs::CalculateDK>("/control/dk_leg_left"); 
    ros::ServiceClient cltCalculateDKLegRight = n.serviceClient<control_msgs::CalculateDK>("/control/dk_leg_right");
    std_msgs::Float32MultiArray msgLegLeftGoalPose;
    std_msgs::Float32MultiArray msgLegRightGoalPose;
    std_msgs::Float32MultiArray msgArmLeftGoalPose;
    std_msgs::Float32MultiArray msgArmRightGoalPose;
    std_msgs::Float32MultiArray msgHeadGoalPose;
    control_msgs::CalculateIK srvIK;

    //
    //Trajectory generation
    //
    float left_start_x  = -0.036;
    float left_start_y  =  0.080;
    float left_start_z  = -0.552;
    float right_start_x = -0.036;
    float right_start_y = -0.080;
    float right_start_z = -0.552;
    float t = 0;
    std::vector<float> leg_left_x     ;
    std::vector<float> leg_left_y     ;
    std::vector<float> leg_left_z     ;
    std::vector<float> leg_left_roll  ;
    std::vector<float> leg_left_pitch ;
    std::vector<float> leg_left_yaw   ;
    std::vector<float> leg_right_x    ;
    std::vector<float> leg_right_y    ;
    std::vector<float> leg_right_z    ;
    std::vector<float> leg_right_roll ;
    std::vector<float> leg_right_pitch;
    std::vector<float> leg_right_yaw  ;
    std::vector<std::vector<float> > leg_left_angles;
    std::vector<std::vector<float> > leg_right_angles;
    const int number_of_points = 32;
    leg_left_x     .resize(number_of_points);
    leg_left_y     .resize(number_of_points);
    leg_left_z     .resize(number_of_points);
    leg_left_roll  .resize(number_of_points);
    leg_left_pitch .resize(number_of_points);
    leg_left_yaw   .resize(number_of_points);
    leg_right_x    .resize(number_of_points);
    leg_right_y    .resize(number_of_points);
    leg_right_z    .resize(number_of_points);
    leg_right_roll .resize(number_of_points);
    leg_right_pitch.resize(number_of_points);
    leg_right_yaw  .resize(number_of_points);
    leg_left_angles .resize(number_of_points);
    leg_right_angles.resize(number_of_points);

    for(int i=0; i < number_of_points/2; i++)
    {
	leg_left_x     [i] = left_start_x + 0.01*( -sin (4*M_PI*t));
	leg_left_y     [i] = left_start_y;// + 0.005*((-cosh(4*M_PI*t - M_PI) + 1)/cosh(M_PI) + 1);//0.02*(-cos(6*M_PI*t) + 1);
	float delta_z = 0.01*( -sin (4*M_PI*t));
	if(delta_z < 0)
	    delta_z *= 0.9;
	leg_left_z     [i] = left_start_z + delta_z;
	leg_left_roll  [i] = 0;
	leg_left_pitch [i] = 0;
	leg_left_yaw   [i] = 0;
	leg_right_x    [i] = right_start_x;
	leg_right_y    [i] = right_start_y;// + 0.005*((-cosh(4*M_PI*t - M_PI) + 1)/cosh(M_PI) + 1);//0.02*(-cos(6*M_PI*t) + 1);
	leg_right_z    [i] = right_start_z;
	leg_right_roll [i] = 0;
	leg_right_pitch[i] = 0;
	leg_right_yaw  [i] = 0;
	t += 1.0/30.0;
    }
    t = 0;
    for(int i=number_of_points/2; i < number_of_points; i++)
    {
	leg_left_x     [i] = left_start_x;
	leg_left_y     [i] = left_start_y;// - 0.005*((-cosh(4*M_PI*t - M_PI) + 1)/cosh(M_PI) + 1);//0.02*(-cos(6*M_PI*t) + 1);
	leg_left_z     [i] = left_start_z;
	leg_left_roll  [i] = 0;
	leg_left_pitch [i] = 0;
	leg_left_yaw   [i] = 0;
	leg_right_x    [i] = right_start_x + 0.01*( -sin (4*M_PI*t));
	leg_right_y    [i] = right_start_y;// - 0.005*((-cosh(4*M_PI*t - M_PI) + 1)/cosh(M_PI) + 1);//0.02*(-cos(6*M_PI*t) + 1);
	float delta_z = 0.01*( -sin (4*M_PI*t));
	if(delta_z < 0)
	    delta_z *= 0.9;
	leg_right_z    [i] = right_start_z + delta_z;
	leg_right_roll [i] = 0;
	leg_right_pitch[i] = 0;
	leg_right_yaw  [i] = 0;
	t += 1.0/30.0;
    }
    for(int i=0; i < number_of_points; i++)
    {
	srvIK.request.x     = leg_left_x    [i];
	srvIK.request.y     = leg_left_y    [i];
	srvIK.request.z     = leg_left_z    [i];
	srvIK.request.roll  = leg_left_roll [i];
	srvIK.request.pitch = leg_left_pitch[i];
	srvIK.request.yaw   = leg_left_yaw  [i];
	if(cltCalculateIKLegLeft.call(srvIK))
	{
	    leg_left_angles[i] = srvIK.response.joint_values;
	    std::cout << "StepTest.->Left angles: ";
	    for(int j=0; j < 6; j++) std::cout << leg_left_angles[i][j] << "\t";
	    std::cout << std::endl;
	}
	else
	{
	    std::cout << "StepTest.->Cannot calculate IK iteration " << i << " values: " << leg_left_x[i] << "\t";
	    std::cout << leg_left_y[i] << "\t" << leg_left_z[i] << "\t" << leg_left_roll[i] << "\t" << leg_left_pitch[i] << "\t";
	    std::cout << leg_left_yaw[i] << std::endl;
	}
    }
    for(int i=0; i < number_of_points; i++)
    {
	srvIK.request.x     = leg_right_x    [i];
	srvIK.request.y     = leg_right_y    [i];
	srvIK.request.z     = leg_right_z    [i];
	srvIK.request.roll  = leg_right_roll [i];
	srvIK.request.pitch = leg_right_pitch[i];
	srvIK.request.yaw   = leg_right_yaw  [i];
	if(cltCalculateIKLegRight.call(srvIK))
	{
	    leg_right_angles[i] = srvIK.response.joint_values;
	    std::cout << "StepTest.->Right angles: ";
	    for(int j=0; j < 6; j++) std::cout << leg_right_angles[i][j] << "\t";
	    std::cout << std::endl;
	}
	else
	{
	    std::cout << "StepTest.->Cannot calculate IK iteration " << i << " values: " << leg_right_x[i] << "\t";
	    std::cout << leg_right_y[i]<<"\t"<< leg_right_z[i] << "\t" << leg_right_roll[i] << "\t" << leg_right_pitch[i] << "\t";
	    std::cout << leg_right_yaw[i] << std::endl;
	}
    }

    int state = 10;
    int idx = 0;

    while(ros::ok())
    {
	switch(state)
	{
	case 0:
	    if(idx < number_of_points)
	    {
		msgLegLeftGoalPose .data = leg_left_angles [idx];
		msgLegRightGoalPose.data = leg_right_angles[idx];
		pubLegLeftGoalPose .publish(msgLegLeftGoalPose );
		pubLegRightGoalPose.publish(msgLegRightGoalPose);
		idx++;
	    }
	    else
	    {
		idx = 0;
		state = 10;
	    }
	    break;
	case 10:
	    //getchar();
	    state = 0;
	    break;
	default:
	    break;
	}
        ros::spinOnce();
        loop.sleep();
    }
}
