#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "ctrl_msgs/CalculateIK.h"
#include "ctrl_msgs/CalculateDK.h"
#define Lao (0.025/2)
#define Hao (0.025/2)
#define Ds (0.025/2)
#define Lan 0 //0.0381
#define Laf 0.105
#define Lab 0.105
#define qb 0 //(0.38 -0.411) // -0.411 es offset
#define qf 0 //-(0.38 -0.411) // -0.411 es offset
#define qgs 0
#define qge 0
#define hgs 0
#define hge 0
#define xsd (0.49*Ds)
#define xed (0.49*Ds)
#define y_left_CoM  0 //0.08
#define y_right_CoM  0 //(-y_left_CoM)

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING STEP FOOT NODE..." << std::endl;
    ros::init(argc, argv, "step_test");
    ros::NodeHandle n;
    ros::Rate loop(40);

    ros::Publisher pubLegLeftGoalPose  = n.advertise<std_msgs::Float32MultiArray>("/hardware/leg_left_goal_pose", 1); 
    ros::Publisher pubLegRightGoalPose = n.advertise<std_msgs::Float32MultiArray>("/hardware/leg_right_goal_pose", 1);
    ros::Publisher pubArmLeftGoalPose  = n.advertise<std_msgs::Float32MultiArray>("/hardware/arm_left_goal_pose", 1); 
    ros::Publisher pubArmRightGoalPose = n.advertise<std_msgs::Float32MultiArray>("/hardware/arm_right_goal_pose", 1);
    ros::Publisher pubHeadGoalPose     = n.advertise<std_msgs::Float32MultiArray>("/hardware/head_goal_pose", 1);
    ros::ServiceClient cltCalculateIKLegLeft  = n.serviceClient<ctrl_msgs::CalculateIK>("/control/ik_leg_left"); 
    ros::ServiceClient cltCalculateIKLegRight = n.serviceClient<ctrl_msgs::CalculateIK>("/control/ik_leg_right");
    ros::ServiceClient cltCalculateDKLegLeft  = n.serviceClient<ctrl_msgs::CalculateDK>("/control/dk_leg_left"); 
    ros::ServiceClient cltCalculateDKLegRight = n.serviceClient<ctrl_msgs::CalculateDK>("/control/dk_leg_right");
    std_msgs::Float32MultiArray msgLegLeftGoalPose;
    std_msgs::Float32MultiArray msgLegRightGoalPose;
    std_msgs::Float32MultiArray msgArmLeftGoalPose;
    std_msgs::Float32MultiArray msgArmRightGoalPose;
    std_msgs::Float32MultiArray msgHeadGoalPose;
    ctrl_msgs::CalculateIK srvIK;

    //Trajectory generation

    //Offsets
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
    const int number_of_points = 64;
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

    //Swing left leg

    //t0

    for(int i=0; i < 1; i++)
    {
    leg_left_x     [i] = left_start_x; //+ 0.004;
    leg_left_y     [i] = left_start_y;
    leg_left_z     [i] = hgs + Lan + left_start_z;
    leg_left_roll  [i] = 0;
    leg_left_pitch [i] = qgs;
    leg_left_yaw   [i] = 0;
    
    leg_right_x    [i] = -(xed) + right_start_x;
    leg_right_y    [i] = right_start_y;
    leg_right_z    [i] = right_start_z;
    leg_right_roll [i] = 0;
    leg_right_pitch[i] = 0;
    leg_right_yaw  [i] = 0;
    t += 1.0/30.0;
    }
    t = 0;

    //Td

    for(int i=1; i < 11; i++)
    {
    leg_left_x     [i] = Lan*sin(qb) + Laf*(1 - cos(qb)) + left_start_x; //+ 0.004;
    leg_left_y     [i] = left_start_y;
    leg_left_z     [i] = hgs + Laf*sin(qb) + Lan*cos(qb) + left_start_z;
    leg_left_roll  [i] = 0;
    leg_left_pitch [i] = qb;
    leg_left_yaw   [i] = 0;
    
    leg_right_x    [i] = -(Ds - xsd) + right_start_x;
    leg_right_y    [i] = y_right_CoM + right_start_y;
    leg_right_z    [i] = right_start_z;
    leg_right_roll [i] = 0;
    leg_right_pitch[i] = 0;
    leg_right_yaw  [i] = 0;
    }
    t = 0;

    // Tm

    for(int i=11; i < 21; i++)
    {
    leg_left_x     [i] = Lao + left_start_x; //+ 0.004;
    leg_left_y     [i] = left_start_y;
    leg_left_z     [i] = Hao + left_start_z;
    leg_left_roll  [i] = 0;
    leg_left_pitch [i] = qb;
    leg_left_yaw   [i] = 0;
    
    leg_right_x    [i] = -(Ds - xsd) + right_start_x;
    leg_right_y    [i] = y_right_CoM + right_start_y;
    leg_right_z    [i] = right_start_z;
    leg_right_roll [i] = 0;
    leg_right_pitch[i] = 0;
    leg_right_yaw  [i] = 0;
    t += 1.0/30.0;
    }
    t = 0;

    //Tc

    for(int i=21; i < 31; i++)
    {
    leg_left_x     [i] = 2*Ds - Lan + sin(qf) - Lab*(1 - cos(qf)) + left_start_x; //+ 0.004;
    leg_left_y     [i] = left_start_y;
    leg_left_z     [i] = hge + Lab*sin(qf) + Lan*cos(qf) + left_start_z;
    leg_left_roll  [i] = 0;
    leg_left_pitch [i] = qf;
    leg_left_yaw   [i] = 0;
    
    leg_right_x    [i] = -(Ds + xed) + right_start_x;
    leg_right_y    [i] = y_right_CoM + right_start_y;
    leg_right_z    [i] = right_start_z;
    leg_right_roll [i] = 0;
    leg_right_pitch[i] = 0;
    leg_right_yaw  [i] = 0;
    t += 1.0/30.0;
    }
    t = 0;

    //Tc+Td

    for(int i=31; i < 32; i++)
    {
    leg_left_x     [i] = 2*Ds + left_start_x; //+ 0.004;
    leg_left_y     [i] = left_start_y;
    leg_left_z     [i] = hge + Lan + left_start_z;
    leg_left_roll  [i] = 0;
    leg_left_pitch [i] = qge;
    leg_left_yaw   [i] = 0;
    
    leg_right_x    [i] = -(Ds + xed) + right_start_x;
    leg_right_y    [i] = y_right_CoM + right_start_y;
    leg_right_z    [i] = right_start_z;
    leg_right_roll [i] = 0;
    leg_right_pitch[i] = 0;
    leg_right_yaw  [i] = 0;
    t += 1.0/30.0;
    }

    //Swing right leg

    //t0

    for(int i=32; i < 33; i++)
    {
    leg_right_x     [i] = right_start_x; //+ 0.004;
    leg_right_y     [i] = right_start_y;
    leg_right_z     [i] = hgs + Lan + right_start_z;
    leg_right_roll  [i] = 0;
    leg_right_pitch [i] = qgs;
    leg_right_yaw   [i] = 0;
    
    leg_left_x    [i] = -(xed) + left_start_x;
    leg_left_y    [i] = left_start_y;
    leg_left_z    [i] = left_start_z;
    leg_left_roll [i] = 0;
    leg_left_pitch[i] = 0;
    leg_left_yaw  [i] = 0;
    t += 1.0/30.0;
    }
    t = 0;

    //Td

    for(int i=33; i < 43; i++)
    {
    leg_right_x     [i] = Lan*sin(qb) + Laf*(1 - cos(qb)) + right_start_x; //+ 0.004;
    leg_right_y     [i] = right_start_y;
    leg_right_z     [i] = hgs + Laf*sin(qb) + Lan*cos(qb) + right_start_z;
    leg_right_roll  [i] = 0;
    leg_right_pitch [i] = qb;
    leg_right_yaw   [i] = 0;
    
    leg_left_x    [i] = -(Ds - xsd) + left_start_x;
    leg_left_y    [i] = y_left_CoM + left_start_y;
    leg_left_z    [i] = left_start_z;
    leg_left_roll [i] = 0;
    leg_left_pitch[i] = 0;
    leg_left_yaw  [i] = 0;
    }
    t = 0;

    // Tm

    for(int i=43; i < 53; i++)
    {
    leg_right_x     [i] = Lao + right_start_x; //+ 0.004;
    leg_right_y     [i] = right_start_y;
    leg_right_z     [i] = Hao + right_start_z;
    leg_right_roll  [i] = 0;
    leg_right_pitch [i] = qb;
    leg_right_yaw   [i] = 0;
    
    leg_left_x    [i] = -(Ds - xsd) + left_start_x;
    leg_left_y    [i] = y_left_CoM + left_start_y;
    leg_left_z    [i] = left_start_z;
    leg_left_roll [i] = 0;
    leg_left_pitch[i] = 0;
    leg_left_yaw  [i] = 0;
    t += 1.0/30.0;
    }
    t = 0;

    //Tc

    for(int i=53; i < 63; i++)
    {
    leg_right_x     [i] = 2*Ds - Lan + sin(qf) - Lab*(1 - cos(qf)) + right_start_x; //+ 0.004;
    leg_right_y     [i] = right_start_y;
    leg_right_z     [i] = hge + Lab*sin(qf) + Lan*cos(qf) + right_start_z;
    leg_right_roll  [i] = 0;
    leg_right_pitch [i] = qf;
    leg_right_yaw   [i] = 0;
    
    leg_left_x    [i] = -(Ds + xed) + left_start_x;
    leg_left_y    [i] = y_left_CoM + left_start_y;
    leg_left_z    [i] = left_start_z;
    leg_left_roll [i] = 0;
    leg_left_pitch[i] = 0;
    leg_left_yaw  [i] = 0;
    t += 1.0/30.0;
    }
    t = 0;

    //Tc+Td

    for(int i=63; i < 64; i++)
    {
    leg_right_x     [i] = 2*Ds + right_start_x; //+ 0.004;
    leg_right_y     [i] = right_start_y;
    leg_right_z     [i] = hge + Lan + right_start_z;
    leg_right_roll  [i] = 0;
    leg_right_pitch [i] = qge;
    leg_right_yaw   [i] = 0;
    
    leg_left_x    [i] = -(Ds + xed) + left_start_x;
    leg_left_y    [i] = y_left_CoM + left_start_y;
    leg_left_z    [i] = left_start_z;
    leg_left_roll [i] = 0;
    leg_left_pitch[i] = 0;
    leg_left_yaw  [i] = 0;
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
