#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "servo_utils.h"

/*---------------------------------------
|            UPPER BODY                  |
-----------------------------------------*/

#define ID_ARM_LEFT_SHOULDER_PITCH  2      //-> LEFT ARM
#define ID_ARM_LEFT_SHOULDER_ROLL   4
#define ID_ARM_LEFT_ELBOW_PITCH     6


#define ID_ARM_RIGHT_SHOULDER_PITCH 1      // --> RIGHT ARM
#define ID_ARM_RIGHT_SHOULDER_ROLL  3
#define ID_ARM_RIGHT_ELBOW_PITCH    5

#define ID_NECK_YAW                 19      // --> HEAD
#define ID_HEAD_PITCH               20

/*--------------------------------------
|             LOWER BODY                |
----------------------------------------*/

#define ID_LEG_LEFT_HIP_YAW         8       // --> LEFT LEG
#define ID_LEG_LEFT_HIP_ROLL        10
#define ID_LEG_LEFT_HIP_PITCH       12
#define ID_LEG_LEFT_KNEE_PITCH      14

#define ID_LEG_LEFT_ANKLE_PITCH     16      // --> LEFT FOOT
#define ID_LEG_LEFT_ANKLE_ROLL      18

#define ID_LEG_RIGHT_HIP_YAW        7       // --> RIGHT LEG
#define ID_LEG_RIGHT_HIP_ROLL       9
#define ID_LEG_RIGHT_HIP_PITCH      11
#define ID_LEG_RIGHT_KNEE_PITCH     13

#define ID_LEG_RIGHT_ANKLE_PITCH    15      // --> RIGHT FOOT
#define ID_LEG_RIGHT_ANKLE_ROLL     17

/*------------------------------------------*\
|               ZERO STATE                   |
| Zero state is considered to be the natural |
|  state of the humanoid when it stands up   |
|  staight.                                  |
\-------------------------------------------*/

#define ZERO_LEG_LEFT_HIP_YAW           2070
#define ZERO_LEG_LEFT_HIP_ROLL          2070
#define ZERO_LEG_LEFT_HIP_PITCH         2048
#define ZERO_LEG_LEFT_KNEE_PITCH        2400
#define ZERO_LEG_LEFT_ANKLE_PITCH       2150
#define ZERO_LEG_LEFT_ANKLE_ROLL        2048

#define ZERO_LEG_RIGHT_HIP_YAW          2019
#define ZERO_LEG_RIGHT_HIP_ROLL         1067
#define ZERO_LEG_RIGHT_HIP_PITCH        3084
#define ZERO_LEG_RIGHT_KNEE_PITCH       2890
#define ZERO_LEG_RIGHT_ANKLE_PITCH      2048
#define ZERO_LEG_RIGHT_ANKLE_ROLL       2048

#define ZERO_ARM_LEFT_SHOULDER_PITCH    2048
#define ZERO_ARM_LEFT_SHOULDER_ROLL     2440
#define ZERO_ARM_LEFT_ELBOW_PITCH       2048

#define ZERO_ARM_RIGHT_SHOULDER_PITCH   1905
#define ZERO_ARM_RIGHT_SHOULDER_ROLL     880 
#define ZERO_ARM_RIGHT_ELBOW_PITCH      2505

#define ZERO_NECK_YAW                   2048
#define ZERO_HEAD_PITCH                 3072

/*------------------------------------------\
|       CLOCKWISE OR COUNTER-CLOCKWISE      |
|  Indicates if a servo is positive or negative
|    for clockwise rotation                 |
\------------------------------------------*/

#define CW_LEG_LEFT_HIP_YAW               -1
#define CW_LEG_LEFT_HIP_ROLL               1
#define CW_LEG_LEFT_HIP_PITCH              1
#define CW_LEG_LEFT_KNEE_PITCH             1
#define CW_LEG_LEFT_ANKLE_PITCH           -1
#define CW_LEG_LEFT_ANKLE_ROLL            -1
                                 
#define CW_LEG_RIGHT_HIP_YAW              -1
#define CW_LEG_RIGHT_HIP_ROLL              1
#define CW_LEG_RIGHT_HIP_PITCH            -1
#define CW_LEG_RIGHT_KNEE_PITCH           -1
#define CW_LEG_RIGHT_ANKLE_PITCH           1
#define CW_LEG_RIGHT_ANKLE_ROLL           -1
                                 
#define CW_ARM_LEFT_SHOULDER_PITCH         1
#define CW_ARM_LEFT_SHOULDER_ROLL         -1
#define CW_ARM_LEFT_ELBOW_PITCH           -1
                                 
#define CW_ARM_RIGHT_SHOULDER_PITCH       -1
#define CW_ARM_RIGHT_SHOULDER_ROLL        -1
#define CW_ARM_RIGHT_ELBOW_PITCH           1
                      
#define CW_NECK_YAW                        1
#define CW_HEAD_PITCH                     -1

uint8_t servos_ids[20] =   // --> SETTING UP THE ARRAY OF ID'S
{
    ID_LEG_LEFT_HIP_YAW    ,
    ID_LEG_LEFT_HIP_ROLL   , 
    ID_LEG_LEFT_HIP_PITCH  , 
    ID_LEG_LEFT_KNEE_PITCH ,
    ID_LEG_LEFT_ANKLE_PITCH,
    ID_LEG_LEFT_ANKLE_ROLL ,
    ID_LEG_RIGHT_HIP_YAW    ,
    ID_LEG_RIGHT_HIP_ROLL   , 
    ID_LEG_RIGHT_HIP_PITCH  , 
    ID_LEG_RIGHT_KNEE_PITCH ,
    ID_LEG_RIGHT_ANKLE_PITCH,
    ID_LEG_RIGHT_ANKLE_ROLL ,
    ID_ARM_LEFT_SHOULDER_PITCH ,
    ID_ARM_LEFT_SHOULDER_ROLL  ,
    ID_ARM_LEFT_ELBOW_PITCH    ,
    ID_ARM_RIGHT_SHOULDER_PITCH ,
    ID_ARM_RIGHT_SHOULDER_ROLL  ,
    ID_ARM_RIGHT_ELBOW_PITCH    ,
    ID_NECK_YAW  ,
    ID_HEAD_PITCH,
};

uint16_t servos_position_zero[20] = // --> Setting up the array of state_zero_positions
{
    ZERO_LEG_LEFT_HIP_YAW    ,
    ZERO_LEG_LEFT_HIP_ROLL   , 
    ZERO_LEG_LEFT_HIP_PITCH  , 
    ZERO_LEG_LEFT_KNEE_PITCH ,
    ZERO_LEG_LEFT_ANKLE_PITCH,
    ZERO_LEG_LEFT_ANKLE_ROLL ,
    ZERO_LEG_RIGHT_HIP_YAW    ,
    ZERO_LEG_RIGHT_HIP_ROLL   , 
    ZERO_LEG_RIGHT_HIP_PITCH  , 
    ZERO_LEG_RIGHT_KNEE_PITCH ,
    ZERO_LEG_RIGHT_ANKLE_PITCH,
    ZERO_LEG_RIGHT_ANKLE_ROLL ,
    ZERO_ARM_LEFT_SHOULDER_PITCH ,
    ZERO_ARM_LEFT_SHOULDER_ROLL  ,
    ZERO_ARM_LEFT_ELBOW_PITCH    ,
    ZERO_ARM_RIGHT_SHOULDER_PITCH ,
    ZERO_ARM_RIGHT_SHOULDER_ROLL  ,
    ZERO_ARM_RIGHT_ELBOW_PITCH    ,
    ZERO_NECK_YAW  ,
    ZERO_HEAD_PITCH,
};

int servos_is_clockwise[20] = // --> Setting up the array of cw info for each servo
{
    CW_LEG_LEFT_HIP_YAW    ,
    CW_LEG_LEFT_HIP_ROLL   , 
    CW_LEG_LEFT_HIP_PITCH  , 
    CW_LEG_LEFT_KNEE_PITCH ,
    CW_LEG_LEFT_ANKLE_PITCH,
    CW_LEG_LEFT_ANKLE_ROLL ,
    CW_LEG_RIGHT_HIP_YAW    ,
    CW_LEG_RIGHT_HIP_ROLL   , 
    CW_LEG_RIGHT_HIP_PITCH  , 
    CW_LEG_RIGHT_KNEE_PITCH ,
    CW_LEG_RIGHT_ANKLE_PITCH,
    CW_LEG_RIGHT_ANKLE_ROLL ,
    CW_ARM_LEFT_SHOULDER_PITCH ,
    CW_ARM_LEFT_SHOULDER_ROLL  ,
    CW_ARM_LEFT_ELBOW_PITCH    ,
    CW_ARM_RIGHT_SHOULDER_PITCH ,
    CW_ARM_RIGHT_SHOULDER_ROLL  ,
    CW_ARM_RIGHT_ELBOW_PITCH    ,
    CW_NECK_YAW  ,
    CW_HEAD_PITCH,
};

uint16_t servos_goal_position[20];  // PENDING TO BE READ

/*------------------------------------------*\
|               CALLBACKS                    |
|-------------------------------------------*/

void callback_legs_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 12)
    {
       std::cout << "CM730.->Error!!: goal position for both legs must be a 12-value array." << std::endl;
       return;
    }
    for(int i=0, j=0; i < 12; i++, j++)
        servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] +
            servos_position_zero[j]);
}

void callback_leg_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 6)
    {
       std::cout << "CM730.->Error!!: goal position for left leg must be a 6-value array." << std::endl;
       return;
    }
    for(int i=0, j=0; i < 6; i++, j++)
        servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] + 
            servos_position_zero[j]);
}

void callback_leg_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 6)
    {
        std::cout << "CM730.->Error!!: goal position for right leg must be a 6-value array." << std::endl;
        return;
    }
    for(int i=0, j=6; i < 6; i++, j++)
        servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] +
            servos_position_zero[j]);
}

void callback_arms_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 6)
    {
        std::cout << "CM730.->Error!!: goal position for both arms must be a 6-value array." << std::endl;
        return;
    }
    for(int i=0, j=12; i < 6; i++, j++)
        servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] +
            servos_position_zero[j]);
}

void callback_arm_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 3)
    {
        std::cout << "CM730.->Error!!: goal position for left arm must be a 3-value array." << std::endl;
        return;
    }
    for(int i=0, j=12; i < 3; i++, j++)
        servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] +
            servos_position_zero[j]);
}

void callback_arm_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 3)
    {
        std::cout << "CM730.->Error!!: goal position for right leg must be a 3-value array." << std::endl;
        return;
    }
    for(int i=0, j=15; i < 3; i++, j++)
        servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] +
            servos_position_zero[j]);
}

void callback_head_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 2)
    {
        std::cout << "CM730.->Error!! goal position for head must be a 2-value array" << std::endl;
        return;
    }
    for(int i=0, j=18; i < 2; i++, j++)
        servos_goal_position[j] = uint16_t(msg->data[i] * SERVO_MX_BITS_PER_RAD * servos_is_clockwise[j] +
            servos_position_zero[j]);
}

int main(int argc, char** argv)
{   
    // --> INITIALIZING ROS  
    std::cout << "INITIALIZING CM730 NODE BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "cm730v2");
    ros::NodeHandle n;
    ros::Rate loop(30);
    // --> OBTAINING USB PORT NAME
    system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");

    /*----------------------------------------------*\
    |               ROS SUBSCRIBERS                  |
    |-----------------------------------------------*/

    ros::Subscriber sub_legs_goal_pose      = n.subscribe("legs_goal_pose",      1, callback_legs_goal_pose);
    ros::Subscriber sub_leg_left_goal_pose  = n.subscribe("leg_left_goal_pose",  1, callback_leg_left_goal_pose);
    ros::Subscriber sub_leg_right_goal_pose = n.subscribe("leg_right_goal_pose", 1, callback_leg_right_goal_pose);
    ros::Subscriber sub_arms_goal_pose      = n.subscribe("arms_goal_pose",      1, callback_arms_goal_pose);
    ros::Subscriber sub_arm_left_goal_pose  = n.subscribe("arm_left_goal_pose",  1, callback_arm_left_goal_pose);
    ros::Subscriber sub_arm_right_goal_pose = n.subscribe("arm_right_goal_pose", 1, callback_arm_right_goal_pose);
    ros::Subscriber sub_head_goal_pose      = n.subscribe("head_goal_pose",      1, callback_head_goal_pose);     

    /*----------------------------------------------*\
    |               ROS PUBLISHERS                   |
    |-----------------------------------------------*/

    ros::Publisher pub_legs_current_pose      = n.advertise<std_msgs::Float32MultiArray>("legs_current_pose",       1);
    ros::Publisher pub_leg_left_current_pose  = n.advertise<std_msgs::Float32MultiArray>("leg_left_current_pose",   1);
    ros::Publisher pub_leg_right_current_pose = n.advertise<std_msgs::Float32MultiArray>("leg_right_current_pose",  1);
    ros::Publisher pub_arms_current_pose      = n.advertise<std_msgs::Float32MultiArray>("arms_current_pose",       1);
    ros::Publisher pub_arm_left_current_pose  = n.advertise<std_msgs::Float32MultiArray>("arm_left_current_pose",   1);
    ros::Publisher pub_arm_right_current_pose = n.advertise<std_msgs::Float32MultiArray>("arm_right_current_pose",  1);
    ros::Publisher pub_head_current_pose      = n.advertise<std_msgs::Float32MultiArray>("head_current_pose",       1);
    ros::Publisher pub_joint_current_angles   = n.advertise<std_msgs::Float32MultiArray>("joint_current_angles",    1);
    ros::Publisher pub_joint_states           = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

    sensor_msgs::JointState msg_joint_states;
    std_msgs::Float32MultiArray msg_joint_current_angles;

    msg_joint_current_angles.data.resize(20);
    msg_joint_states.name.resize(20);
    msg_joint_states.position.resize(20);
  
    msg_joint_states.name[0]  = "left_hip_yaw";   
    msg_joint_states.name[1]  = "left_hip_roll";  
    msg_joint_states.name[2]  = "left_hip_pitch"; 
    msg_joint_states.name[3]  = "left_knee_pitch";
    msg_joint_states.name[4]  = "left_ankle_pitch";
    msg_joint_states.name[5]  = "left_ankle_roll";
  
    msg_joint_states.name[6]  = "right_hip_yaw";
    msg_joint_states.name[7]  = "right_hip_roll";
    msg_joint_states.name[8]  = "right_hip_pitch";
    msg_joint_states.name[9]  = "right_knee_pitch";
    msg_joint_states.name[10] = "right_ankle_pitch";
    msg_joint_states.name[11] = "right_ankle_roll";

    msg_joint_states.name[12] = "left_shoulder_pitch";  
    msg_joint_states.name[13] = "left_shoulder_roll";   
    msg_joint_states.name[14] = "left_elbow_pitch";    
                                              
    msg_joint_states.name[15] = "right_shoulder_pitch"; 
    msg_joint_states.name[16] = "right_shoulder_roll";  
    msg_joint_states.name[17] = "right_elbow_pitch";   

    msg_joint_states.name[18] = "neck_yaw";   
    msg_joint_states.name[19] = "head_pitch";

    Servo::CommHandler comm("/dev/ttyUSB0");

    {int counter{0};
    while( !comm.startComm() )
    {
        counter++;
        ros::Duration(1.0).sleep();
        if(counter > 10)
        {
            ROS_ERROR("Shutting down after attempting to open port. Shutting down");
            return -1;
        };
    }}

    /*---------------------------------------
    |      WAKE UP CM730 (ADDR 24)          |
    -----------------------------------------*/
    comm.wakeupAllServos();

    while(ros::ok())
    {
        uint16_t dxl_current_pos;
        // Read all positions
        for(int id=0; id<20; ++id)
        {
            dxl_current_pos = comm.getPosition(id);
            msg_joint_states.position[id] = (int(dxl_current_pos) - int(servos_position_zero[id]))
                                            * SERVO_MX_RADS_PER_BIT 
                                            * servos_is_clockwise[id];
            msg_joint_current_angles.data[id] = msg_joint_states.position[id];
        }

        // Write all positions
        for(int id=0; id<20; ++id)
        {
            bool ret = comm.setPosition(id, dxl_current_pos);
        }

        msg_joint_states.header.stamp = ros::Time::now();
        pub_joint_states.publish(msg_joint_states);
        pub_joint_current_angles.publish(msg_joint_current_angles);
        ros::spinOnce();
        loop.sleep();
    }

    //After killing the node, shutdown all servos
    comm.shutdownAllServos();

    return 0;
    
}
