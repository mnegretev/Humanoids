#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16MultiArray.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/String.h"
#include <sstream>

/*---------------------------------------
|      DEFINE SERVO ID'S AND NAMING     |
-----------------------------------------*/

//-> ROS THINKS CM730 IS A SERVO
#define ID_CM730    200

/*---------------------------------------
            UPPER BODY
-----------------------------------------
*///-> LEFT SHOULDER
#define ID_ARM_LEFT_SHOULDER_PITCH  2
#define ID_ARM_LEFT_SHOULDER_ROLL   4
// --> LEFT ELBOW
#define ID_ARM_LEFT_ELBOW_PITCH     6

// --> RIGHT SHOULDER
#define ID_ARM_RIGHT_SHOULDER_PITCH 1
#define ID_ARM_RIGHT_SHOULDER_ROLL  3
// --> RIGHT ELBOW
#define ID_ARM_RIGHT_ELBOW_PITCH    5

// --> HEAD
#define ID_NECK_YAW                 19
#define ID_HEAD_PITCH               20

/*--------------------------------------
             LOWER BODY
----------------------------------------
*/// --> LEFT HIP
#define ID_LEG_LEFT_HIP_YAW         8
#define ID_LEG_LEFT_HIP_ROLL        10
#define ID_LEG_LEFT_HIP_PITCH       12
// --> LEFT KNEE
#define ID_LEG_LEFT_KNEE_PITCH      14
// --> LEFT ANKLE
#define ID_LEG_LEFT_ANKLE_PITCH     16
#define ID_LEG_LEFT_ANKLE_ROLL      18

// --> RIGHT HIP
#define ID_LEG_RIGHT_HIP_YAW        7
#define ID_LEG_RIGHT_HIP_ROLL       9
#define ID_LEG_RIGHT_HIP_PITCH      11
// --> RIHT KNEE
#define ID_LEG_RIGHT_KNEE_PITCH     13
// --> RIGHT ANKLE
#define ID_LEG_RIGHT_ANKLE_PITCH    15
#define ID_LEG_RIGHT_ANKLE_ROLL     17

/*------------------------------------------*\
|               ZERO STATE                   |    |
| Zero state is considered to be the natural |
|  state of the humanoid when it stands up   |
|  staight.                                  |
\-------------------------------------------*/

#define ZERO_LEG_LEFT_HIP_YAW       2061 // Servo 8
#define ZERO_LEG_LEFT_HIP_ROLL      2100 
#define ZERO_LEG_LEFT_HIP_PITCH     1903
#define ZERO_LEG_LEFT_KNEE_PITCH    2439
#define ZERO_LEG_LEFT_ANKLE_PITCH   2119
#define ZERO_LEG_LEFT_ANKLE_ROLL    2075

#define ZERO_LEG_RIGHT_HIP_YAW      2049
#define ZERO_LEG_RIGHT_HIP_ROLL     1060
#define ZERO_LEG_RIGHT_HIP_PITCH    3219
#define ZERO_LEG_RIGHT_KNEE_PITCH   2857
#define ZERO_LEG_RIGHT_ANKLE_PITCH  2019
#define ZERO_LEG_RIGHT_ANKLE_ROLL   2021

#define ZERO_ARM_LEFT_SHOULDER_PITCH    2041
#define ZERO_ARM_LEFT_SHOULDER_ROLL     2462
#define ZERO_ARM_LEFT_ELBOW_PITCH       1617

#define ZERO_ARM_RIGHT_SHOULDER_PITCH   1943
#define ZERO_ARM_RIGHT_SHOULDER_ROLL    939
#define ZERO_ARM_RIGHT_ELBOW_PITCH      2920

#define ZERO_NECK_YAW                   2048
#define ZERO_HEAD_PITCH                 2048

/*------------------------------------------\
|       CLOCKWISE OR COUNTER-CLOCKWISE
|  Servos position goes from 0 to 4095 in a 
|  counter clock-wise direction, but some
|  servos
\-------------------------------------------*/

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

/*  Servos have 4096 different positions, since there are
    2*PI radians in a circumference, eache radian has
    4096/2*PI = 651.898646904 BITS PER RADIAN*/
#define SERVO_MX_BITS_PER_RAD       651.898646904

/*  Inverse of SERVO_MX_BITS_PER_RAD*/
#define SERVO_MX_RADS_PER_BIT       0.001533981

/*  DYNAMIXEL PROTOCOL VERSION: 1.0
    DYNMIXEL PROTOCOL 2.0 IS NOT CURRENTLY SUPPORTED */
#define SERVO_PROTOCOL_VERSION      1.0

/*
    DYNAMIXEL SERVOS HAVE UP TO 70 DIFFERENT CONTROL OPTIONS
    EITHER TO READ DATA OR WRITE DATA. THE NUMBERS SHOWN BELOW
    ARE THE ACTUAL ADDRESSES THAT WE WILL BE USING. FOR THIS NODE
    WE WILL NEED TO TURN ON THE MOTOR (24), REVIEW THE CURRENT
    POSITION OF THE MOTOR (36) AND SEND A DESIRED GOAL POSITION (30)

    TO SEE THE COMPLETE LIST OF CONTROL OPTIONS VISIT:
    https://emanual.robotis.com/docs/en/dxl/mx/mx-64/#control-table-of-eeprom-area*/
#define ADDR_MX_CURRENT_POSITION    36
#define ADDR_LEN_CURRENT_POSITION   2

#define ADDR_MX_GOAL_POSITION       30
#define ADDR_LEN_GOAL_POSITION      2

#define ADDR_CM730_DYNAMIXEL_POWER  24

// --> SETTING UP THE ARRAY OF ID'S
uint8_t servos_ids[20] =
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

//->Setting up the array of state_zero_positions
uint16_t servos_position_zero[20] =
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

// -->Setting up the array of cw info for each servo
int servos_is_clockwise[20] =
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

uint16_t servos_goal_position[20];

/*------------------------------------------*\
|                CALLBACKS
|
|
|
\-------------------------------------------*/

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

    // --> OBTAINING USB PORT NAME
    std::string usbport="ttyUSB0";
    n.getParam("usbport", usbport);
    
    std::string lat1, lat2, latencia;
    lat1 = "echo 1 | sudo tee /sys/bus/usb-serial/devices/";
    lat2="/latency_timer";
    latencia= lat1+usbport+lat2;
    char const *late = latencia.data();
    ros::Rate loop(30);
    system(late);
    //system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");


    ros::Subscriber sub_legs_goal_pose      = n.subscribe("legs_goal_pose",      1, callback_legs_goal_pose);
    ros::Subscriber sub_leg_left_goal_pose  = n.subscribe("leg_left_goal_pose",  1, callback_leg_left_goal_pose);
    ros::Subscriber sub_leg_right_goal_pose = n.subscribe("leg_right_goal_pose", 1, callback_leg_right_goal_pose);
    ros::Subscriber sub_arms_goal_pose      = n.subscribe("arms_goal_pose",      1, callback_arms_goal_pose);
    ros::Subscriber sub_arm_left_goal_pose  = n.subscribe("arm_left_goal_pose",  1, callback_arm_left_goal_pose);
    ros::Subscriber sub_arm_right_goal_pose = n.subscribe("arm_right_goal_pose", 1, callback_arm_right_goal_pose);
    ros::Subscriber sub_head_goal_pose      = n.subscribe("head_goal_pose",      1, callback_head_goal_pose);     

    ros::Publisher pub_legs_current_pose      = n.advertise<std_msgs::Float32MultiArray>("legs_current_pose",       1);
    ros::Publisher pub_leg_left_current_pose  = n.advertise<std_msgs::Float32MultiArray>("leg_left_current_pose",   1);
    ros::Publisher pub_leg_right_current_pose = n.advertise<std_msgs::Float32MultiArray>("leg_right_current_pose",  1);
    ros::Publisher pub_arms_current_pose      = n.advertise<std_msgs::Float32MultiArray>("arms_current_pose",       1);
    ros::Publisher pub_arm_left_current_pose  = n.advertise<std_msgs::Float32MultiArray>("arm_left_current_pose",   1);
    ros::Publisher pub_arm_right_current_pose = n.advertise<std_msgs::Float32MultiArray>("arm_right_current_pose",  1);
    ros::Publisher pub_head_current_pose      = n.advertise<std_msgs::Float32MultiArray>("head_current_pose",       1);
    ros::Publisher pub_joint_current_angles   = n.advertise<std_msgs::Float32MultiArray>("joint_current_angles",    1);
    
    ros::Publisher  pub_joint_states = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

    sensor_msgs::JointState msg_joint_states;
    std_msgs::Float32MultiArray msg_joint_current_angles;
    msg_joint_current_angles.data.resize(20);
  
    msg_joint_states.name.resize(20);
    msg_joint_states.position.resize(20);
  
    msg_joint_states.name[0]  ="left_hip_yaw";   
    msg_joint_states.name[1]  ="left_hip_roll";  
    msg_joint_states.name[2]  ="left_hip_pitch"; 
    msg_joint_states.name[3]  ="left_knee_pitch";
    msg_joint_states.name[4]  ="left_ankle_pitch";
    msg_joint_states.name[5]  ="left_ankle_roll";
  
    msg_joint_states.name[6]  ="right_hip_yaw";
    msg_joint_states.name[7]  ="right_hip_roll";
    msg_joint_states.name[8]  ="right_hip_pitch";
    msg_joint_states.name[9]  ="right_knee_pitch";
    msg_joint_states.name[10] ="right_ankle_pitch";
    msg_joint_states.name[11] ="right_ankle_roll";

    msg_joint_states.name[12] ="left_shoulder_pitch";  
    msg_joint_states.name[13] ="left_shoulder_roll";   
    msg_joint_states.name[14] ="left_elbow_pitch";    
                                              
    msg_joint_states.name[15] ="right_shoulder_pitch"; 
    msg_joint_states.name[16] ="right_shoulder_roll";  
    msg_joint_states.name[17] ="right_elbow_pitch";   

    msg_joint_states.name[18] ="neck_yaw";   
    msg_joint_states.name[19] ="head_pitch";
    
    std::string dev="/dev/";
    std::string puerto = dev+usbport;
    char const *puertochar = puerto.data();
    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(puertochar);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(SERVO_PROTOCOL_VERSION);

    if(portHandler->openPort())
        std::cout << "CM730.->Serial port successfully openned on port " << std::endl;
    else
    {
        std::cout << "CM730.->Cannot open serial port" << std::endl;
        return -1;
    }
    
    if(portHandler->setBaudRate(1000000))
        std::cout << "CM730.->Baudrate successfully set to " << std::endl;
    else
    {
        std::cout << "CM730.->Cannot set baud rate" << std::endl;
        return -1;
    }

    /*Sync read is not supported for protocol 1.0 communication, only for protocol 2.0 communication
      so that is the reason why we are using bulk read communication*/
    
    // Initialize GroupBulkRead instance
    dynamixel::GroupBulkRead groupRead(portHandler, packetHandler);
    
    // Initialize GroupSyncWrite instance
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, ADDR_LEN_GOAL_POSITION);

    // Variable to verify if instruction was added succesfully to groupRead
    bool dxl_addparam_result = false;
    // Variable to verify data result
    bool dxl_getdata_result = false;

    uint8_t  dxl_error       = 0;
    int      dxl_comm_result = COMM_TX_FAIL;
    uint16_t dxl_current_pos;
    uint8_t param_goal_position[2];

    /*---------------------------------------
    |      WAKE UP CM730 (ADDR 24)          |
    -----------------------------------------*/
    std::cout << "Wake up CM730 " << std::endl;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID_CM730, ADDR_CM730_DYNAMIXEL_POWER, 1, &dxl_error);
    
    if(dxl_comm_result != COMM_SUCCESS)
        std::cout << "CM730.->Commnunication problem while turning on dynamixel power." << std::endl;
    if(dxl_error != 0)
        std::cout << "CM730.->Status error after turning on dynamixel power: " << int(dxl_error) << std::endl;
    ros::Duration(1.0).sleep();
   
    /*---------------------------------------
    |      READ ALL POSITIONS (ADDR 36)      |
    -----------------------------------------*/

    for(int i=0; i<20; ++i)
    {
        dxl_addparam_result = groupRead.addParam(servos_ids[i], 
                            ADDR_MX_CURRENT_POSITION, 
                            ADDR_LEN_CURRENT_POSITION);
        if(dxl_addparam_result != true){
            std::cout << "CM730 -> Failed to add id " << servos_ids[i] << " to the list of groupBulkRead" << std::endl;
        }
    } 
    dxl_comm_result = groupRead.txRxPacket();
    if(dxl_comm_result != COMM_SUCCESS)
    {
        std::cout << "Communication error with Bulk Read Instruction (dxl_comm_result1)" << std::endl;
        //packetHandler->printTxRxResult(dxl_comm_result);
    }
    for(int i=0; i<20; ++i)
    {
        dxl_getdata_result = groupRead.isAvailable(servos_ids[i],
                                                   ADDR_MX_CURRENT_POSITION,
                                                   ADDR_LEN_CURRENT_POSITION);
        if (dxl_getdata_result != true)
        {
        std::cout << "CM730 -> FAILED TO OBTAIN DATA FROM SERVO " << servos_ids[i] << " IN groupRead.isAvailable()";
        }
        dxl_current_pos = groupRead.getData(servos_ids[i],
                                            ADDR_MX_CURRENT_POSITION,
                                            ADDR_LEN_CURRENT_POSITION);
        servos_goal_position[i] = dxl_current_pos;
        msg_joint_states.position[i] = (int(dxl_current_pos) - int(servos_position_zero[i]))
                                        * SERVO_MX_RADS_PER_BIT 
                                        * servos_is_clockwise[i];
    }
    groupRead.clearParam();
    ros::Duration(1.0).sleep();

    /*---------------------------------------\
    |      HUMANOID IS ALIVE, NOW            |
    -----------------------------------------*/

    while(ros::ok())
    {

        /*---------------------------------------\
        |      READ ALL POSITIONS                |
        -----------------------------------------*/

        for(int i=0; i<20; ++i)
        {
            dxl_addparam_result = groupRead.addParam(servos_ids[i], 
                                ADDR_MX_CURRENT_POSITION, 
                                ADDR_LEN_CURRENT_POSITION);
            if(dxl_addparam_result != true){
                std::cout << "CM730 -> Failed to add id " << servos_ids[i] << " to the list of groupBulkRead" << std::endl;
            }
        }
        dxl_comm_result = groupRead.txRxPacket();
        if(dxl_comm_result != COMM_SUCCESS)
        {
            std::cout << "Communication error with Bulk Read Instruction (dxl_comm_result2)" << std::endl;
            //packetHandler->printTxRxResult(dxl_comm_result);
        }
        for(int i=0; i<20; ++i)
        {
            std::cout << i << std::endl;
            dxl_getdata_result = groupRead.isAvailable(servos_ids[i],
                                                       ADDR_MX_CURRENT_POSITION,
                                                       ADDR_LEN_CURRENT_POSITION);
            if (dxl_getdata_result != true)
            {
            std::cout << "CM730 -> FAILED TO OBTAIN DATA FROM SERVO " << servos_ids[i] << " IN groupRead.isAvailable()";
            }
            dxl_current_pos = groupRead.getData(servos_ids[i],
                                                ADDR_MX_CURRENT_POSITION,
                                                ADDR_LEN_CURRENT_POSITION);
            msg_joint_states.position[i] = (int(dxl_current_pos) - int(servos_position_zero[i]))
                                            * SERVO_MX_RADS_PER_BIT 
                                            * servos_is_clockwise[i];
            msg_joint_current_angles.data[i] = msg_joint_states.position[i];
        }
        groupRead.clearParam();


        /*---------------------------------------\
        |      WRITE ALL POSITIONS (30)          |
        -----------------------------------------*/
        
        for(int i=0; i<20; ++i)
        {
            param_goal_position[0] = DXL_LOBYTE(servos_goal_position[i]);
            param_goal_position[1] = DXL_HIBYTE(servos_goal_position[i]);
            dxl_addparam_result = groupSyncWrite.addParam(servos_ids[i], param_goal_position);
            if (dxl_addparam_result != true)
            {
            std::cout << "CM730 -> ERROR ADDING ID TO GROUP SYNC WRITE (dxl_addparam_result)" << std::endl; 
            }
        }
        dxl_comm_result = groupSyncWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS){
            std::cout << "CM730 -> Communication error (groupSyncWrite.txPacket())" << std::endl;
        }
        groupSyncWrite.clearParam();

       msg_joint_states.header.stamp = ros::Time::now();
       pub_joint_states.publish(msg_joint_states);
       pub_joint_current_angles.publish(msg_joint_current_angles);
       ros::spinOnce();
       loop.sleep();
    }
    portHandler->closePort();
    return 0;
}
