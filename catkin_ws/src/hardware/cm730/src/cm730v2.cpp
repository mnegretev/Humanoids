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

#define ID_CM730    200                    //-> ROS THINKS CM730 IS A SERVO

/*---------------------------------------
|            UPPER BODY                  |
-----------------------------------------*/

#define ID_ARM_LEFT_SHOULDER_PITCH  2      //-> LEFT ARM
#define ID_ARM_LEFT_SHOULDER_ROLL   4
#define ID_ARM_LEFT_ELBOW_PITCH     6


#define ID_ARM_RIGHT_SHOULDER_PITCH 1      // --> RIGHT SHOULDER
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

#define ZERO_LEG_LEFT_HIP_YAW       2061
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

#define SERVO_MX_BITS_PER_RAD       651.898646904   // Servos have 4096 different positions, 
                                                    // since there are 2  PI radians in a circumference, 
                                                    // each radian has 4096/2*PI = 651.898646904 BITS PER RADIAN

#define SERVO_MX_RADS_PER_BIT       0.001533981     // Inverse of SERVO_MX_BITS_PER_RAD

#define SERVO_PROTOCOL_VERSION      1.0             //  SERVOS USED BY THE HUMANOID USE PROTOCOL 1.0

/*
    DYNAMIXEL SERVOS HAVE UP TO 70 DIFFERENT CONTROL OPTIONS
    EITHER TO READ DATA OR WRITE DATA. THE NUMBERS SHOWN BELOW
    ARE THE ACTUAL ADDRESSES THAT WE WILL BE USING. FOR THIS NODE
    WE WILL NEED TO TURN ON THE MOTOR (24), REVIEW THE CURRENT
    POSITION OF THE MOTOR (36) AND SEND A DESIRED GOAL POSITION (30)

    TO SEE THE COMPLETE LIST OF CONTROL OPTIONS VISIT:
    https://emanual.robotis.com/docs/en/dxl/mx/mx-64/#control-table-of-eeprom-area

*/

#define ADDR_MX_CURRENT_POSITION    36      // PRESENT POSITION
#define ADDR_LEN_CURRENT_POSITION   2       // SIZE OF ADDRESS = 2 BYTES

#define ADDR_MX_GOAL_POSITION       30      // GOAL POSITION
#define ADDR_LEN_GOAL_POSITION      2       // SIZE OF ADDRESS = 2 BYTES

#define ADDR_CM730_DYNAMIXEL_POWER  24      // TORQUE ENABLE ADDRESS

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

/*------------------------------------------*\
|               CALLBACKS                    |
|-------------------------------------------*/

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
    
    /*----------------------------------------------*\
    |               PORT AND PACKET HANDLER          |
    |-----------------------------------------------*/

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

    /*----------------------------------------------*\
    |               BAUDRATE                         |
    |-----------------------------------------------*/
    
    if(portHandler->setBaudRate(1000000))
        std::cout << "CM730.->Baudrate successfully set to " << std::endl;
    else
    {
        std::cout << "CM730.->Cannot set baud rate" << std::endl;
        return -1;
    }

    dynamixel::GroupBulkRead groupRead(portHandler, packetHandler);     // SyncRead is not supported for protocol 1.0, only for protocol 2.0,
                                                                        // instead we use GroupBulkRead
                                                                        // BulkRead allows us to read data from multiple addreses
                                                                        // but for the moment we only need to read position
    
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler,               // We don't need BulkWrite since we are only writing to a single address
                                             packetHandler,             // SyncWrite is enough because we are only reading at the same address in
                                             ADDR_MX_GOAL_POSITION,     // all ID's
                                             ADDR_LEN_GOAL_POSITION);

    bool dxl_addparam_result = false;                                   // Verify if the ID was succesfully added to the list of BulkRead
    bool dxl_getdata_result = false;                                    // Verify if data result is succesful

    uint8_t  dxl_error       = 0;                                       // Shows any error returned by the servo
    int      dxl_comm_result = COMM_TX_FAIL;                            // Communication Status
    uint16_t dxl_current_pos;                                           // STORES THE VALUE FROM SERVO
    uint8_t param_goal_position[2];                                     // SyncWrite requires to store 16-bit int into an array of 8-bit

    /*---------------------------------------
    |      WAKE UP CM730 (ADDR 24)          |
    -----------------------------------------*/

    std::cout << "\n\tCM730 -> WAKE UP!!\n" << std::endl;

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,
                                                    ID_CM730,
                                                    ADDR_CM730_DYNAMIXEL_POWER,
                                                    1,
                                                    &dxl_error);
    
    if(dxl_comm_result != COMM_SUCCESS)
        std::cout << "CM730.-> Commnunication problem while trying to wake up CM730" << std::endl;
    
    if(dxl_error != 0)
        std::cout << "CM730.-> Status error after turning on dynamixel power: " << int(dxl_error) << std::endl;

    ros::Duration(1.0).sleep();
   
    /*---------------------------------------
    |      READ ALL POSITIONS (ADDR 36)      |
    -----------------------------------------*/

    for(int i=0; i<20; ++i)
    {
        dxl_addparam_result = groupRead.addParam(servos_ids[i],                 // Pushes ID, ADDR and DATA_LENGHT to the list
                                                 ADDR_MX_CURRENT_POSITION,      // Returns false when ID already exists in the list
                                                 ADDR_LEN_CURRENT_POSITION);

        if(dxl_addparam_result != true)
            std::cout << "CM730 -> Failed to add id " << servos_ids[i] << " to the list of groupBulkRead (First process)" << std::endl;

    } 

    dxl_comm_result = groupRead.txRxPacket();       // Transmits and receives the packet

    if(dxl_comm_result != COMM_SUCCESS){
        std::cout << "\nCommunication error with Bulk Read Instruction (dxl_comm_result)\n" << std::endl;
        return 0;
    }

    for(int i=0; i<20; ++i)
    {
        dxl_getdata_result = groupRead.isAvailable(servos_ids[i],               // Checks wether there is available data in the storage. 
                                                   ADDR_MX_CURRENT_POSITION,    // Returns false if there is no data from target address
                                                   ADDR_LEN_CURRENT_POSITION);

        if (dxl_getdata_result != true)
        {
            std::cout << "CM730 -> FAILED TO OBTAIN DATA FROM SERVO " << servos_ids[i] << " in groupRead.isAvailable()";
        }
        else
        {
            dxl_current_pos = groupRead.getData(servos_ids[i],                  // Gets data from received packet at specific address
                                                ADDR_MX_CURRENT_POSITION,
                                                ADDR_LEN_CURRENT_POSITION);

            servos_goal_position[i] = dxl_current_pos;          

            msg_joint_states.position[i] = (int(dxl_current_pos) - int(servos_position_zero[i]))
                                            * SERVO_MX_RADS_PER_BIT * servos_is_clockwise[i];
        }
    }

    groupRead.clearParam();     // This function clears the groupRead ID list. If we miss this instruction, the list will
                                // not be cleared and the id's will keep adding up.
    
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
                std::cout << "CM730 -> Failed to add id " << (int)servos_ids[i] << " to the list of groupBulkRead" << std::endl;
            }
        }

        dxl_comm_result = groupRead.txRxPacket();
        
        if(dxl_comm_result != COMM_SUCCESS)
        {
            std::cout << "\n\tCommunication error with Bulk Read Instruction (Second Process)\n" << std::endl;
        }

        for(int i=0; i<20; ++i)
        {
            dxl_getdata_result = groupRead.isAvailable(servos_ids[i],
                                                       ADDR_MX_CURRENT_POSITION,
                                                       ADDR_LEN_CURRENT_POSITION);
            if (dxl_getdata_result != true)
            {
            std::cout << "CM730 -> FAILED TO OBTAIN DATA FROM SERVO " << (int)servos_ids[i] << " IN groupRead.isAvailable()";
            }
            else
            {
                dxl_current_pos = groupRead.getData(servos_ids[i],
                                                ADDR_MX_CURRENT_POSITION,
                                                ADDR_LEN_CURRENT_POSITION);
                msg_joint_states.position[i] = (int(dxl_current_pos) - int(servos_position_zero[i]))
                                            * SERVO_MX_RADS_PER_BIT 
                                            * servos_is_clockwise[i];
                msg_joint_current_angles.data[i] = msg_joint_states.position[i];
            }

        }

        groupRead.clearParam();


        /*---------------------------------------\
        |      WRITE ALL POSITIONS (addr 30)     |
        -----------------------------------------*/
        
        for(int i=0; i<20; ++i)
        {
            param_goal_position[0] = DXL_LOBYTE(servos_goal_position[i]);           // SyncWrite can only receive uint8_t arrays
            param_goal_position[1] = DXL_HIBYTE(servos_goal_position[i]);           // since position is uint16_t, we split it with LOBYTE() AND HIBYTE()

            dxl_addparam_result = groupSyncWrite.addParam(servos_ids[i], 
                                                          param_goal_position);     // Pushes Id to the SyncWrite list, and initializes storage

            if (dxl_addparam_result != true)
                std::cout << "CM730 -> ERROR ADDING ID TO GROUP SYNC WRITE (dxl_addparam_result)" << std::endl;
        }

        dxl_comm_result = groupSyncWrite.txPacket();      // Transmits packet to the number of servos

        if (dxl_comm_result != COMM_SUCCESS){
            std::cout << "CM730 -> Communication error while writing positions (groupSyncWrite.txPacket())" << std::endl;
            return 0;
        }

        groupSyncWrite.clearParam();

        msg_joint_states.header.stamp = ros::Time::now();
        pub_joint_states.publish(msg_joint_states);
        pub_joint_current_angles.publish(msg_joint_current_angles);
        ros::spinOnce();
        loop.sleep();
    }
    /*---------------------------------------\
    |  KILLING PROCESS, SHUTTING DOWN MOTORS |
    -----------------------------------------*/
     dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,
                                                    ID_CM730,
                                                    ADDR_CM730_DYNAMIXEL_POWER,
                                                    0,
                                                    &dxl_error);


    if(dxl_comm_result != COMM_SUCCESS)
        std::cout << "CM730.-> Commnunication problem while trying to killing process CM730" << std::endl;
    
    if(dxl_error != 0)
        std::cout << "CM730.-> Status error after turning off dynamixel power: " << int(dxl_error) << std::endl;

    portHandler->closePort();
    return 0;
    //DONE MOTORS OFF
    //>:( 
}
