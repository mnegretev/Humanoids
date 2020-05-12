#include <humanoid/Humanoid.h>

int Humanoid::sampling_freq = 60;
bool Humanoid::enable_stop = false;

std::string Humanoid::movement;

ros::ServiceClient Humanoid::clt_get_predef_poses;
ros::ServiceClient Humanoid::clt_speed_profile;

ros::Publisher Humanoid::pub_robot_stop;
ros::Publisher Humanoid::pub_head_positions;
ros::Publisher Humanoid::pub_legs_positions;
ros::Publisher Humanoid::pub_left_arm_positions;
ros::Publisher Humanoid::pub_right_arm_positions;

std::vector<float> Humanoid::left_kick_pose_duration;
std::vector<float> Humanoid::right_kick_pose_duration;
std::vector<float> Humanoid::prone_get_up_pose_duration;
std::vector<float> Humanoid::supine_get_up_pose_duration;
std::vector<float> Humanoid::ready_to_kick_pose_duration;
std::vector<float> Humanoid::current_joint_position(20, 0);

std::vector<std::vector<float> > Humanoid::left_kick_movements;
std::vector<std::vector<float> > Humanoid::right_kick_movements;
std::vector<std::vector<float> > Humanoid::prone_get_up_movements;
std::vector<std::vector<float> > Humanoid::supine_get_up_movements;
std::vector<std::vector<float> > Humanoid::ready_to_kick_movements;
std::vector<std::vector<float> > Humanoid::joint_profile_positions;


void Humanoid::setNodeHandle(ros::NodeHandle* nh) {
    std::cout<< "Humanoid->Setting ros node..." << std::endl;

    Humanoid::clt_get_predef_poses = nh->serviceClient<humanoid_msgs::predefPoses>("/humanoid/get_predef_poses");
    Humanoid::clt_speed_profile    = nh->serviceClient<humanoid_msgs::speedProfile>("/humanoid/get_speed_profile");

    Humanoid::pub_robot_stop = nh->advertise<std_msgs::Bool>("/robot_stop", 1);
    Humanoid::pub_head_positions = nh->advertise<std_msgs::Float32MultiArray>("/hardware/head_goal_pose", 1);
    Humanoid::pub_legs_positions = nh->advertise<std_msgs::Float32MultiArray>("/hardware/legs_goal_pose", 1);
    Humanoid::pub_left_arm_positions = nh->advertise<std_msgs::Float32MultiArray>("/hardware/arm_left_goal_pose", 1);
    Humanoid::pub_right_arm_positions = nh->advertise<std_msgs::Float32MultiArray>("/hardware/arm_right_goal_pose", 1);
}

void Humanoid::getCurrentPose() {
    sensor_msgs::JointStateConstPtr pose = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    for(int id=0; id < 20 ; id++)
      current_joint_position[id] = pose->position[id]; 
}

void Humanoid::stop() {
    std::cout << "Robot stopped" << std::endl;
    std_msgs::BoolConstPtr enable = ros::topic::waitForMessage<std_msgs::Bool>("/robot_stop");
}

void Humanoid::restart() {
    std::cout << "Robot restart" << std::endl;
    std_msgs::Bool enable_msg;
    enable_msg.data = true;
    
    pub_robot_stop.publish(enable_msg); 
    ros::spinOnce();
}

void Humanoid::leftKick() {
    movement = "left_kick";

    Humanoid::getCurrentPose();
    Humanoid::loadPredefPoses(left_kick_movements, left_kick_pose_duration);

    for(int robot_pose=0; robot_pose < left_kick_movements.size(); robot_pose++) {

    	Humanoid::computeSpeedProfile(left_kick_movements,left_kick_pose_duration, robot_pose);
    	Humanoid::publishPositions();
	}
}

void Humanoid::rightKick(){
    movement = "right_kick";

    Humanoid::getCurrentPose();
    Humanoid::loadPredefPoses(right_kick_movements, right_kick_pose_duration);

    for(int robot_pose=0; robot_pose < right_kick_movements.size(); robot_pose++) {

    	Humanoid::computeSpeedProfile(right_kick_movements, right_kick_pose_duration, robot_pose);
    	Humanoid::publishPositions();
	}
}

void Humanoid::proneGetUp() {
    movement = "prone_get_up";
    
    Humanoid::getCurrentPose();
    Humanoid::loadPredefPoses(prone_get_up_movements, prone_get_up_pose_duration);

    for(int robot_pose=0; robot_pose < prone_get_up_movements.size(); robot_pose++) {

    	Humanoid::computeSpeedProfile(prone_get_up_movements, prone_get_up_pose_duration, robot_pose);
    	Humanoid::publishPositions();
	}
}

void Humanoid::supineGetUp() {
    movement = "supine_get_up";

    Humanoid::getCurrentPose();
    Humanoid::loadPredefPoses(supine_get_up_movements, supine_get_up_pose_duration);

    for(int robot_pose=0; robot_pose < supine_get_up_movements.size(); robot_pose++) {

    	Humanoid::computeSpeedProfile(supine_get_up_movements, supine_get_up_pose_duration, robot_pose);
    	Humanoid::publishPositions();
	}
}

void Humanoid::readyToKick() {
    movement = "ready_to_kick";

    Humanoid::getCurrentPose();
    Humanoid::loadPredefPoses(ready_to_kick_movements, ready_to_kick_pose_duration);

    for(int robot_pose=0; robot_pose < ready_to_kick_movements.size(); robot_pose++) {

        if(robot_pose == 5)
            Humanoid::stop();

    	Humanoid::computeSpeedProfile(ready_to_kick_movements, ready_to_kick_pose_duration, robot_pose);
    	Humanoid::publishPositions();
	}
}

void Humanoid::loadPredefPoses(std::vector<std::vector<float> >& all_movements, std::vector<float>& pose_duration) {
    std::cout << "Loading pose from->" << movement << ".yaml" << std::endl;
    
    int robot_pose=0, number_poses;
    humanoid_msgs::predefPoses srv;
    srv.request.pose_to_load = movement;
    
    do{
        srv.request.robot_pose = robot_pose;

        if(clt_get_predef_poses.call(srv)) {
            all_movements.push_back(srv.response.joint_goal_position.data);
            pose_duration.push_back(srv.response.delay);
            number_poses = srv.response.number_poses;
        }
        else 
            ROS_ERROR("Failed to contact with get_predef_poses_server.py :'(");
    }while(++robot_pose < number_poses);
}

void Humanoid::computeSpeedProfile(std::vector<std::vector<float> >& joint_goal_positions, std::vector<float>& profile_time, int robot_pose) {
    //std::cout << "Computing speed profile" << std::endl;
    humanoid_msgs::speedProfile srv;
    srv.request.dt = 1.0/sampling_freq;
    srv.request.tf = profile_time[robot_pose];

    joint_profile_positions.resize(20);

    for(int id=0; id < 20; id++) {
        srv.request.p0 = current_joint_position[id];
        srv.request.pf = joint_goal_positions[robot_pose][id];
        
        if(clt_speed_profile.call(srv))
            joint_profile_positions[id] = srv.response.profiled_positions.data;

        current_joint_position[id] = joint_goal_positions[robot_pose][id];
	}
    
    std::cout << "Robot pose: " << robot_pose << std::endl;
}

void Humanoid::publishPositions() {
    //std::cout << "Publishing position " << std::endl;
    std_msgs::Float32MultiArray head_msg, legs_msg, left_arm_msg, right_arm_msg;

    head_msg.data.resize(2);
    legs_msg.data.resize(12);
    left_arm_msg.data.resize(3);
    right_arm_msg.data.resize(3);

    ros::Rate loop(sampling_freq);
    
    for(int time_k=0; time_k<joint_profile_positions[0].size(); time_k++) {

        for(int id=0; id<20; id++){
           if(id<12)
                legs_msg.data[id] = joint_profile_positions[id][time_k];
            if(id >= 12 && id < 15)
                left_arm_msg.data[id-12] = joint_profile_positions[id][time_k];
            if(id >= 15 && id <18)
                right_arm_msg.data[id-15] = joint_profile_positions[id][time_k];
            if(id >= 18)
                head_msg.data[id-18] = joint_profile_positions[id][time_k];
        }

        if(enable_stop)
            Humanoid::stop();
 
        Humanoid::pub_head_positions.publish(head_msg);
        Humanoid::pub_legs_positions.publish(legs_msg);
        Humanoid::pub_left_arm_positions.publish(left_arm_msg);
        Humanoid::pub_right_arm_positions.publish(right_arm_msg);

        ros::spinOnce();
        loop.sleep();

    }

    loop.sleep();
}
