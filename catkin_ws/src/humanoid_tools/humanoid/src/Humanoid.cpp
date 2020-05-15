#include <humanoid/Humanoid.h>

int Humanoid::step_number;
int Humanoid::goal_steps = 0;
int Humanoid::sampling_freq = 60;
bool Humanoid::enable_stop = false;

std::string Humanoid::movement;

ros::ServiceClient Humanoid::clt_speed_profile;
ros::ServiceClient Humanoid::clt_get_predef_poses;
ros::ServiceClient Humanoid::clt_calculate_ik_leg_left;
ros::ServiceClient Humanoid::clt_calculate_ik_leg_right;

ros::Publisher Humanoid::pub_robot_stop;
ros::Publisher Humanoid::pub_head_positions;
ros::Publisher Humanoid::pub_legs_positions;
ros::Publisher Humanoid::pub_left_arm_positions;
ros::Publisher Humanoid::pub_right_arm_positions;
ros::Publisher Humanoid::pub_leg_left_goal_pose;
ros::Publisher Humanoid::pub_leg_right_goal_pose;

ros::Subscriber Humanoid::sub_stop_by_topic;

std::vector<float> Humanoid::left_kick_pose_duration;
std::vector<float> Humanoid::right_kick_pose_duration;
std::vector<float> Humanoid::prone_get_up_pose_duration;
std::vector<float> Humanoid::supine_get_up_pose_duration;
std::vector<float> Humanoid::ready_to_kick_pose_duration;
std::vector<float> Humanoid::current_joint_position(20, 0);

std::vector<std::vector<float> > Humanoid::leg_left_angles;
std::vector<std::vector<float> > Humanoid::leg_right_angles;
std::vector<std::vector<float> > Humanoid::left_kick_movements;
std::vector<std::vector<float> > Humanoid::right_kick_movements;
std::vector<std::vector<float> > Humanoid::prone_get_up_movements;
std::vector<std::vector<float> > Humanoid::supine_get_up_movements;
std::vector<std::vector<float> > Humanoid::ready_to_kick_movements;
std::vector<std::vector<float> > Humanoid::joint_profile_positions;


void Humanoid::setNodeHandle(ros::NodeHandle* nh) {
    std::cout<< "Humanoid->Setting ros node..." << std::endl;

    Humanoid::sub_stop_by_topic = nh->subscribe("/stop_by_topic", 1, &Humanoid::callback_stop_by_topic);

    Humanoid::clt_calculate_ik_leg_left  = nh->serviceClient<ctrl_msgs::CalculateIK>("/control/ik_leg_left");
    Humanoid::clt_calculate_ik_leg_right = nh->serviceClient<ctrl_msgs::CalculateIK>("/control/ik_leg_right"); 
    Humanoid::clt_get_predef_poses = nh->serviceClient<humanoid_msgs::predefPoses>("/humanoid/get_predef_poses");
    Humanoid::clt_speed_profile    = nh->serviceClient<humanoid_msgs::speedProfile>("/humanoid/get_speed_profile");

    Humanoid::pub_robot_stop = nh->advertise<std_msgs::Bool>("/robot_stop", 1);
    Humanoid::pub_head_positions = nh->advertise<std_msgs::Float32MultiArray>("/hardware/head_goal_pose", 1);
    Humanoid::pub_legs_positions = nh->advertise<std_msgs::Float32MultiArray>("/hardware/legs_goal_pose", 1);
    Humanoid::pub_left_arm_positions = nh->advertise<std_msgs::Float32MultiArray>("/hardware/arm_left_goal_pose",   1);
    Humanoid::pub_right_arm_positions = nh->advertise<std_msgs::Float32MultiArray>("/hardware/arm_right_goal_pose", 1);
    Humanoid::pub_leg_left_goal_pose  = nh->advertise<std_msgs::Float32MultiArray>("/hardware/leg_left_goal_pose",  1);
    Humanoid::pub_leg_right_goal_pose = nh->advertise<std_msgs::Float32MultiArray>("/hardware/leg_right_goal_pose", 1);
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

void Humanoid::callback_stop_by_topic(const std_msgs::Bool::ConstPtr& msg) {
    
    enable_stop = msg->data;
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
 
        Humanoid::pub_head_positions.publish(head_msg);
        Humanoid::pub_legs_positions.publish(legs_msg);
        Humanoid::pub_left_arm_positions.publish(left_arm_msg);
        Humanoid::pub_right_arm_positions.publish(right_arm_msg);

        ros::spinOnce();
        loop.sleep();

    }

    loop.sleep();
}

void Humanoid::loadWalkPositions() {
    std::cout << "Loading walk positions..." << std::endl;
    ctrl_msgs::CalculateIK srvIK;

    //Trajectory generation
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

    for(int i=0; i < number_of_points/2; i++) {
        leg_left_x     [i] = left_start_x + 0.01*( -sin (4*M_PI*t));
        leg_left_y     [i] = left_start_y;
    
        float delta_z = 0.01*( -sin (4*M_PI*t));
        if(delta_z < 0)
            delta_z *= 0.9;

        leg_left_z     [i] = left_start_z + delta_z;
        leg_left_roll  [i] = 0;
        leg_left_pitch [i] = 0;
        leg_left_yaw   [i] = 0;
        leg_right_x    [i] = right_start_x;
        leg_right_y    [i] = right_start_y;
        leg_right_z    [i] = right_start_z;
        leg_right_roll [i] = 0;
        leg_right_pitch[i] = 0;
        leg_right_yaw  [i] = 0;
        t += 1.0/30.0;
    }

    t = 0;
    for(int i=number_of_points/2; i < number_of_points; i++) {
        leg_left_x     [i] = left_start_x;
        leg_left_y     [i] = left_start_y;
        leg_left_z     [i] = left_start_z;
        leg_left_roll  [i] = 0;
        leg_left_pitch [i] = 0;
        leg_left_yaw   [i] = 0;
        leg_right_x    [i] = right_start_x + 0.01*( -sin (4*M_PI*t));
        leg_right_y    [i] = right_start_y;

        float delta_z = 0.01*( -sin (4*M_PI*t));
        if(delta_z < 0)
            delta_z *= 0.9;
        
        leg_right_z    [i] = right_start_z + delta_z;
        leg_right_roll [i] = 0;
        leg_right_pitch[i] = 0;
        leg_right_yaw  [i] = 0;
        t += 1.0/30.0;
    }

    for(int i=0; i < number_of_points; i++) {
        srvIK.request.x     = leg_left_x    [i];
        srvIK.request.y     = leg_left_y    [i];
        srvIK.request.z     = leg_left_z    [i];
        srvIK.request.roll  = leg_left_roll [i];
        srvIK.request.pitch = leg_left_pitch[i];
        srvIK.request.yaw   = leg_left_yaw  [i];
 
        if(clt_calculate_ik_leg_left.call(srvIK)) {
            leg_left_angles[i] = srvIK.response.joint_values;
            std::cout << "StepTest.->Left angles: ";
            for(int j=0; j < 6; j++) std::cout << leg_left_angles[i][j] << "\t";
            std::cout << std::endl;
        }

        else {
            std::cout << "walk.->Cannot calculate IK iteration " << i << " values: " << leg_left_x[i] << "\t";
            std::cout << leg_left_y[i] << "\t" << leg_left_z[i] << "\t" << leg_left_roll[i] << "\t" << leg_left_pitch[i] << "\t";
            std::cout << leg_left_yaw[i] << std::endl;
        }
    }

    for(int i=0; i < number_of_points; i++) {
        srvIK.request.x     = leg_right_x    [i];
        srvIK.request.y     = leg_right_y    [i];
        srvIK.request.z     = leg_right_z    [i];
        srvIK.request.roll  = leg_right_roll [i];
        srvIK.request.pitch = leg_right_pitch[i];
        srvIK.request.yaw   = leg_right_yaw  [i];
     
        if(clt_calculate_ik_leg_right.call(srvIK)) {
            leg_right_angles[i] = srvIK.response.joint_values;
            std::cout << "walk.->Right angles: ";
            for(int j=0; j < 6; j++) std::cout << leg_right_angles[i][j] << "\t";
            std::cout << std::endl;
        }

        else {
            std::cout << "StepTest.->Cannot calculate IK iteration " << i << " values: " << leg_right_x[i] << "\t";
            std::cout << leg_right_y[i]<<"\t"<< leg_right_z[i] << "\t" << leg_right_roll[i] << "\t" << leg_right_pitch[i] << "\t";
            std::cout << leg_right_yaw[i] << std::endl;
        }
    }
}

void Humanoid::setStepsNumber(int _goal_steps) {
    goal_steps = _goal_steps;
}

void Humanoid::walk() {
    
    int idx = 0;
    int state = 10;
    ros::Rate loop(40);

    const int number_of_points = 32;
    step_number = 0;

    std_msgs::Float32MultiArray leg_left_goal_pose_msg;
    std_msgs::Float32MultiArray leg_right_goal_pose_msg;

    while(ros::ok() && !enable_stop) {

        if(step_number >= goal_steps && goal_steps != 0)
            break;

	    switch(state) {
        	case 0:
                if(idx < number_of_points/2) {
            		leg_left_goal_pose_msg.data = leg_left_angles [idx];
            		leg_right_goal_pose_msg.data = leg_right_angles[idx];
                    
                    //if(idx == 0)    getchar();
            		pub_leg_left_goal_pose.publish(leg_left_goal_pose_msg);
            		pub_leg_right_goal_pose.publish(leg_right_goal_pose_msg);
            		idx++;

                    if(idx == number_of_points/2 -1) {
                        step_number++;
                        std::cout << "step_number: " << step_number << std::endl;
                   }
	            }

                else if(idx < number_of_points) {
            		leg_left_goal_pose_msg.data = leg_left_angles [idx];
            		leg_right_goal_pose_msg.data = leg_right_angles[idx];

                    //if(idx == number_of_points/2)    getchar();
            		pub_leg_left_goal_pose.publish(leg_left_goal_pose_msg);
            		pub_leg_right_goal_pose.publish(leg_right_goal_pose_msg);
            		idx++;

                    if(idx == number_of_points - 1) {
                        step_number++;
                        std::cout << "step_number: " << step_number << std::endl;
                   }
	            }
                
	            else {
            		idx = 0;
            		state = 10;
	            }

    	    break;

        	case 10:
        	    state = 0;
	        break;

        	default:
	        break;
    	}

        ros::spinOnce();
        loop.sleep();
    }
    
    step_number = 0;
    enable_stop = false;
}
