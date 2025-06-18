#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <random>
#include <vector>

bool initial_hypothesis_published = false;
struct Particle {
    float x, y, theta;
};

float randomFloat(float min, float max) {
    static std::default_random_engine e(std::random_device{}());
    std::uniform_real_distribution<float> dist(min, max);
    return dist(e);
}
// Global variables
std::vector<Particle> particles_cloud;
bool particles_received_from_corrector = false; // Flag that indicates first time or particled filtered received

void filteredParticlesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    if (msg->poses.empty()) {
        ROS_WARN_THROTTLE(1, "Received empty PoseArray from corrector. Skipping update.");
        return;
    }

    particles_cloud.clear(); 
    for (const auto& pose : msg->poses) {
        Particle p;
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.theta = tf::getYaw(pose.orientation);
        particles_cloud.push_back(p);
    }
    particles_received_from_corrector = true; 
    ROS_DEBUG_THROTTLE(1, "Received %zu filtered particles from corrector.", particles_cloud.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "particle_generator");
    ros::NodeHandle nh;

    ros::Publisher filtered_particle_pub = nh.advertise<geometry_msgs::PoseArray>("/particles_cloud", 1, true);
   
    ros::Subscriber filtered_particles_sub = nh.subscribe("/filtered_particles_cloud", 1, filteredParticlesCallback);
; 
    
    ros::Rate loop_rate(1);  

    // Particle distribution in field
    int num_particles = 1000;
    //CASE 1
    float x1_min = -4.5, x1_max = 4.5;
    float y1_min = -3.0, y1_max = 3.0;

    //CASE 2
    float x2_min = 0, x2_max = 4.5;
    float y2_min = -4.0, y2_max = -2.0;

    int initial_robot_position_id = 1;

    ROS_INFO("Initializing particles for the first time...");
    // current_particles.reserve(num_particles); // Pre-asigna memoria

    // Generate particles
    if(initial_robot_position_id ==1){
        for (int i = 0; i < num_particles; ++i) {
        Particle p;
        p.x = randomFloat(x1_min, x1_max);
        p.y = randomFloat(y1_min, y1_max);
        p.theta = randomFloat(-M_PI, M_PI);
        particles_cloud.push_back(p);
    }
    ROS_INFO("Initializing particles around Start Position 1 (x:%.2f, y:%.2f)", x1_min, y1_min);
    }
    else {
        for (int i = 0; i < num_particles; ++i) {
        Particle p;
        p.x = randomFloat(x2_min, x2_max);
        p.y = randomFloat(y2_min, y2_max);
        p.theta = randomFloat(-M_PI, M_PI);
        particles_cloud.push_back(p);
        }
    ROS_INFO("Initializing particles around Start Position 2 (x:%.2f, y:%.2f)", x2_min, y2_min);
    }
    
    //Convert vector paryicles_cloud to Pose Array
    geometry_msgs::PoseArray pose_array_particles_cloud;
    pose_array_particles_cloud.header.frame_id = "map";
    pose_array_particles_cloud.header.stamp = ros::Time::now();

    for(const auto& p : particles_cloud){
        geometry_msgs::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = 0.0;
        pose.orientation = tf::createQuaternionMsgFromYaw(p.theta);
        pose_array_particles_cloud.poses.push_back(pose);
    }

    while (ros::ok()) {

        ros::spinOnce();
        if (!particles_received_from_corrector && !initial_hypothesis_published){
            ROS_INFO("PARTICLES NOT RECEIVED FROM SUBS. PUBLISHING INITIAL HIPOTHESIS");
            pose_array_particles_cloud.poses.clear();
            pose_array_particles_cloud.header.stamp = ros::Time::now();
            for (auto& p : particles_cloud){
                p.x += randomFloat(-0.01, 0.01);
                p.y += randomFloat(-0.01, 0.01);
                p.theta += randomFloat(-0.02, 0.02);

                geometry_msgs::Pose pose;
                pose.position.x = p.x;
                pose.position.y = p.y;
                pose.position.z = 0.0;
                pose.orientation = tf::createQuaternionMsgFromYaw(p.theta);
                pose_array_particles_cloud.poses.push_back(pose);

            }
            filtered_particle_pub.publish(pose_array_particles_cloud);
            ros::spinOnce();
            loop_rate.sleep();
            initial_hypothesis_published = true;
            continue;;
            }
        if (particles_received_from_corrector) {
            ROS_INFO("PARTICLES RECEIVED FROM CORRECTOR");
            geometry_msgs::PoseArray pose_array;
            pose_array.header.frame_id = "map";
            pose_array.header.stamp = ros::Time::now();

            float delta_x_odom = 0.05; // Simulación de movimiento hacia adelante
            float delta_y_odom = 0.0;
            float delta_theta_odom = 0.01; // Simulación de giro

            for (auto& p : particles_cloud) {
                p.x += delta_x_odom * cos(p.theta) - delta_y_odom * sin(p.theta);
                p.y += delta_x_odom * sin(p.theta) + delta_y_odom * cos(p.theta);
                p.theta += delta_theta_odom;

                // Añade ruido al movimiento para cada partícula (incertidumbre)
                p.x += randomFloat(-0.01, 0.01);  // Ruido en posición x
                p.y += randomFloat(-0.01, 0.01);  // Ruido en posición y
                p.theta += randomFloat(-0.02, 0.02); // Ruido en orientación theta

                // Normaliza el ángulo theta de la partícula después de la adición de ruido
                p.theta = atan2(sin(p.theta), cos(p.theta)); 

                geometry_msgs::Pose pose;
                pose.position.x = p.x;
                pose.position.y = p.y;
                pose.position.z = 0.0;
                pose.orientation = tf::createQuaternionMsgFromYaw(p.theta);
                pose_array.poses.push_back(pose);
            }
            filtered_particle_pub.publish(pose_array);
            particles_received_from_corrector = false; 
        }
        else {
            ROS_WARN_THROTTLE(1, "current_particles is empty after initialization.");            
        }

    }
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    return 0;
}