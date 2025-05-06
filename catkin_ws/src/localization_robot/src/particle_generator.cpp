#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <random>

struct Particle {
    float x, y, theta;
};

float randomFloat(float min, float max) {
    static std::default_random_engine e(std::random_device{}());
    std::uniform_real_distribution<float> dist(min, max);
    return dist(e);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "particle_generator");
    ros::NodeHandle nh;

    ros::Publisher particle_pub = nh.advertise<geometry_msgs::PoseArray>("particle_cloud", 1, true);

    ros::Rate loop_rate(1);  

    // Field parameters
    float x_min = -4.5, x_max = 4.5;
    float y_min = -3.0, y_max = 3.0;
    int num_particles = 500;

    // Generate particles
    std::vector<Particle> particles;
    for (int i = 0; i < num_particles; ++i) {
        Particle p;
        p.x = randomFloat(x_min, x_max);
        p.y = randomFloat(y_min, y_max);
        p.theta = randomFloat(-M_PI, M_PI);
        particles.push_back(p);
    }

    // Convert to PoseArray
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = ros::Time::now();

    for (const auto& p : particles) {
        geometry_msgs::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = 0.0;
        pose.orientation = tf::createQuaternionMsgFromYaw(p.theta);
        pose_array.poses.push_back(pose);
    }

    while (ros::ok()) {
    pose_array.poses.clear();
    pose_array.header.stamp = ros::Time::now();

    for (auto& p : particles) {
        // Simula pequeño movimiento
        p.x += randomFloat(-0.01, 0.01);  // ruido en posición
        p.y += randomFloat(-0.01, 0.01);
        p.theta += randomFloat(-0.02, 0.02);  // ruido en orientación

        geometry_msgs::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = 0.0;
        pose.orientation = tf::createQuaternionMsgFromYaw(p.theta);
        pose_array.poses.push_back(pose);
    }

    particle_pub.publish(pose_array);
    ros::spinOnce();
    loop_rate.sleep();
}

    ros::spinOnce();
    ros::Duration(0.5).sleep();

    return 0;
}