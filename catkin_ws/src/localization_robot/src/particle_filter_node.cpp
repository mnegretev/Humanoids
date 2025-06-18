// //BASED ON: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7402168
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <vision_msgs/VisionObject.h>
#include <tf/tf.h>
#include <random>
#include <cmath>
#include <vector>
#include <map>
// --- Structs ---
struct Particle {
    float x, y, theta, weight;
};
struct Landmark {
    int id;
    float x, y;
    std::string name;
};

// --- Filter global variables ---
std::vector<Particle> current_particles;
std::map<int, Landmark> known_landmarks_map;

// --- Flags ---
bool particles_initialized = false;
bool new_vision_data_arrived = false;

// --- Helper fuctions ---
float getYawFromPose(const geometry_msgs::Pose& pose){
    return tf::getYaw(pose.orientation);
}
/* --- EXPECTED ANGLE ---
The position of the robot is then obtained by the mean of the particles 
pose 〈x, y, θ〉, where the mean of θ  is calculated by the Equation:
α_exp​=atan2(y_lm​ − y_p​, x_lm​ − x_p​)− θp 
This is de lm angle from the PARTICLE's pov.
*/
float expectedAngle(const Particle& p, const Landmark& lm){
    float dx = lm.x - p.x;
    float dy = lm.y - p.y;
    float angle = atan2(dy, dx) - p.theta; //Local robot angle from lm
    return atan2(sin(angle), cos(angle)); // Normalize [-pi, pi]
}
/* --- SIMILARITY ---
The similarity σ is obtained by applying a sigmoid function to the 
difference between the measured angle αseen and the expected angle αexp
*/

float similarity(float alpha_seen, float alpha_exp){
    float d = std::abs (alpha_seen - alpha_exp);
    d =std::abs (atan2(sin(d), cos(d)))/M_1_PI;
// ternary operator: condition ? exp_if_true : exp_if_false;
    return std::exp (-50.0f * std::pow(d,2));
}

// Necessary because weights are represented as a probability distribution.
void normalizeWeights(std::vector<Particle>& particles){
    float sum = 0.0f;
    for (const auto& p : particles) sum += p.weight;
    if (sum == 0.0f) { // Avoids division by zero if all weights are 0
        ROS_WARN("All particle weights are zero! Reinitializing particles.");
        // Re-asign uniform weights (soft reset)
        float uniform_weight = 1.0f / particles.size();
        for (auto& p : particles) p.weight = uniform_weight;
        return;
    }

    for (auto& p : particles) p.weight /= sum;
}

// Low Variance Resampling
std::vector<Particle> resample(const std::vector<Particle>& particles) {
    std::vector<Particle> new_particles;
    int N = particles.size();
    if (N == 0) return new_particles;

    std::vector<float> cumulative_weights(N); //CDF cumulative distribution function
    cumulative_weights[0] = particles[0].weight;
    for (int i = 1; i<N; ++i){
        cumulative_weights[i] = cumulative_weights[i-1]+particles[i].weight;
    }
    // CDF ends
    //STEP 3:
    float step = 1.0f / N;
    float r = static_cast<float>(rand())/RAND_MAX * step; //ensures the algorithm is not deterministic
    int index = 0;

    for(int i = 0; i < N; i++){
        float u = r + i * step; // Actual sampler point
         while (index < N && u > cumulative_weights[index]) { // Looks for the particle segment
            index++;
        }
        // Index between limits
        if (index >= N) index = N - 1; 

         new_particles.push_back(particles[index]);
    }
    /*este proceso garantiza que los N puntos de muestreo (u) se distribuyan de 
    manera uniforme a lo largo de la línea numérica (0 a 1 donde cada peso representa un espacio), y que cada vez que un u cae 
    en un segmento, la partícula correspondiente sea seleccionada. Como los 
    segmentos más grandes corresponden a partículas con mayor peso, estas tienen 
    más oportunidades de ser seleccionadas (duplicadas).*/
    return new_particles;
}
// --- ROS Callback functions 
// Initialize the particles
void  predictedParticlesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    current_particles.clear();
    for(const auto& pose : msg->poses){
        Particle p;
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.theta = getYawFromPose(pose);
        p.weight = 1.0f; // Reset weights from prediction for the update step
        current_particles.push_back(p);
    }
    particles_initialized=true;
}

//STEP 2: UPDATE (weighting)
// This function updates particle weights based on a new observation
void visionCallback(const vision_msgs::VisionObject::ConstPtr& msg){
    new_vision_data_arrived= true;

    if (current_particles.empty()) {
        ROS_INFO( "Vision data received but particles not initialized or empty. Skipping update.");
    return;
    }
    /*--- Landmark Identification: Heuristic Method ---
     1. Robot's Y-axis is to its left (+Y).
     2. Map's goals are at Y=0 (4.5, 0.0) and (-4.5, 0.0).
     3. msg->pose.position.y is the Y-coordinate of the detected object
        in the robot's local frame.*/

    Landmark detected_map_landmark;
    bool landmark_identified = false;
    float x_robot_frame = msg->pose.position.x; //Y yolo detection in robot's frame
    const float X_TRESH = 0.01;

    if (known_landmarks_map.count(1) && known_landmarks_map.count(2)) { 
        ROS_INFO("CHOSING GOAL LANDMARK");
        std::cout<<x_robot_frame<<std::endl;
        if (x_robot_frame > X_TRESH) { // Goal is to the robot's left (positive Y)
            detected_map_landmark = known_landmarks_map[2]; // Assign to the internally defined LEFT goal (ID 2)
            landmark_identified = true;
            ROS_INFO("Vision: Object (robot_Y:%.2f) heuristically assigned to LEFT goal ('%s').",
                      x_robot_frame, detected_map_landmark.name.c_str());

        } else if (x_robot_frame < -X_TRESH) { // Goal is to the robot's right 
            detected_map_landmark = known_landmarks_map[1]; 
            landmark_identified = true;
            ROS_INFO("Vision: Object (robot_Y:%.2f) heuristically assigned to RIGHT goal ('%s').",
                      x_robot_frame, detected_map_landmark.name.c_str());

        } else {
            // Ambiguous case: Goal is straight ahead. Skip update.
            ROS_WARN_THROTTLE(1, "Observed goal directly in front (robot_X:%.2f) within threshold. Cannot disambiguate with X-heuristic. Skipping update.", x_robot_frame);
            return;
        }
    } else {
        ROS_WARN_THROTTLE(1, "Known landmarks not properly defined in map.");
        return;
    }
    float angle_seen = std::atan2(msg->pose.position.y, msg->pose.position.x);
    std::cout<<angle_seen<<std::endl;
    for (auto& p : current_particles){
        float angle_expected = expectedAngle(p, detected_map_landmark);
        p.weight *= similarity(angle_seen,angle_expected);
    }
    
}

int main (int argc, char** argv){
    ros::init(argc, argv, "particle_filter_node");
    ros:: NodeHandle nh;

    // --ROS Subscribers ---
    ros::Subscriber vision_sub = nh.subscribe("/vision/landmarks", 1, visionCallback);
    ros::Subscriber particle_sub = nh.subscribe("/particles_cloud", 1, predictedParticlesCallback);
    
    // --ROS Publishers ---
    ros::Publisher filtered_particles_pub = nh.advertise<geometry_msgs::PoseArray>("/filtered_particles_cloud", 1);ros::Rate rate(10); // 10 Hz, por ejemplo
    ros::Publisher estimated_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/estimated_robot_pose", 1);
    
    known_landmarks_map[1] = {1, 4.5f, 0.0f, "right_goal"};
    known_landmarks_map[2] = {2, -4.5f, 0.0f, "left_goal"};
    

    while (ros::ok()) { 
        ros::spinOnce(); 
        // --- Particle Filter Cycle (Correction & Resampling) ---

        if (!particles_initialized || current_particles.empty()) { // Checks for particles response in predictedParticles
            ROS_INFO("Waiting for initial predicted particles to start filter cycle..."); 
            rate.sleep(); 
            continue; // Returns to while
        }

        if (new_vision_data_arrived) {
            normalizeWeights(current_particles); 
            ROS_INFO("Normalized weights.");

            current_particles = resample(current_particles);
            ROS_INFO("Resampled particles. New count: %zu", current_particles.size());
            // Robot pose estimation
            float estimated_x = 0.0f;
            float estimated_y = 0.0f;
            float sum_sin_theta = 0.0f;
            float sum_cos_theta = 0.0f;
            for (const auto& p : current_particles) {
                estimated_x += p.x;
                estimated_y += p.y;
                sum_sin_theta += sin(p.theta);
                sum_cos_theta += cos(p.theta);
            }
            // Average x, y
            estimated_x /= current_particles.size();
            estimated_y /= current_particles.size();
            // Circular mean
            float estimated_theta = atan2(sum_sin_theta, sum_cos_theta);

            // Publish the robot pose estimation
            geometry_msgs::PoseStamped estimated_pose_msg;
            estimated_pose_msg.pose.position.x = estimated_x;
            estimated_pose_msg.pose.position.y = estimated_y;
            estimated_pose_msg.pose.position.z = 0.0; 
            estimated_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(estimated_theta); 
            estimated_pose_pub.publish(estimated_pose_msg); // Publica la pose
            ROS_INFO("Published estimated pose: x=%.2f, y=%.2f, theta=%.2f", estimated_x, estimated_y, estimated_theta);
             // Particles cloud publishing
            geometry_msgs::PoseArray particle_cloud_msg;
            particle_cloud_msg.header.frame_id = "map"; 
            for (const auto& p : current_particles) { 
                geometry_msgs::Pose pose;
                pose.position.x = p.x;
                pose.position.y = p.y;
                pose.position.z = 0.0;
                pose.orientation = tf::createQuaternionMsgFromYaw(p.theta); 
                particle_cloud_msg.poses.push_back(pose);
            }
            filtered_particles_pub.publish(particle_cloud_msg); // Publica el array de poses
            ROS_INFO("Published %zu filtered particles.", current_particles.size());
            new_vision_data_arrived = false;
        } 
        else {
             ROS_INFO("No new vision data this cycle. Skipping normalize/resample.");
        }

        rate.sleep();   
    }

    return 0;
}