#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <cmath>

ros::Publisher marker_pub;

geometry_msgs::Point makePoint(float x, float y) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    return p;
}

void publishLineStrip(const std::vector<geometry_msgs::Point>& points, int id, float r, float g, float b) {
    visualization_msgs::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.ns = "field_lines";
    line.id = id;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.08;  // Grosor de línea: 2 cm
    line.color.r = r;
    line.color.g = g;
    line.color.b = b;
    line.color.a = 1.0;

    line.points = points;

    marker_pub.publish(line);
}

void publishCircle(float cx, float cy, float radius, int id, float r, float g, float b) {
    std::vector<geometry_msgs::Point> circle_points;
    int segments = 100;
    for (int i = 0; i <= segments; ++i) {
        float theta = 2.0 * M_PI * float(i) / float(segments);
        float x = cx + radius * cos(theta);
        float y = cy + radius * sin(theta);
        circle_points.push_back(makePoint(x, y));
    }
    publishLineStrip(circle_points, id, r, g, b);
}

void publishDot(float x, float y, int id) {
    visualization_msgs::Marker dot;
    dot.header.frame_id = "map";
    dot.header.stamp = ros::Time::now();
    dot.ns = "penalty_spot";
    dot.id = id;
    dot.type = visualization_msgs::Marker::SPHERE;
    dot.action = visualization_msgs::Marker::ADD;
    dot.pose.position = makePoint(x, y);
    dot.scale.x = dot.scale.y = dot.scale.z = 0.05; // 5cm
    dot.color.r = 1.0;
    dot.color.g = 1.0;
    dot.color.b = 1.0;
    dot.color.a = 1.0;

    marker_pub.publish(dot);
}

void createField() {
    float field_length = 9.0;
    float field_width = 6.0;
    float border = 1.0;

    float total_length = field_length + 2 * border;
    float total_width = field_width + 2 * border;

    float start_x = -field_length/2.0;
    float start_y = -field_width/2.0;

    float end_x = start_x + field_length;
    float end_y = start_y + field_width;

    float center_x = (start_x + end_x) / 2.0;
    float center_y = (start_y + end_y) / 2.0;

    int id = 0;

    // green background
    visualization_msgs::Marker ground;
    ground.header.frame_id = "map";
    ground.header.stamp = ros::Time::now();
    ground.ns = "field";
    ground.id = id++;
    ground.type = visualization_msgs::Marker::CUBE;
    ground.action = visualization_msgs::Marker::ADD;
    ground.pose.position.x = 0.0;
    ground.pose.position.y = 0.0;
    ground.pose.position.z = -0.01; // under lines
    ground.scale.x = total_length;
    ground.scale.y = total_width;
    ground.scale.z = 0.01;
    ground.color.r = 0.1f;
    ground.color.g = 0.5f;
    ground.color.b = 0.1f;
    ground.color.a = 1.0f;
    marker_pub.publish(ground);

    // Field length and width
    publishLineStrip({
        makePoint(start_x, start_y), makePoint(end_x, start_y),
        makePoint(end_x, end_y), makePoint(start_x, end_y), makePoint(start_x, start_y)
    }, id++, 1.0, 1.0, 1.0);

    //  Center line
    publishLineStrip({
        makePoint(center_x, start_y), makePoint(center_x, end_y)
    }, id++, 1.0, 1.0, 1.0);

    // Círculo central
    publishCircle(center_x, center_y, 0.75, id++, 1.0, 1.0, 1.0);

    // Penalty mark
    publishDot(start_x + 1.5, center_y, id++);
    publishDot(end_x - 1.5, center_y, id++);

    // Penalty area (left)
    publishLineStrip({
        makePoint(start_x, start_y + 0.5), makePoint(start_x + 2.0, start_y + 0.5),
        makePoint(start_x + 2.0, end_y - 0.5), makePoint(start_x, end_y - 0.5)
    }, id++, 1.0, 1.0, 1.0);

    // Penalty area (right)
    publishLineStrip({
        makePoint(end_x, start_y + 0.5), makePoint(end_x - 2.0, start_y + 0.5),
        makePoint(end_x - 2.0, end_y - 0.5), makePoint(end_x, end_y - 0.5)
    }, id++, 1.0, 1.0, 1.0);

    // Goal areas
    publishLineStrip({
        makePoint(start_x, start_y + 1.5), makePoint(start_x + 1.0, start_y + 1.5),
        makePoint(start_x + 1.0, end_y - 1.5), makePoint(start_x, end_y - 1.5)
    }, id++, 1.0, 1.0, 1.0);

    publishLineStrip({
        makePoint(end_x, start_y + 1.5), makePoint(end_x - 1.0, start_y + 1.5),
        makePoint(end_x - 1.0, end_y - 1.5), makePoint(end_x, end_y - 1.5)
    }, id++, 1.0, 1.0, 1.0);

    // Goal depth
    publishLineStrip({
        makePoint(start_x, start_y + 1.9), makePoint(start_x - 0.6, start_y + 1.9),
        makePoint(start_x - 0.6, end_y - 1.9), makePoint(start_x, end_y - 1.9)
    }, id++, 1.0, 1.0, 1.0);

    publishLineStrip({
        makePoint(end_x, start_y + 1.9), makePoint(end_x + 0.6, start_y + 1.9),
        makePoint(end_x + 0.6, end_y - 1.9), makePoint(end_x, end_y - 1.9)
    }, id++, 1.0, 1.0, 1.0);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "map_node");
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate rate(1.0);
    while (ros::ok()) {
        createField();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}