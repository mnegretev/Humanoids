#include "ros/ros.h"
#include "vision_msgs/ProcessObject.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PointStamped.h"
#include <cstdlib>

// socket libs
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>

#define UDP_PORT 5000
#define BUFFER_SIZE 1024

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Luckfox_node");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<vision_msgs::ProcessObject>("/intercept_plane_service");
    ros::Publisher pub = n.advertise<vision_msgs::VisionObject>("/vision/landmarks", 1);
    ros::Publisher pub_point = n.advertise<visualization_msgs::MarkerArray>("/vision/landmarks_points", 1);
    
    vision_msgs::ProcessObject srv;
    std::string ip_server; 
    ros::param::param<std::string>("~ip_server", ip_server, "127.0.0.1"); // Cambiado a localhost por defecto
    
    // Configuración del socket UDP
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
    char buffer[BUFFER_SIZE];
    
    // Crear socket UDP
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        ROS_ERROR("Error al crear el socket UDP");
        return 1;
    }
    
    // Configurar timeout de recepción
    struct timeval tv;
    tv.tv_sec = 1;  // 1 segundo de timeout
    tv.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
    
    // Configurar dirección del servidor
    servaddr.sin_family = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = inet_addr(ip_server.c_str()); // Usar la IP del parámetro
    servaddr.sin_port = htons(UDP_PORT);
    
    // Vincular el socket a la dirección
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        ROS_ERROR("Error al vincular el socket UDP");
        return 1;
    }
    
    ROS_INFO("Esperando datos UDP en %s:%d...", ip_server.c_str(), UDP_PORT);

    char nombre[100];
    int sX, sY, eX, eY;
    float prop;

    while (ros::ok())
    {
        memset(buffer, 0, BUFFER_SIZE);
        socklen_t len = sizeof(cliaddr);
        
        // Recibir datos UDP
        int bytesRead = recvfrom(sockfd, buffer, BUFFER_SIZE, 
                                MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);
        
        if (bytesRead <= 0)
        {
            if (bytesRead == -1)
                ROS_DEBUG("Timeout o error en recepción UDP");
            continue;
        }
        
        buffer[bytesRead] = '\0'; // Asegurar terminación de string
        ROS_DEBUG("Mensaje UDP recibido: %s", buffer);

        // Parsear mensaje (formato: nombre@(x1,y1,x2,y2)conf)
        int elementos_leidos = sscanf(buffer, "%99[^@]@(%d,%d,%d,%d)%f", nombre, &sX, &sY, &eX, &eY, &prop);

        if (elementos_leidos != 6)
        {
            ROS_WARN("Formato de mensaje incorrecto. Se esperaba: nombre@(x1,y1,x2,y2)conf");
            continue;
        }

        // Preparar mensaje para el servicio ROS
        srv.request.object.id = nombre;
        srv.request.object.confidence = prop;
        srv.request.object.x = sX;
        srv.request.object.y = sY;
        srv.request.object.width = abs(eX - sX);
        srv.request.object.height = abs(eY - sY);

        if (client.call(srv))
        {
            ROS_DEBUG("Service call successful");
            vision_msgs::VisionObject msg = srv.response.object;
            
            // Crear marcadores de visualización
            visualization_msgs::MarkerArray marks;
            visualization_msgs::Marker marker;
            
            marker.header.frame_id = "map";  // Ajustar según tu frame_id
            marker.header.stamp = ros::Time::now();
            marker.ns = "vision_objects";
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = msg.pose.position;
            marker.pose.orientation.w = 1.0;
            
            // Configurar marcador según el tipo de objeto
            if(msg.id == "ball") 
            {
                marker.id = 0;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
                marker.color.a = 1.0; // No olvidar el alpha
                marks.markers.push_back(marker);
            }
            else if(msg.id == "goal") 
            {
                marker.id = 1;
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.scale.x = marker.scale.y = 0.5; marker.scale.z = 1.0;
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
                marker.color.a = 1.0; // No olvidar el alpha
                marks.markers.push_back(marker);
            }
            // Añadir más casos según necesites
            
            pub.publish(msg);
            pub_point.publish(marks);
        }
        else
        {
            ROS_ERROR("Failed to call service Process objects");
        }

        ros::spinOnce();
    }

    close(sockfd);
    return 0;
}