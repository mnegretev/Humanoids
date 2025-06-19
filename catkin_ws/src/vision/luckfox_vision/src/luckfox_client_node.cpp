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
#define PORT 5000

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Luckfox_node");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<vision_msgs::ProcessObject>("/intercept_plane_service");
    ros::Publisher pub = n.advertise<vision_msgs::VisionObject>("/vision/landmarks", 1);
    ros::Publisher pub_point = n.advertise<visualization_msgs::MarkerArray>("/vision/landmarks_points", 1);
    vision_msgs::ProcessObject srv;
    std::string ip_server; 
    ros::param::param<std::string>("~ip_server", ip_server, "10.42.0.102");


    if (argc != 3)
    {
        ROS_INFO("Reading UDP from the luckfox");
        // return 1;
    }
    ros::Rate loop_rate(10);

    // Cración de socket
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1)
    {
        std::cerr << "Error al crear el socket del cliente" << std::endl;
        return 1;
    }

    // Configurar la dirección del servidor
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(5000);                     // Puerto 5000
    serverAddress.sin_addr.s_addr = inet_addr(ip_server.c_str()); // Dirección local

    // Conectar al servidor
    if (connect(clientSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1)
    {
        std::cerr << "Error al conectar con el servidor" << std::endl;
        close(clientSocket);
        return 1;
    }

    std::cout << "Conectado al servidor. Recibiendo mensajes..." << std::endl;
    std::cout << "Presiona Ctrl+C para detener el cliente." << std::endl;

    char buffer[1024] = {0};
    char nombre[100];
    int sX, sY, eX, eY;
    float prop;
    

    while (ros::ok())
    {
        memset(buffer, 0, sizeof(buffer));

        // Leer datos del servidor
        int bytesRead = recv(clientSocket, buffer, sizeof(buffer), 0);

        if (bytesRead <= 0)
        {
            std::cerr << "Conexión cerrada por el servidor o error de lectura" << std::endl;
            break;
        }

        // Almacenar y mostrar el mensaje
        std::string message(buffer, bytesRead);
        std::cout << "Mensaje recibido: " << message;

        int elementos_leidos = sscanf(buffer, "%99[^@] @ (%d %d %d %d) %f", nombre, &sX, &sY, &eX, &eY, &prop);

        srv.request.object.id = nombre;
        srv.request.object.confidence = prop;
        srv.request.object.x = sX;
        srv.request.object.y = sY;
        srv.request.object.width = abs(eX - sX);
        srv.request.object.height = abs(eY - sY);
        std::cout << srv.request.object.confidence << std::endl;
        if (client.call(srv))
        {
            ROS_INFO("The service worked well");
            vision_msgs::VisionObject msg;
            visualization_msgs::MarkerArray marks;
            visualization_msgs::Marker marker;
            msg = srv.response.object;
            marker.header.stamp = ros::Time();
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = msg.pose.position.x;
            marker.pose.position.y = msg.pose.position.x;
            marker.pose.position.z = msg.pose.position.x;
            marker.scale.x = 1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            if(msg.id =="ball") 
            {
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marks.markers[0]=marker;
            }
            else if(msg.id =="goal") 
            {
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marks.markers[1]=marker;
            }
            else if(msg.id =="humanoid") 
            {
                marker.type = visualization_msgs::Marker::CUBE;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marks.markers[2]=marker;
            }
            else if(msg.id =="center") 
            {
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.scale.z = 0.01;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marks.markers[3]=marker;
            }
            else
            {
                ROS_ERROR("Objeto desconocido detectado %s",msg.id.c_str());
            }
            pub.publish(msg);
            pub_point.publish(marks);
        }
        else
        {
            ROS_ERROR("Failed to call service Process objects");
            ROS_ERROR("Service call failed: %s", client.getService().c_str());
            return 1;
        }
        ros::spinOnce();
    }

    return 0;
}
