#include "ros/ros.h"
#include "vision_msgs/ProcessObject.h"
#include <cstdlib>

//socket libs

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
    if (argc !=3)
    {
        ROS_INFO("Reading UDP from the luckfox");
        return 1;
    }
    ros::Rate loop_rate(10);

    //Cración de socket
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) 
    {
        std::cerr << "Error al crear el socket del cliente" << std::endl;
        return 1;
    }

    ros::NodeHandle n; 
    ros::ServiceClient client = n.serviceClient<vision_msgs::ProcessObject>("ProcessObjects");
    vision_msgs::ProcessObject srv;

    // Configurar la dirección del servidor
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(5000); // Puerto 5000
    serverAddress.sin_addr.s_addr = inet_addr("127.0.0.1"); // Dirección local

    // Conectar al servidor
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
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
        
        if (bytesRead <= 0) {
            std::cerr << "Conexión cerrada por el servidor o error de lectura" << std::endl;
            break;
        }

        // Almacenar y mostrar el mensaje
        std::string message(buffer, bytesRead);
        std::cout << "Mensaje recibido: " << message;

        int elementos_leidos = sscanf(cadena, "%99[^@] @ (%d %d %d %d) %f",nombre, &sX, &sY, &eX, &eY, &prop);

        srv.request.object.id = nombre;
        srv.request.object.confidence = prop;
        srv.request.object.x = sX;
        srv.request.object.y = sY;
        srv.request.object.width = abs(sX-sY);
        srv.request.object.height = abs(eX-eY);

        if (client.call(srv))
        {
            
            ROS_INFO("The service worked well");
            ros::Publisher pub = n.advertise<vision_msgs::VisionObject>("Luckfox_finds", 100);
            vision_msgs::VisionObject msg;
            msg = srv.reponse;
            pub.publish(msg);
        }   
        else
        {
            ROS_ERROR("Failed to call service Process objects");
            return 1;
        }
        ros::spinOnce();
    }
    
    

    return 0;
}
