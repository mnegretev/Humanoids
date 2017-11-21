#include "ros/ros.h" 
#include "std_msgs/Float32.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#define DEVICE_NAME      "/dev/ttyUSB0"
#define BAUDRATE         57600
#define PROTOCOL_VERSION 1.0

#define DXL_ID                          1                   
#define BAUDRATE                        57600

int main(int argc, char** argv)
 {     
    uint8_t  dxl_error;
    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if(portHandler->openPort())
  std::cout << "Setup.->Serial port successfully openned" << std::endl;
    else
    {
  std::cout << "Setup.->Cannot open serial port" << std::endl;
  return -1;
    }
    if(portHandler->setBaudRate(BAUDRATE))
  std::cout << "Setup.->Baudrate successfully set to " << BAUDRATE << std::endl;
    else
    {
  std::cout << "Setup.->Cannot set baud rate" << std::endl;
  return -1;
    }
      packetHandler->write1ByteTxRx(portHandler, DXL_ID, 3, 1, &dxl_error); //Changes ID
}

