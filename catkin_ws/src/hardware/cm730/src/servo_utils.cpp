#include "servo_utils.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <iostream>

namespace Servo
{
    CommHandler::CommHandler(const std::string & portName):
    portName(portName.c_str()), baudrate(baudrate)
    {
        port_h   =  dynamixel::PortHandler::getPortHandler(this->portName);
        packet_h =  dynamixel::PacketHandler::getPacketHandler(SERVO_PROTOCOL_VERSION);
    }

    bool CommHandler::portIsActive()
    {
        return true;
    }

    bool CommHandler::startComm() 
    {
        if(!(port_h -> openPort()))
        {
            std::cout << "[CommHandler] Could not open port: " << portName << std::endl;
            return false;
        }
        if(!(port_h -> setBaudRate(baudrate)))
        {
            std::cout << "[CommHandler] Failed to set baudrate to "<< baudrate << std::endl;
            return false;
        }

        return true;
    }
    
    bool CommHandler::wakeupAllServos()
    {
        uint8_t error;
        int comm_result = packet_h -> write1ByteTxRx(port_h,
                                                        ID_CM730,
                                                        MX64::TORQUE_ENABLE,
                                                        1,
                                                        &error);
    
        if(comm_result == COMM_SUCCESS && error == 0)
        {
            return true;
        }
        std::cout << "[CommHandler] Failed to send wakeup signal " <<
                   "\tError code:" << int(error) << std::endl;
        return false;
    }

    uint16_t CommHandler::getPosition( const int& id )
    {
        uint16_t position;
        uint8_t error;
        int comm_result = packet_h -> read2ByteTxRx(port_h, 
                                                    id, 
                                                    MX64::PRESENT_POSITION,
                                                    &position,
                                                    &error);
        if(comm_result == COMM_SUCCESS && error == 0)
        {
            return position;
        }
        std::cout << "[CommHandler] Failed to get position from id: " << id 
                  << ".\tError code: " << (int)error << std::endl;
        return -1;
    }
    
    bool CommHandler::setPosition( const int & id, const uint16_t& position )
    {
        uint8_t error;
        int comm_result = packet_h->write2ByteTxRx(port_h,
                                                   id,
                                                   MX64::GOAL_POSITION,
                                                   position,
                                                   &error);
        if(comm_result == COMM_SUCCESS && error == 0)
        {
            return true;
        }
        std::cout << "[CommHandler] Failed to write goal pose "<< (int)position
                  << " to servo "<< id << "\tError code:" << int(error) << std::endl;
        return false;
    }

    bool CommHandler::shutdownAllServos()
    {
        uint8_t error;
        int comm_result = packet_h -> write1ByteTxRx(port_h,
                                                     ID_CM730,
                                                     MX64::TORQUE_ENABLE,
                                                     0,
                                                     &error);
        if(comm_result == COMM_SUCCESS && error == 0)
        {
            return true;
        }
        std::cout << "[CommHandler] Failed to send shutdown signal " <<
                   "\tError code:" << int(error) << std::endl;
        return false; 
    }

    
};
