#include "servo_utils.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <iostream>


namespace Servo
{
    CommHandler::CommHandler(const std::string& portName):
    portName(portName.c_str()),
    port_h(dynamixel::PortHandler::getPortHandler(this->portName)),
    packet_h(dynamixel::PacketHandler::getPacketHandler(SERVO_PROTOCOL_VERSION)),
    bulk_read_current_position(dynamixel::GroupBulkRead(port_h, packet_h)),
    sync_write_goal_position(dynamixel::GroupSyncWrite(port_h, packet_h, MX64::GOAL_POSITION, 2))
    {}

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

    bool CommHandler::registerIDs(std::vector<Servo::servo_t>& servo_list)
    {
        if(servo_list.empty())
        {
            std::cout << "[SERVO_UTILS] ERROR, unable to register ids. Servo list is empty" << std::endl;
            return false;
        }
        for(auto servo: servo_list)
        {
            
            int comm_result = bulk_read_current_position.addParam(servo.id, MX64::PRESENT_POSITION, 2);
            if(comm_result != true)
            {
                std::cout << "CM730 -> Failed to add id " << servo.id << " to the list of groupBulkRead" << std::endl;
            }
            registered_ids.push_back(servo.id);
        }
        
        return true;
    }

    bool CommHandler::getAllPositions(std::vector<uint16_t>& present_position)
    {
        int dxl_comm_result = bulk_read_current_position.txRxPacket();
        if(dxl_comm_result != COMM_SUCCESS)
        {
            std::cout << "[SERVO_UTILS] Communication error with Bulk Read Instruction" << std::endl;
            return false;
        }
        for(auto id: registered_ids)
        {
            if(bulk_read_current_position.isAvailable(id, MX64::PRESENT_POSITION, 2))
            {
                present_position[id] = bulk_read_current_position.getData(id, MX64::PRESENT_POSITION, 2);
            }
            else
            {
                std::cout << "[SERVO_UTILS] Could not fetch data from servo: " << id << std::endl;
            }
        }
        return true;
    }
    
    bool CommHandler::setPositions(const std::vector<uint16_t>& position_vector, const std::vector<Servo::servo_t>& servos)
    {
        sync_write_goal_position.clearParam();
        for(auto servo: servos)
        {
            uint8_t param_goal_position[2];
            param_goal_position[0] = DXL_LOBYTE(position_vector[servo.id]);
            param_goal_position[1] = DXL_HIBYTE(position_vector[servo.id]);
            bool dxl_addparam_result = sync_write_goal_position.addParam(servo.id, param_goal_position);
            if(dxl_addparam_result != true){
                std::cout << "[SERVO_UTILS]Failed to add id " << servo.id << " to the list of SyncWrite" << std::endl;
            }
        }
        int dxl_comm_result = bulk_read_current_position.txRxPacket();
        if(dxl_comm_result != COMM_SUCCESS)
        {
            std::cout << "Communication error with Sync Write Instruction" << std::endl;
            return false;
        }
        return true;
    }
};
