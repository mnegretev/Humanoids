#ifndef I2C_DEVICE_HPP_
#define I2C_DEVICE_HPP_

#include <stdexcept>
#include <cstdint>
#include <string>
#include <limits>

class I2CDevice {
public:

    I2CDevice();
    ~I2CDevice();
    void openI2CBus(const std::string& bus_uri, uint8_t dev_addr);
    
    void readBytes(uint8_t reg_addr, uint8_t length, uint8_t *data);
    void readWords(uint8_t reg_addr, uint8_t length, uint16_t *data);
    
    uint8_t readByte(uint8_t reg_addr);
    uint8_t readByteBit(uint8_t reg_addr, uint8_t bit_num);
    uint8_t readByteBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length);

    uint16_t readWordBit(uint8_t reg_addr, uint8_t bit_num);
    uint16_t readWord(uint8_t reg_addr);
    uint16_t readWordBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length);
    
    void writeBytes(uint8_t reg_addr, uint8_t length, const uint8_t *data);
    void writeWords(uint8_t reg_addr, uint8_t length, const uint16_t *data);
    void writeByte(uint8_t reg_addr, uint8_t data);
    void writeWord(uint8_t reg_addr, uint16_t data);
    void setByteBit(uint8_t reg_addr, uint8_t bit_num);
    void clearByteBit(uint8_t reg_addr, uint8_t bit_num);
    void setWordBit(uint8_t reg_addr, uint8_t bit_num);
    void clearWordBit(uint8_t reg_addr, uint8_t bit_num);
    void setByteBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);
    void setWordBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t data);
    void changeByteBitValue(uint8_t reg_addr, uint8_t bit_num, bool bit_value);
    void changeWordBitValue(uint8_t reg_addr, uint8_t bit_num, bool bit_value);

private:
    int i2c_bus_fd_;
    uint8_t dev_addr_;
    void throwIOSystemError(const std::string& msg);

    template <typename T>
    void checkBitNumber(uint8_t bit_number)
    {
        if (bit_number >= std::numeric_limits<T>::digits) throw std::runtime_error("Bit index invalid");
    }

    template <typename T>
    void checkBitsRange(uint8_t bit_start, uint8_t length)
    {
        if ((bit_start + length) > std::numeric_limits<T>::digits) throw std::runtime_error("Bits range invalid");
    }

    template <typename T>
    T generateBitMaskOfOnes(uint8_t bit_start, uint8_t length)
    {
        return ((1 << length) - 1) << bit_start;
    }

    template <typename T>
    T setBits(uint8_t bit_start, uint8_t length, T bits_to_be_set, T current_data)
    {
        T mask = this->generateBitMaskOfOnes<T>(bit_start, length);
        bits_to_be_set <<= bit_start;     // shift bits_value into correct position
        bits_to_be_set &= mask;           // zero all non-important bits in bits_value
        current_data &= ~(mask);          // zero all important bits in existing byte
        current_data |= bits_to_be_set;   // combine bits_value with existing byte
        return current_data;
    }
};

#endif  // I2C_DEVICE_HPP_