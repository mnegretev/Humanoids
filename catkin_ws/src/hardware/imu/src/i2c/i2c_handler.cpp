#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <cerrno>
#include <cstring>
#include <sstream>

#include "i2c/i2c_handler.hpp"

I2CDevice::I2CDevice() {}

void I2CDevice::openI2CBus(const std::string& bus_uri, uint8_t dev_addr) {
  dev_addr_ = dev_addr;

  if ((i2c_bus_fd_ = open(bus_uri.c_str(), O_RDWR)) < 0)
    throwIOSystemError("Error obtaining I2C system file descriptor");

  if (ioctl(i2c_bus_fd_, I2C_SLAVE, dev_addr_) < 0) {
    std::stringstream error_msg;
    error_msg << "Failed to set I2C_SLAVE at address: 0x" << std::hex << dev_addr_;
    throwIOSystemError(error_msg.str());
  }
}

void I2CDevice::readBytes(uint8_t reg_addr, uint8_t length, uint8_t *data) {
  if (write(i2c_bus_fd_, &reg_addr, 1) < 0) throwIOSystemError("Failed to write to the I2C bus");

  int bytes_received = read(i2c_bus_fd_, data, length);
  if (bytes_received < 0 || bytes_received != length) throwIOSystemError("Failed to read from the I2C bus");
}

void I2CDevice::readWords(uint8_t reg_addr, uint8_t length, uint16_t *data) {
  if (write(i2c_bus_fd_, &reg_addr, 1) < 0) throwIOSystemError("Failed to write to the I2C bus");

  int in_buf_length = length * 2;
  uint8_t *in_buf = new uint8_t[in_buf_length];

  int bytes_received = read(i2c_bus_fd_, in_buf, in_buf_length);
  if (bytes_received < 0 || bytes_received != in_buf_length) {
    delete[] in_buf;
    throwIOSystemError("Failed to read from the I2C bus");
  }

  for (size_t i = 0; i < length; i++) data[i] = (in_buf[i*2] << 8) | in_buf[i*2+1];

  delete[] in_buf;
}

uint8_t I2CDevice::readByte(uint8_t reg_addr) {
  uint8_t byte_read;
  this->readBytes(reg_addr, 1, &byte_read);
  return byte_read;
}

uint16_t I2CDevice::readWord(uint8_t reg_addr) {
  uint16_t word_read;
  this->readWords(reg_addr, 1, &word_read);
  return word_read;
}

uint8_t I2CDevice::readByteBit(uint8_t reg_addr, uint8_t bit_num) {
  this->checkBitNumber<uint8_t>(bit_num);
  return this->readByte(reg_addr) & (1 << bit_num);
}

uint16_t I2CDevice::readWordBit(uint8_t reg_addr, uint8_t bit_num) {
  this->checkBitNumber<uint16_t>(bit_num);
  return this->readWord(reg_addr) & (1 << bit_num);
}

uint8_t I2CDevice::readByteBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length) {
  this->checkBitsRange<uint8_t>(bit_start, length);
  uint8_t mask = this->generateBitMaskOfOnes<uint8_t>(bit_start, length);
  uint8_t byte_read = this->readByte(reg_addr);
  return (byte_read & mask) >> bit_start;
}

uint16_t I2CDevice::readWordBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length) {
  this->checkBitsRange<uint16_t>(bit_start, length);
  uint16_t mask = this->generateBitMaskOfOnes<uint16_t>(bit_start, length);
  uint16_t byte_read = this->readWord(reg_addr);
  return (byte_read & mask) >> bit_start;
}


void I2CDevice::writeBytes(uint8_t reg_addr, uint8_t length, const uint8_t* data) {
  uint8_t *out_buf = new uint8_t[length + 1];
  out_buf[0] = reg_addr;
  memcpy(&out_buf[1], data, length);

  if (write(i2c_bus_fd_, out_buf, length + 1) < 0) {
    delete[] out_buf;
    throwIOSystemError("Failed to write to the I2C bus");
  }

  delete[] out_buf;
}

void I2CDevice::writeWords(uint8_t reg_addr, uint8_t length, const uint16_t* data) {
  size_t out_buf_length = 2 * length;
  uint8_t *out_buf = new uint8_t[out_buf_length];

  for (size_t i = 0; i < length; i++) {
    out_buf[i*2] = data[i] >> 8;
    out_buf[i*2+1] = data[i] >> 0;
  }

  this->writeBytes(reg_addr, out_buf_length, out_buf);

  delete[] out_buf;
}

void I2CDevice::writeByte(uint8_t reg_addr, uint8_t data) {
  this->writeBytes(reg_addr, 1, &data);
}

void I2CDevice::writeWord(uint8_t reg_addr, uint16_t data) {
  this->writeWords(reg_addr, 1, &data);
}

void I2CDevice::setByteBit(uint8_t reg_addr, uint8_t bit_num) {
  this->checkBitNumber<uint8_t>(bit_num);
  uint8_t byte_read = this->readByte(reg_addr);
  return writeByte(reg_addr, byte_read | (1 << bit_num));
}

void I2CDevice::clearByteBit(uint8_t reg_addr, uint8_t bit_num) {
  this->checkBitNumber<uint8_t>(bit_num);
  uint8_t byte_read = this->readByte(reg_addr);
  return writeByte(reg_addr, byte_read & ~(1 << bit_num));
}

void I2CDevice::setWordBit(uint8_t reg_addr, uint8_t bit_num) {
  this->checkBitNumber<uint16_t>(bit_num);
  uint16_t word_read = this->readWord(reg_addr);
  return writeWord(reg_addr, word_read | (1 << bit_num));
}

void I2CDevice::clearWordBit(uint8_t reg_addr, uint8_t bit_num) {
  this->checkBitNumber<uint16_t>(bit_num);
  uint16_t word_read = this->readByte(reg_addr);
  return writeByte(reg_addr, word_read & ~(1 << bit_num));
}

void I2CDevice::setByteBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data) {
  this->checkBitsRange<uint8_t>(bit_start, length);
  uint8_t byte_read = readByte(reg_addr);
  uint8_t new_reg_data = this->setBits<uint8_t>(bit_start, length, data, byte_read);
  writeByte(reg_addr, new_reg_data);
}

void I2CDevice::setWordBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t data) {
  this->checkBitsRange<uint16_t>(bit_start, length);
  uint16_t word_read = readWord(reg_addr);
  uint16_t new_reg_data = this->setBits<uint16_t>(bit_start, length, data, word_read);
  writeWord(reg_addr, new_reg_data);
}

void I2CDevice::throwIOSystemError(const std::string& msg) {
  std::stringstream error_msg;
  error_msg << msg << ": " << std::strerror(errno);
  throw std::runtime_error(error_msg.str());
}

void I2CDevice::changeByteBitValue(uint8_t reg_addr, uint8_t bit_num, bool bit_value) {
  if (bit_value == 1) {
    this->setByteBit(reg_addr, bit_num);
    return;
  }
  this->clearByteBit(reg_addr, bit_num);
}

void I2CDevice::changeWordBitValue(uint8_t reg_addr, uint8_t bit_num, bool bit_value) {
  if (bit_value == 1) {
    this->setWordBit(reg_addr, bit_num);
    return;
  }
  this->clearWordBit(reg_addr, bit_num);
}

I2CDevice::~I2CDevice() {
  close(i2c_bus_fd_);
}