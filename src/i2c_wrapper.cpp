#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <stdexcept>
#include "i2c_wrapper.hpp"
#include "ICM_20948_C.h"

ICM_20948_Status_e i2c_write(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user){
    I2CWrapper* wrapper = static_cast<I2CWrapper*>(user);
    if (len > 63) {
        return ICM_20948_Stat_ParamErr; // Too much data for our buffer
    }
    uint8_t buffer[64];
    buffer[0] = regaddr;
    std::memcpy(buffer + 1, pdata, len);
    try {
        wrapper->write(buffer, len + 1);
        return ICM_20948_Stat_Ok;
    } catch (const std::exception& e) {
        return ICM_20948_Stat_Err;
    }
}

ICM_20948_Status_e i2c_read(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user) {
    I2CWrapper* wrapper = static_cast<I2CWrapper*>(user);
    try {
        wrapper->write(&regaddr, 1); // Write register address
        wrapper->read(pdata, len);    // Read data
        return ICM_20948_Stat_Ok;
    } catch (const std::exception& e) {
        return ICM_20948_Stat_Err;
    }
}
  

I2CWrapper::~I2CWrapper() {
    close();
}

I2CWrapper::I2CWrapper(int device_number, int address) {
    device_number_ = device_number;
    is_open_ = false;
    if (is_open_) {
        throw std::runtime_error("I2C device already open");
    }

    char device[20];
    snprintf(device, sizeof(device), "/dev/i2c-%d", device_number);
    file_descriptor_ = ::open(device, O_RDWR);
    if (file_descriptor_ < 0) {
        throw std::runtime_error("Failed to open I2C device");
    }

    if (ioctl(file_descriptor_, I2C_SLAVE, address) < 0) {
        ::close(file_descriptor_);
        file_descriptor_ = -1;
        throw std::runtime_error("Failed to set I2C slave address");
    }

    address_ = address;
    is_open_ = true;
}

void I2CWrapper::write(const uint8_t* data, size_t length) {
    if (!is_open_) {
        throw std::runtime_error("I2C device not open");
    }

    if (::write(file_descriptor_, data, length) != static_cast<ssize_t>(length)) {
        throw std::runtime_error("Failed to write to I2C device");
    }
}

void I2CWrapper::read(uint8_t* data, size_t length) {
    if (!is_open_) {
        throw std::runtime_error("I2C device not open");
    }

    if (::read(file_descriptor_, data, length) != static_cast<ssize_t>(length)) {
        throw std::runtime_error("Failed to read from I2C device");
    }
}

ICM_20948_Serif_t I2CWrapper::get_serif() const {
    ICM_20948_Serif_t serif;
    serif.write = i2c_write;
    serif.read = i2c_read;
    serif.user = const_cast<I2CWrapper*>(this); // Pass the wrapper instance as user data
    return serif;
}

void I2CWrapper::close() {
    if (is_open_ && file_descriptor_ >= 0) {
        ::close(file_descriptor_);
        file_descriptor_ = -1;
        is_open_ = false;
    }
}

bool I2CWrapper::is_open() const {
    return is_open_;
}
