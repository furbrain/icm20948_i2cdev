#ifndef I2C_WRAPPER_HPP
#define I2C_WRAPPER_HPP

#include <string>
#include <cstdint>
#include "ICM_20948_C.h"

class I2CWrapper
{
public:
    I2CWrapper(int device_number, int address);
    ~I2CWrapper();

    void close();
    void write(const uint8_t* data, size_t length);
    void read(uint8_t* data, size_t length);
    bool is_open() const;
    ICM_20948_Serif_t get_serif() const; 

private:
    int device_number_;
    uint8_t address_;
    int file_descriptor_;
    bool is_open_;
};

#endif