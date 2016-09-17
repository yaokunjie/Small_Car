#ifndef ARDUINO_I2C_H
#define ARDUINO_I2C_H

#include <chrono>
#include <iostream>
#include <vector>
#include "Arduino.h"
#include "Wire.h"


typedef unsigned char uint8_t;
class Arduino_i2c
{
public:
    uint8_t buffer[6];

    void i2c_write_byte(uint8_t dev_address, uint8_t reg_address, uint8_t data)
    {
        Wire.beginTransmission(dev_address);
        Wire.write(reg_address);
        Wire.write(data);
        Wire.endTransmission();
    }

    void i2c_read_bytes(uint8_t dev_address, uint8_t reg_address, uint8_t length)
    {
        //auto read_start = std::chrono::steady_clock::now();
        Wire.beginTransmission(dev_address);
        Wire.write(reg_address);
        Wire.endTransmission();
        //auto read_end = std::chrono::steady_clock::now();
        //std::cout << "read_data : " << std::chrono::duration<float, std::milli>(read_end - read_start).count() << " ms\n";
        int count = 0;
        Wire.beginTransmission(dev_address);
        Wire.requestFrom(dev_address, length);
        for(;Wire.available();count++)
        {
            buffer[count] = Wire.read();
            //std::cout << int(buffer[count]) << std::endl;
        }
        Wire.endTransmission();
    }
	uint8_t i2c_read_byte(uint8_t dev_address, uint8_t reg_address)
	{
		i2c_read_bytes(dev_address, reg_address, 1);
		return buffer[0];
	}
};

#endif // ARDUINO_I2C_H
