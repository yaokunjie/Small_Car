#ifndef MPU6050_H
#define MPU6050_H

#include "sensor.h"
#include "arduino_i2c.h"


class MPU6050
{
public:

        static const uint8_t DEVRICE_ADDRESS = 0x68; //MPU6050设备默认地址
        int ACC_1G;
        AcceleraterRaw acc_offset;
        GyroscopeRaw gyr_offset;
        Arduino_i2c I2C;
        float acc_mScale; //加速度输出分辨率
        float gyr_mScale; //陀螺仪输出分辨率
		MPU6050();
		AcceleraterRaw read_acc_raw();
		GyroscopeRaw read_gyr_raw();
		void set_clock_source(); //配置时钟源
		void set_acc_scale(int scale); //配置加速度量程
		void set_gyr_scale(int scale); //配置陀螺仪量程
		void set_dlpf(int band_width); //配置数字低通滤波器
		void setSampleRate(const uint8_t & rate);
		void acc_cor_offset(const int & callbrating_acc_cycles);
		void gyr_cor_offset(const int & callbrating_gyr_cycles);
		void mpu_cor_offset();


};

#endif // MPU6050_H
