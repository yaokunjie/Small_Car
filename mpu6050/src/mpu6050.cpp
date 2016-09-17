#include "mpu6050.h"

MPU6050::MPU6050()
{
//    set_clock_source();
//    set_acc_scale(2);
//    set_gyr_scale(500);
}
void MPU6050::set_clock_source()
{
    const uint8_t PWR_MGMT_1 = 0x6B; //配置复位，温度传感器，低功耗模式，时钟源寄存器地址
    I2C.i2c_write_byte(DEVRICE_ADDRESS, PWR_MGMT_1, uint8_t(0x01));;
}

//@set_dlpf
//@band_width 260Hz,184Hz,94Hz,44Hz,21Hz,10Hz,5Hz
void MPU6050::set_dlpf(int band_width)
{
    const uint8_t CONFIG = 0x1A; //配置加速度，陀螺仪低通滤波器(DLPF)寄存器地址
    uint8_t mode;
    switch (band_width)
    {
    case 260:
        mode = 0x00;
        break;
    case 184:
        mode = 0x01;
        break;
    case 94:
        mode = 0x02;
        break;
    case 44:
        mode = 0x03;
        break;
    case 21:
        mode = 0x04;
        break;
    case 10:
        mode = 0x05;
        break;
    case 5:
        mode = 0x06;
        break;
    default:
        mode = 0x07; //保留
        break;
    }
    I2C.i2c_write_byte(DEVRICE_ADDRESS, CONFIG, mode);
}
//@set_gyr_scale
//@scale 250,500,1000,2000
void MPU6050::set_gyr_scale(int scale)
{
    const uint8_t GYRO_CONFIG = 0x1B; //配置陀螺仪量程寄存器地址
    uint8_t mode;
    switch(scale)
    {
    case 250:
        mode = 0x00;
        gyr_mScale = 7.63; //±250º/s,131 LSB/(º/s)
        break;
    case 500:
        mode = 0x01;
        gyr_mScale = 15.27; //±500º/s,65.5 LSB/(º/s)
        break;
    case 1000:
        mode = 0x02;
        gyr_mScale = 30.49; //±1000º/s,32.8 LSB/(º/s)
        break;
    case 2000:
        mode = 0x03;
        gyr_mScale = 60.98; //±2000º/s,16.4 LSB/(º/s)
        break;
    default:
        mode = 0x00;
        gyr_mScale = 7.63;
        break;
    }
    mode = mode << 3;
    I2C.i2c_write_byte(DEVRICE_ADDRESS, GYRO_CONFIG, mode);
}
//@set_acc_scale
//@scale 2,4,8,16
void MPU6050::set_acc_scale(int scale)
{
    const uint8_t ACCEL_CONFIG = 0x1C; //配置加速度量程寄存器地址
    uint8_t mode;
    switch(scale)
    {
    case 2: //±2g,16384 LSB/g
        mode = 0x00;
        acc_mScale = 0.06;
        ACC_1G = 16384;
        break;
    case 4: //±4g,8192 LSB/g
        mode = 0x01;
        acc_mScale = 0.12;
        ACC_1G = 8192;
        break;
    case 8: //±8g,4096 LSB/g
        mode = 0x02;
        acc_mScale = 0.24;
        ACC_1G = 4096;
        break;
    case 16: //±16g,2048 LSB/g
        mode = 0x03;
        acc_mScale = 0.49;
        ACC_1G = 2048;
        break;
    default:
        mode = 0x00;
        acc_mScale = 0.06;
        ACC_1G = 16384;
        break;
    }
    mode = mode << 3;
    I2C.i2c_write_byte(DEVRICE_ADDRESS, ACCEL_CONFIG, mode);
}
void MPU6050::setSampleRate(const uint8_t & rate)
{
    const uint8_t SMPRT_DIV = 0x19;
    I2C.i2c_write_byte(DEVRICE_ADDRESS, SMPRT_DIV, rate);
}
void MPU6050::setFIFOEnabled(bool status)
{
    const uint8_t USER_CTRL = 0x6A;
    uint8_t mode;
    if(status)
        mode = 0x40;
    else
        mode = 0x00;
    I2C.i2c_write_byte(DEVRICE_ADDRESS, USER_CTRL, mode);
}
void MPU6050::setGyrFIFOEnabled(bool status)
{
    const uint8_t FIFO_EN = 0x23;
    uint8_t mode;
    if(status)
        mode = 0x70;
    else
        mode = 0;
    I2C.i2c_write_byte(DEVRICE_ADDRESS, FIFO_EN, mode);
}
void MPU6050::resetFIFO()
{
    const uint8_t USER_CTRL = 0x6A;
    I2C.i2c_write_byte(DEVRICE_ADDRESS, USER_CTRL, uint8_t(0x44));

}

std::vector<uint8_t> MPU6050::getFIFOBytes(uint8_t length)
{
    const uint8_t FIFO_R_W = 0x74;
    return I2C.i2c_read_bytes(DEVRICE_ADDRESS, FIFO_R_W, length);
}
GyroscopeRaw MPU6050::getFIFOGyrRaw()
{
    std::vector<uint8_t> buffer;
    GyroscopeRaw gyrRaw;
    buffer = getFIFOBytes(uint8_t(6));
    gyrRaw.XAis = int16_t(int16_t(int16_t(buffer[0]) << 8) | buffer[1]);
    gyrRaw.YAis = int16_t(int16_t(int16_t(buffer[2]) << 8) | buffer[3]);
    gyrRaw.ZAis = int16_t(int16_t(int16_t(buffer[4]) << 8) | buffer[5]);
    return gyrRaw;
}
unsigned short MPU6050::getFIFOCount()
{
    const uint8_t FIFO_COUNT_H = 0x72;
    std::vector<uint8_t> buffer;
    unsigned short count;
    buffer = I2C.i2c_read_bytes(DEVRICE_ADDRESS, FIFO_COUNT_H, (uint8_t)2);
    count = (((int16_t)buffer[0]) << 8) | buffer[1];
    return count;

}

//读取加速度原始数据
AcceleraterRaw MPU6050::read_acc_raw()
{
    const uint8_t ACCEL_XOUT_H = 0x3B; //0x3B->0x40,分别存放AXH(加速度),AXL,AYH,AYL.AZH,AZL
    std::vector<uint8_t> buffer = I2C.i2c_read_bytes(DEVRICE_ADDRESS, ACCEL_XOUT_H, uint8_t(6));
    AcceleraterRaw raw;
    raw.XAis = int16_t(int16_t(int16_t(buffer[0]) << 8) | buffer[1]);
    raw.YAis = int16_t(int16_t(int16_t(buffer[2]) << 8) | buffer[3]);
    raw.ZAis = int16_t(int16_t(int16_t(buffer[4]) << 8) | buffer[5]);
    //raw = raw - acc_offset;
    // std::cout << "raw.XAis:" << raw.XAis << std::endl
    //           << "raw.YAis:" << raw.YAis << std::endl
    //           << "raw.ZAis:" << raw.ZAis << std::endl;
    return raw;
}
//读取陀螺仪原始数据
GyroscopeRaw MPU6050::read_gyr_raw()
{
    const uint8_t GYRO_XOUT_H = 0x43; //0x43->0x48, 分别存放GXH(陀螺仪),GXL,GYH,GYL,GZH,GZL
    std::vector<uint8_t> buffer = I2C.i2c_read_bytes(DEVRICE_ADDRESS, GYRO_XOUT_H, uint8_t(6));
    GyroscopeRaw raw;
    raw.XAis = int16_t(int16_t(int16_t(buffer[0]) << 8) | buffer[1]);
    raw.YAis = int16_t(int16_t(int16_t(buffer[2]) << 8) | buffer[3]);
    raw.ZAis = int16_t(int16_t(int16_t(buffer[4]) << 8) | buffer[5]);
    //raw = raw - gyr_offset;
    return raw;
}
void MPU6050::acc_cor_offset(const int & callbrating_acc_cycles)
{
    static int cnt_a = 0;
    static AcceleraterScale temp_scale;
    AcceleraterRaw raw;
    if(cnt_a == 0)
    {
        temp_scale = 0;
        acc_offset = 0;
        cnt_a = 1;
        return;
    }
    raw = read_acc_raw();
    temp_scale += raw;
    if(cnt_a == callbrating_acc_cycles)
    {
        acc_offset = temp_scale / cnt_a;
        acc_offset.ZAis = acc_offset.ZAis - ACC_1G;
        cnt_a = 0;
        std::cout << "acc_offset.XAis:" << acc_offset.XAis << std::endl
                  << "acc_offset.YAis:" << acc_offset.YAis << std::endl
                  << "acc_offset.ZAis:" << acc_offset.ZAis << std::endl;
        return;
    }
    cnt_a ++;
}
void MPU6050::gyr_cor_offset(const int & callbrating_gyr_cycles)
{
    static int cnt_g = 0;
    static GyroscopeScale temp_scale;
    GyroscopeRaw raw;
    if(cnt_g==0)
    {
        temp_scale = 0;
        gyr_offset = 0;
        cnt_g = 1;
        return;
    }
    raw = read_gyr_raw();
    temp_scale += raw;
    if(cnt_g == callbrating_gyr_cycles)
    {
        gyr_offset = temp_scale / cnt_g;
        cnt_g = 0;
        std::cout << "gyr_offset.XAis:" << gyr_offset.XAis << std::endl
                  << "gyr_offset.YAis:" << gyr_offset.YAis << std::endl
                  << "gyr_offset.ZAis:" << gyr_offset.ZAis << std::endl;
        return;
    }
    cnt_g ++;
}
void MPU6050::mpu_cor_offset()
{
    const int cal_acc_cycles = 400, cal_gyr_cycles = 1000;

    for(int i=0;i<=cal_acc_cycles;i++)
    {
        acc_cor_offset(cal_acc_cycles);
        gyr_cor_offset(cal_gyr_cycles);
    }
    for(int n=cal_acc_cycles+1;n<=cal_gyr_cycles;n++)
    {
        gyr_cor_offset(cal_gyr_cycles);
    }
}

