#include <iostream>
#include "mpu6050.h"
#include "Arduino.h"
#include "Wire.h"
using namespace std;

int main()
{
    init();
    //---------- Open Wire ---------
    Wire.begin();
    //-----------------------------
    MPU6050 mpu;
    SensorRaw accRaw, gyrRaw;
    const unsigned long sample_count = 100;
    unsigned long count = 0;
    //------ Init ---------
    mpu.set_clock_source();
    mpu.setSampleRate(7);
    mpu.set_acc_scale(2);
    mpu.set_gyr_scale(250);
    delay(100);
    while(1)
    {
        //-----------Read Data-----------
        accRaw = mpu.read_acc_raw();
        gyrRaw = mpu.read_gyr_raw();
        //--------------------------
        cout << "Acclerater : " << accRaw.XAis << "\t" << accRaw.YAis << "\t" << accRaw.ZAis << "\n";
        cout << "Gyrscope : "   << gyrRaw.XAis << "\t" << gyrRaw.YAis << "\t" << gyrRaw.ZAis << "\n";
        count ++;
        if(count == sample_count)
            break;
    }
    return 0;
}

