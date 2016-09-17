#include <iostream>
#include "hmc5883l.h"
#include "Arduino.h"
#include "Wire.h"

using namespace std;

int main()
{
    init();
    //------Open Wire----------
    Wire.begin();
    //------------------
    HMC5883L hmc;
    SensorRaw magRaw;
    //-----------------
    hmc.set_scale(HMC5883L::M_SCALE_1P3);
    hmc.set_measurement_mode();
    //-------------------------
    delay(100);
    const unsigned long sample_count = 100;
    unsigned long count = 0;
    while(1)
    {
        magRaw = hmc.read_magnetometer_raw();
        cout << "Mag : " << magRaw.XAis << "\t" << magRaw.YAis << "\t" << magRaw.ZAis << "\n";
        count ++;
        if(count == sample_count)
            break;
    }
}
