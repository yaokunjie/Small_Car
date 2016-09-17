#include <boost/lockfree/spsc_queue.hpp>
#include <iostream>
#include <thread>
#include <fstream>
#include <string>
#include "mpu6050.h"
#include "hmc5883l.h"
#include "Arduino.h"
#include "Wire.h"

//---------------------------------
int parameter[5] = {0};
//--------------------------------
MPU6050 mpu;
HMC5883L hmc;
//--------------sq------------------------------
boost::lockfree::spsc_queue<SensorRaw> gyr_sq(1000);
boost::lockfree::spsc_queue<SensorRaw> acc_sq(1000);
boost::lockfree::spsc_queue<SensorRaw> mag_sq(1000);
//--------------------------------------------------
using namespace std;



void data_producer()
{
    cout << "data_producer_thread excution\n";
    //--------------------------------------------------
    const unsigned long sample_count = parameter[3];
    const float frequency = parameter[4];
    const float sample_period = 1.0f/frequency * 1000.0f;
    float deltaT = sample_period;
    auto start = chrono::steady_clock::now();
    auto now = chrono::steady_clock::now();
    ofstream fout("./data/time_data.dat", ios_base::out | ios_base::trunc | ios_base::binary);
    //--------------------------------------------------
    unsigned long count = 0;
    //----------------------------------------------
    SensorRaw accRaw, gyrRaw, magRaw;
    //----------------------------------------------
    while(1)
    {
        if(deltaT >= sample_period)
        {
            fout.write((char*)&deltaT, sizeof(float));
            //cout << "sample_period : " << deltaT << "ms\n";
            deltaT = 0.0f;
            start = chrono::steady_clock::now();
            //-------------Start Collect Data From Sensor----------------
            //auto read_start = start;
            accRaw = mpu.read_acc_raw();
            //auto read_end = chrono::steady_clock::now();
            //cout << "read_acc : " << chrono::duration<float, milli>(read_end - read_start).count() << " ms\n";
            //read_start = chrono::steady_clock::now();
            gyrRaw = mpu.read_gyr_raw();
            //read_end = chrono::steady_clock::now();
            //cout << "read_gyr : " << chrono::duration<float, milli>(read_end - read_start).count() << " ms\n";
            //read_start = chrono::steady_clock::now();
            magRaw = hmc.read_magnetometer_raw();
            //read_end = chrono::steady_clock::now();
            //cout << "read_mag : " << chrono::duration<float, milli>(read_end - read_start).count() << " ms\n";
            //-----------Start Push Raw Data To sq------------
            if(gyr_sq.write_available())
                gyr_sq.push(gyrRaw);
            if(acc_sq.write_available())
                acc_sq.push(accRaw);
            if(mag_sq.write_available())
                mag_sq.push(magRaw);
            //-----------------------------------------------
            count++;
        }
        now = chrono ::steady_clock::now();
        deltaT = chrono::duration<float, milli>(now - start).count();
        if(count == sample_count)
            break;
    }
    fout.close();
    cout << "data_produce end\n";
}
void gyr_consumer()
{
    cout << "gyr_consumer thread excution\n";
    SensorRaw pop_valuer;
    ofstream fout("./data/gyr_data.dat", ios_base::out | ios_base::trunc | ios_base::binary);
    const unsigned long sample_count = parameter[3];
    unsigned long count = 0;
    while(1)
    {
        if(gyr_sq.read_available())
        {
            gyr_sq.pop(pop_valuer);
            fout.write((char*)&pop_valuer, sizeof(SensorRaw));
            count++;
        }
        if(count == sample_count)
            break;
    }
    fout.close();
    cout << "gyr_consumer thread end\n";
}

void acc_consumer()
{
    cout << "acc_consumer thread excution\n";
    SensorRaw pop_valuer;
    ofstream fout("./data/acc_data.dat", ios_base::out | ios_base::trunc | ios_base::binary);
    const unsigned long sample_count = parameter[3];
    unsigned long count = 0;
    while(1)
    {
        if(acc_sq.read_available())
        {
            acc_sq.pop(pop_valuer);
            fout.write((char*)&pop_valuer, sizeof(SensorRaw));
            count ++;
        }
        if(count == sample_count)
            break;
    }
    fout.close();
    cout << "acc_consumer thread end\n";
}

void mag_consumer()
{
    cout << "mag_consumer thread excution\n";
    SensorRaw pop_valuer;
    ofstream fout("./data/mag_data.dat", ios_base::out | ios_base::trunc | ios_base::binary);
    const unsigned long sample_count = parameter[3];
    unsigned long count = 0;
    while(1)
    {
        if(mag_sq.read_available())
        {
            mag_sq.pop(pop_valuer);
            fout.write((char*)&pop_valuer, sizeof(SensorRaw));
            count++;
        }
        if(count == sample_count)
            break;
    }
    fout.close();
    cout << "mag_consumer thread end\n";
}

void setup()
{
    init();
    Wire.begin();
    mpu.set_clock_source();
    mpu.set_acc_scale(parameter[0]);
    mpu.set_gyr_scale(parameter[1]);
    //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    mpu.setSampleRate(parameter[2]);
    hmc.set_scale(HMC5883L::M_SCALE_1P3);
    hmc.set_measurement_mode();
    delay(100);

}

void loop(){}
int main()
{
    int i = 0;
	fstream parameterFin("./parameter.txt");
	string sparameter;
	for (char a[16]; parameterFin.getline(&a[0], 16);)
	{
		sparameter = a;
		if (i > 5)
		{
			cout << "parameter over number\n";
			exit(0);
		}
		parameter[i] = stoi(sparameter);
		i++;
	}
	parameterFin.close();
    cout << "Acclerater Scale : " << parameter[0] << "(g)\n"
         << "Gyrscope Scale : " << parameter[1] << "(deg)\n"
         << "MPU6050 SampleRate : " << 8000/(parameter[2]+1) << "(Hz)\n"
         << "Sample Number : " << parameter[3] << "\n"
         << "Sample Frequency : " << parameter[4] << "\n";
    //------------------Sensor Init---------------
    setup();
    //----------------------------------------------------Start Open Thread---------------------------------------------
    thread data_producer_thread(data_producer), gyr_consumer_thread(gyr_consumer), acc_consumer_thread(acc_consumer), mag_consumer_thread(mag_consumer);
    data_producer_thread.join();
    gyr_consumer_thread.join();
    acc_consumer_thread.join();
    mag_consumer_thread.join();
    return 0;
}
