#include "boost/lockfree/spsc_queue.hpp"
#include <iostream>
#include <thread>
#include <fstream>
#include <string>
#include "AHRS.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "motorcontrol.h"
#include "Arduino.h"
#include "Wire.h"
#include "transform.h"
#include <math.h>

//---------------------------------
float parameter[15] = {0.0f};
//--------------------------------
MPU6050 mpu;
HMC5883L hmc;
MotorControl mc;
Transform tf;
AHRS ahrs(1/256, 1.0f, 1.0f);
float error_theta = 0.0f;
float error_r = 0.0f;
float target_theta=.0f;
bool data_produce_sign = true;
Vector3 target(.0f, .0f, .0f), car(.0f, .0f, .0f), direction(.0f, .0f, .0f);
//--------------sq------------------------------
boost::lockfree::spsc_queue<SensorRaw> gyr_sq(1000);
boost::lockfree::spsc_queue<SensorRaw> mag_sq(1000);
boost::lockfree::spsc_queue<float> deltat_sq(1000);
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
    unsigned long count = 0;
    SensorRaw accRaw, gyrRaw, magRaw;
    gyr_sq.push(gyrRaw);
    mag_sq.push(magRaw);
    while(1)
    {
        if(deltaT >= sample_period)
        {
            if(deltat_sq.write_available())
                deltat_sq.push(deltaT);
            deltaT = 0.0f;
            start = chrono::steady_clock::now();
            //-----------Start Collect Data From Sensor----------------
            gyrRaw = mpu.read_gyr_raw();
            magRaw = hmc.read_magnetometer_raw();
            //-----------Start Push Raw Data To sq------------
            if(gyr_sq.write_available())
                gyr_sq.push(gyrRaw);
            if(mag_sq.write_available())
                mag_sq.push(magRaw);
            //-----------------------------------------------
            count++;
        }
        now = chrono ::steady_clock::now();
        deltaT = chrono::duration<float, milli>(now - start).count();
        if(count == sample_count)
		{
			data_produce_sign = false;
			break;
		}
		
    }
    cout << "data_produce end\n";
}
void ahrs_init()
{
	cout << "ahrs init start...\n";
	const unsigned long sample_count = 1000; //初始化采样点
	unsigned count = 0;
	SensorRaw m_r, g_r;
	Vector3 m_s, g_s;
	float deltat;
	while(!(gyr_sq.read_available() && mag_sq.read_available() && deltat_sq.read_available())); //等待第一次采样完成
	mag_sq.pop();	gyr_sq.pop();	deltat_sq.pop();
	count++;
	while(!(gyr_sq.read_available() && mag_sq.read_available() && deltat_sq.read_available())); //等待第二次采样完成
	mag_sq.pop(m_r);
	gyr_sq.pop();
	deltat_sq.pop();
	count++;
	//----------------计算初始位置坐标----------
	cout << "m_r : \t" << m_r.XAis << "\t" << m_r.YAis << "\t" << m_r.ZAis << "\n";
	m_s.x = m_r.XAis * hmc.Xs + hmc.Xb;
	m_s.y = m_r.YAis * hmc.Ys + hmc.Yb;
	m_s.z = 0.0f;
	m_s.normalize();
	//----------------------------------------
	ahrs.mag_init_coor = m_s;
	cout << "ahrs init coor:\t" << ahrs.mag_init_coor.x << "\t"
		 << ahrs.mag_init_coor.y << "\t" << ahrs.mag_init_coor.z << "\n";
	cout << "start quaternion init...\n";
	 while(1)
    {
        if(gyr_sq.read_available() && mag_sq.read_available() && deltat_sq.read_available())
        {
            gyr_sq.pop(g_r);
            mag_sq.pop(m_r);
            deltat_sq.pop(deltat);
			//--------------
            g_s = (g_r-mpu.gyr_offset) * (mpu.gyr_mScale*0.001);//unit deg/s
			//-----------指南针矫正-------------
			m_s.x = m_r.XAis * hmc.Xs + hmc.Xb;
			m_s.y = m_r.YAis * hmc.Ys + hmc.Yb;
			//----------------
			m_s.z = 0.0f;
			g_s.x = 0.0f;
			g_s.y = 0.0f;
			//--------------------
			ahrs.kp = 1.0f;
            ahrs.sample_period = deltat * 0.001f;
            //------get current rotate quaternion-------
            ahrs.update(degtorad(g_s), m_s);
			count++;
        }
		if(count == sample_count)
		{
			cout << "end quaternion end\n";
			break;
		}
			
    }
	cout << "ahrs init end\n";
}
void calculate()
{
	cout << "calculate thread start\n";
    SensorRaw g_r, m_r;
    Vector3 g_s, m_s;
	//----------------------
    float deltat=0.0f, yaw = 0.0f;
	//-----------------------
	ahrs_init(); 
	//target_theta = 30.0f;
    while(1)
    {
        if(gyr_sq.read_available() && mag_sq.read_available() && deltat_sq.read_available())
        {
            gyr_sq.pop(g_r);
            mag_sq.pop(m_r);
            deltat_sq.pop(deltat);
			//--------------
            g_s = (g_r-mpu.gyr_offset) * (mpu.gyr_mScale*0.001);//unit deg/s
			//-----------指南针矫正-------------
			m_s.x = m_r.XAis * hmc.Xs + hmc.Xb;
			m_s.y = m_r.YAis * hmc.Ys + hmc.Yb;
			//----------------
			m_s.z = 0.0f;
			g_s.x = 0.0f;
			g_s.y = 0.0f;
			//--------------------
            ahrs.sample_period = deltat * 0.001f;
            //------get current rotate quaternion-------
            ahrs.update(degtorad(g_s), m_s);
			yaw = ahrs.get_heading(); //get current yaw			

			tf.correspondence("1");
			target = Vector3(tf.goal_x, tf.goal_y, .0f);
			car = Vector3(tf.car_x, tf.car_y, .0f);

			direction = target - car;
			direction.normalize(); // 归一化
			target_theta = radtodeg(atan2(direction.x, direction.y));
			//----------更新误差角度-----------
			error_theta = target_theta + yaw;
			if (error_theta > 180)
				error_theta = error_theta - 360.0f;
			else if (error_theta < -180)
				error_theta = error_theta + 360.0f;
			//------------更新误差距离----------
			error_r = distance(target, car);//unit cm
			//----------------------------------
			//cout << "error_theta : \t" << error_theta << "\n";
        }
		if(!data_produce_sign)
			break;
    }
	cout << "calculate thread end\n";
}
//void position()
//{
//	cout << "pisotion thread start\n";
//	Vector3 target(.0f, .0f, .0f);
//	Vector3 car(.0f, .0f, .0f);
//	Vector3 shadow(.0f, .0f, .0f);
//	float target_r, shadow_r;
//	while(true)
//	{
//		tf.correspondence("1");
//		cout << "receive over!!\tcar_x" << tf.car_x << "car_y:\t" << tf.car_y << "\n";
//		target = Vector3(tf.goal_x, tf.goal_y, 0.0f); //获得目标向量
//		car = Vector3(tf.car_x, tf.car_y, 0.0f); //获取小车向量
//		//----调试，将目标向量固定------------
//		target.x = 210.0f; //unit cm
//		target.y = 210.0f; //unit cm
//		//--------------------------------------
//		target_r = vectorMag(target);
//		target_theta = radtodeg(PI/2 - atan2(target.y, target.x)); //目标向量极坐标
//		shadow = target * (pointProduct(target, car) / (target_r*target_r)); //投影坐标
//		shadow_r = vectorMag(shadow);
//		//------更新误差距离---------------
//		error_r = target_r - shadow_r;
//		//cout << "error_r:\t" << error_r << "cm" << endl;
//		if(!data_produce_sign)
//			break;
//	}
//	cout << "position thread end\n";
//}
//静态目标控制函数
void control()
{
	cout << "static target control thread start\n";
	float car_x=.0f, car_y=.0f;
	while (1)
	{
		if (car_x != tf.car_x && car_y != tf.car_y)
		{
			if (error_theta <= 3 && error_theta >= -3)
			{
				if (!mc.mid_symbol)
				{
					mc.turn_mid();
					delay(1);
				}
				if (!mc.stop_symbol)
				{
					mc.motorGoStop();
					delay(1);
				}
				if (!mc.low_symbol)
				{
					mc.adjust_speed(122);
					delay(1);
				}
				if (error_r <= 10 && error_r >= -10)
				{
					if (!mc.stop_symbol)
					{
						mc.motorGoStop();
						delay(1);
					}
				}
				else if (error_r > 10)
				{
					if (!mc.go_symbol)
					{
						mc.motorGoForward();
						delay(1);
					}
				}
				else
				{
					if (!mc.back_symbol)
					{
						mc.motorGoBack();
						delay(1);
					}
				}
			}
			else if (error_theta > 3)
			{
				if (!mc.right_symbol)
				{
					mc.turn_right();
					delay(1);
				}
				if (!mc.high_symbol)
				{
					mc.adjust_speed(254);
					delay(1);
				}
				if (!mc.go_symbol)
				{
					mc.motorGoForward();
					delay(1);
				}
			}
			else
			{
				if (!mc.left_symbol)
				{
					mc.turn_left();
					delay(1);
				}
				if (!mc.high_symbol)
				{
					mc.adjust_speed(254);
					delay(1);
				}
				if (!mc.go_symbol)
				{
					mc.motorGoForward();
					delay(1);
				}
			}
			if (!data_produce_sign)
			{
				mc.turn_mid();
				delay(1);
				mc.motorGoStop();
				delay(1);
				break;
			}
		}else
		{
			//cout << "no position!!!\tcar_x:" << tf.car_x << "\tcar_y:" << tf.car_y << "\n";
			//cout << "no position!!\n";
			if(!mc.mid_symbol)
			{
				mc.turn_mid();
				delay(1);
			}
			if(!mc.stop_symbol)
			{
				mc.motorGoStop();
				delay(1);
			}
		}
	}
	cout << "control thread end\n";
}
void setup()
{
    mc.serialInit();
    Wire.begin();
    mpu.set_clock_source();
    mpu.set_acc_scale(int(parameter[0]));
    mpu.set_gyr_scale(int(parameter[1]));
    //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    mpu.setSampleRate(int(parameter[2]));
    hmc.set_scale(HMC5883L::M_SCALE_1P3);
    hmc.set_measurement_mode();
    mc.adjust_speed(parameter[5]);
	delay(1);
    mpu.mpu_cor_offset();
	hmc.Xs = parameter[6];
	hmc.Ys = parameter[7];
	hmc.Xb = parameter[8];
	hmc.Yb = parameter[9];
	tf.init_socket();
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
		parameter[i] = stof(sparameter);
		i++;
	}
    cout << "Acclerater Scale : " << parameter[0] << "(g)\n"
         << "Gyrscope Scale : " << parameter[1] << "(deg)\n"
         << "MPU6050 SampleRate : " << 8000/(parameter[2]+1) << "(Hz)\n"
         << "Sample Number : " << parameter[3] << "\n"
         << "Sample Frequency : " << parameter[4] << "\n"
         << "Car PWM : " << parameter[5] << "\n"
		 << "HMC Xs : " << parameter[6] << "\n"
		 << "HMC Ys : " << parameter[7] << "\n"
		 << "HMC Xb : " << parameter[8] << "\n"
		 << "HMC Yb : " << parameter[9] << "\n"
		 << "Init Sample numbers : " << parameter[10] << "\n";
	parameterFin.close();
    setup();
	thread data_produce_thread(data_producer), calculate_thread(calculate);
	thread control_thread(control);
	//thread position_thread(position);
	data_produce_thread.join();
	calculate_thread.join();
	control_thread.join();
	//position_thread.join();
    return 0;
}
