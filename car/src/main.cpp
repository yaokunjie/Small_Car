#include "motorcontrol.h"
#include "Arduino.h"
#include "iostream"
int main()
{
	using namespace std;
	MotorControl mc;
	mc.serialInit();
	mc.adjust_speed(254);
	delay(1);
	mc.turn_right();
	delay(1000);
	mc.motorGoForward();
	delay(2000);
	mc.motorGoStop();
	delay(1);
	mc.turn_left();
	delay(1000);
	mc.motorGoBack();
	delay(2000);
	mc.motorGoStop();
	delay(1);
	mc.turn_mid();
	delay(1000);
	cout << "Hello World\n";
	return 0;
}