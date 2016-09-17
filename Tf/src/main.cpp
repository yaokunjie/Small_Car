#include "transform.h"
#include "iostream"
#include "chrono"
using namespace std;
int main()
{
	Transform tf;
	int count = 0;
	auto start = chrono::steady_clock::now();
    auto now = chrono::steady_clock::now();
	float deltaT = 0.0f;
	while(1)
	{
		start = chrono::steady_clock::now();
		tf.correspondence("1");
		now = chrono ::steady_clock::now();
		deltaT = chrono::duration<float, milli>(now - start).count();
		cout << "deltaT:\t" << deltaT << "\n";
		count++;
		if(count==1000)
			break;
	}
}