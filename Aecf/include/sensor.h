#ifndef SENSOR_H
#define SENSOR_H

class SensorScale;

class SensorRaw
{
public:
    int XAis;
    int YAis;
    int ZAis;
    SensorRaw();
    SensorRaw(int XAis, int YAis, int ZAis);
    void operator=(const int & t);
    SensorRaw operator-(const SensorRaw & t) const;
    SensorScale operator*(const float & t) const;
    //Vector3 operator*(const Vector3& a) const;
    //SensorScale operator*(const SensorScale & t) const;
};
class SensorScale
{
public:
    float XAis;
    float YAis;
    float ZAis;
    SensorScale();
    void operator=(const int & t);
    void operator+=(const SensorRaw & t);
    SensorScale operator-(const SensorRaw & t);
    SensorRaw operator/(const int & t) const;
};
typedef SensorRaw AcceleraterRaw;
typedef SensorRaw GyroscopeRaw;
typedef SensorRaw MagnetometerRaw;

typedef SensorScale AcceleraterScale;
typedef SensorScale GyroscopeScale;
typedef SensorScale MagnetometerScale;

typedef unsigned char uint8_t;
typedef short int16_t;

#endif // SENSOR_H
