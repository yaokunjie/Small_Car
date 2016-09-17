#ifndef AHRS_H
#define AHRS_H

#include "Quaternion.h"
#include "vector3.h"

class AHRS
{
    public:
        float sample_period, kp, ki, kp_init;
        int init_period;
        Vector3 int_error;
		Vector3 mag_init_coor;
        Quaternion q;
        void update(Vector3 gyr, Vector3 mag);
        AHRS();
        AHRS(float sp, float k_p, float k_p_init);
        float get_heading();
};
extern Vector3 radtodeg(const Vector3& a);
extern Vector3 degtorad(const Vector3& a);
extern float radtodeg(const float& a);
extern float degtorad(const float& a);
#endif // ARHS_H
