#include <assert.h>
#include "Quaternion.h"
#include "vector3.h"
#define _USE_MATH_DEFINES
#include <math.h>

void Quaternion::setToRotateAboutZ(float theta)
{
	float thetaOver2 = theta *.5f;
	w = cosf(thetaOver2);
	x = 0.0f;
	y = 0.0f;
	z = sinf(thetaOver2);
}
//---------------------------------------------------------------------------
Quaternion Quaternion::operator *(const Quaternion &a) const {
	Quaternion result;
	result.w = w*a.w - x*a.x - y*a.y - z*a.z;
	result.x = w*a.x + x*a.w + z*a.y - y*a.z;
	result.y = w*a.y + y*a.w + x*a.z - z*a.x;
	result.z = w*a.z + z*a.w + y*a.x - x*a.y;
	return result;
}
//---------------------------------------------------------------------------
Quaternion &Quaternion::operator *= (const Quaternion &a)
{
	*this = *this * a;
	return *this;
}

Quaternion Quaternion::operator*(const float &a) const
{
    Quaternion result;
    result.w = w * a;
    result.x = x * a;
    result.y = y * a;
    result.z = z * a;
    return result;
}

Quaternion Quaternion::operator+(const Quaternion &a) const
{
    Quaternion result;
    result.w = w + a.w;
    result.x = x + a.x;
    result.y = y + a.y;
    result.z = z + a.z;
    return result;
}
//---------------------------------------------------------------------------
void Quaternion::normalize()
{
	float mag = float(sqrt(w*w + x*x + y*y + z*z));
	if (mag > 0.0f) {
		// Normalize it
		float oneOverMag = 1.0f / mag;
		w *= oneOverMag;
		x *= oneOverMag;
		y *= oneOverMag;
		z *= oneOverMag;
	}
	else {
		// Houston, we have a problem
		assert(false);
		// In a release build, just slam it to something
		identity();
	}
}
Quaternion conjugate(const Quaternion &q) {
	Quaternion result;
	result.w = q.w;
	result.x = -q.x;
	result.y = -q.y;
	result.z = -q.z;
	return result;
}
const Quaternion kQuaternionIdentity(1.0f, 0.0f, 0.0f, 0.0f );
float dotProduct(const Quaternion &a, const Quaternion &b) {
	return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

