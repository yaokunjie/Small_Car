#ifndef _Quaternion_H
#define _Quaternion_H
class Vector3;
class Quaternion
{
public:
	float w, x, y, z;
	Quaternion() {}
	Quaternion(float qw, float qx, float qy, float qz) : w(qw), x(qx), y(qy), z(qz) {}
	void identity() { w = 1.0f; x = 0.0f; y = 0.0f; z = 0.0f; }
	void setToRotateAboutZ(float theta);
	Quaternion operator * (const Quaternion &a) const;
	Quaternion &operator *= (const Quaternion &a);
	Quaternion operator * (const float &a) const;
	Quaternion operator + (const Quaternion &a) const;
	void normalize();
};
extern const Quaternion kQuaternionIdentity;
extern float dotProduct(const Quaternion &a, const Quaternion &b);
extern Quaternion conjugate(const Quaternion &q);


#endif //Quaternion_H



