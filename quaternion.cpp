/*
 * quaternion.cpp
 *
 *  Created on: Jul 2, 2018
 *      Author: Don
 */


#include "math.h"
#include "string.h" // for strcmp
#include "quaternion.h"

#define fPI 3.14159265358979f

double dot(const VECTOR3& a, const VECTOR3& b)
{
	double result = 0.0;
	result = a.x * b.x + a.y * b.y + a.z * b.z;
	return result;
};


// ***************************************************************************************************
// construct a quaternion equal to 1
QUATERNION::QUATERNION()
	: w(1.0f), x(0.0f), y(0.0f), z(0.0f)
{
}

// ***************************************************************************************************
// Returns the Norm (magnitude) of the quaternion
double QUATERNION::Norm()
{
	return sqrt(w * w + x * x + y * y + z * z);
}


// ***************************************************************************************************
// Normalizes a quaternion
void QUATERNION::Normalize()
{
	// set all quaternions to unit vector
	double magnitude = sqrt(w * w + x * x + y * y + z * z);

	if (magnitude == 0.0f)
	{
		x = y = z = 0.0f;
		w = 1.0f;
	}
	else
	{
		w = w / magnitude;
		x = x / magnitude;
		y = y / magnitude;
		z = z / magnitude;
	}
}

// ***************************************************************************************************
// Constructs a quaternion from Euler angles given in degrees or radians
QUATERNION::QUATERNION(double roll, double pitch, double yaw, ANGLE_TYPE units)
{
	if (units == ANGLE_TYPE::DEGREES)
	{
		yaw = yaw / 180.0f * fPI;
		pitch = pitch / 180.0f * fPI;
		roll = roll / 180.0f * fPI;
	}
	double cy = cos(yaw * 0.5f);
	double sy = sin(yaw * 0.5f);
	double cr = cos(roll * 0.5f);
	double sr = sin(roll * 0.5f);
	double cp = cos(pitch * 0.5f);
	double sp = sin(pitch * 0.5f);

	w = (double)(cy * cr * cp + sy * sr * sp);
	x = (double)(cy * sr * cp - sy * cr * sp);
	y = (double)(cy * cr * sp + sy * sr * cp);
	z = (double)(sy * cr * cp - cy * sr * sp);
}

// ***************************************************************************************************
// Assigns component values to a quaternion from r-p-y in degrees or radians
void QUATERNION::Assign(double roll, double pitch, double yaw, ANGLE_TYPE units)
{
	if (units == ANGLE_TYPE::DEGREES)
	{
		yaw = yaw / 180.0f * fPI;
		pitch = pitch / 180.0f * fPI;
		roll = roll / 180.0f * fPI;
	}
	double cy = cos(yaw * 0.5f);
	double sy = sin(yaw * 0.5f);
	double cr = cos(roll * 0.5f);
	double sr = sin(roll * 0.5f);
	double cp = cos(pitch * 0.5f);
	double sp = sin(pitch * 0.5f);

	w = (double)(cy * cr * cp + sy * sr * sp);
	x = (double)(cy * sr * cp - sy * cr * sp);
	y = (double)(cy * cr * sp + sy * sr * cp);
	z = (double)(sy * cr * cp - cy * sr * sp);
}

// ***************************************************************************************************
// Assigns component values to a quaternion
void QUATERNION::Assign(double ww, double xx, double yy, double zz)
{
	w = ww;
	x = xx;
	y = yy;
	z = zz;
}


// ***************************************************************************************************
double QUATERNION::Roll(ANGLE_TYPE units)
{
	// from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

	double retval;

	// roll (x-axis rotation)
	double sinr = +2.0f * (w * x + y * z);
	double cosr = +1.0f - 2.0f * (x * x + y * y);
	retval = atan2(sinr, cosr);

	if (units == ANGLE_TYPE::DEGREES)
		retval = retval * 180.0f / fPI;

	return retval;
}

// ***************************************************************************************************
double QUATERNION::Pitch(ANGLE_TYPE units)
{
	// from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

	double retval;

	// pitch (y-axis rotation)
	double sinp = +2.0f * (w * y - z * x);
	if (fabs(sinp) >= 1.0f)
		retval = sinp < 0.0f ? (double)fPI / -2.0f : (double)fPI / 2.0f; // use 90 degrees if out of range
	else
		retval = asin(sinp);

	if (units == ANGLE_TYPE::DEGREES)
		retval = retval * 180.0f / fPI;

	return retval;
}

// ***************************************************************************************************
double QUATERNION::Yaw(ANGLE_TYPE units)
{
	// from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

	double retval;

	// yaw (z-axis rotation)
	double siny = +2.0f * (w * z + x * y);
	double cosy = +1.0f - 2.0f * (y * y + z * z);
	retval = atan2(siny, cosy);

	if (units == ANGLE_TYPE::DEGREES)
		retval = retval * 180.0f / fPI;

	return retval;
}


// ***************************************************************************************************
// Multiply Quaternions   courtesy of Erik Beall
// quaternion product, q0 = q2 x q1
QUATERNION QUATERNION::operator * (QUATERNION qr)
{
	QUATERNION q;

	// from https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
	q.w = qr.w * w - qr.x * x - qr.y * y - qr.z * z;
	q.x = qr.w * x + qr.x * w - qr.y * z + qr.z * y;
	q.y = qr.w * y + qr.x * z + qr.y * w - qr.z * x;
	q.z = qr.w * z - qr.x * y + qr.y * x + qr.z * w;
	return q;
}

// ***************************************************************************************************
// Return the conjugate of the quaternion
QUATERNION QUATERNION::Conjugate()
{
	QUATERNION q = *this;

	q.x = -q.x;
	q.y = -q.y;
	q.z = -q.z;

	return q;
}

// ***************************************************************************************************
// Return the reciprocal of the quaternion
QUATERNION QUATERNION::Reciprocal()
{
	QUATERNION q = this->Conjugate();

	double nsq = w * w + x * x + y * y + z * z;

	q.w /= nsq;
	q.x /= nsq;
	q.y /= nsq;
	q.z /= nsq;

	return q;
}

VECTOR3 QUATERNION::Rotate(const VECTOR3 v) const
{
	double qi = x;
	double qj = y;
	double qk = z;
	double qr = w;

	double s = std::sqrt(qi * qi + qj * qj + qk * qk + qr * qr);
	double vx = v.x;
	double vy = v.y;
	double vz = v.z;
	VECTOR3 result
	{
		(1 - 2 * s * (qj * qj + qk * qk)) * vx + 2 * s * (qi * qj - qk * qr) * vy + 2 * s * (qi * qk + qj * qr) * vz,
		2 * s * (qi * qj + qk * qr) * vx + (1 - 2 * s * (qi * qi + qk * qk)) * vy + (2 * s * (qj * qk - qi * qr)) * vz,
		2 * s * (qi * qk - qj * qr) * vx + 2 * s * (qj * qk + qi * qr) * vy + (1 - 2 * s * (qi * qi + qj * qj)) * vz
	};
	return result;
}
