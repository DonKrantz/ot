/*
 * quaternion.cpp
 *
 *  Created on: Jul 2, 2018
 *  Revised August 2022
 *      Author: Don
 *
 *  Yaw: East is 0 deg and North is 90.
 *  Heading: North is 0 deg, east is 90, south is 180 and west is 270
 */


#include "quaternion.h"
#include "math.h"
#include "string.h" // for strcmp



 // ***************************************************************************************************
 // construct a quaternion equal to 1
Quaternion::Quaternion()
	: w(1.0f), x(0.0f), y(0.0f), z(0.0f)
{
}

// derived from THREE.js library functions which set a rotation matrix from basis vectors
// then gets the quaternion from the rotation matrix
// seems to work! LL june 2024
void Quaternion::setFromBasisVectors(const vec3& x_axis, const vec3& y_axis, const vec3& z_axis)
{
	float m11 = x_axis.x;
	float m21 = x_axis.y;
	float m31 = x_axis.z;
	float m12 = y_axis.x;
	float m22 = y_axis.y;
	float m32 = y_axis.z;
	float m13 = z_axis.x;
	float m23 = z_axis.y;
	float m33 = z_axis.z;

	float trace = m11 + m22 + m33;

	if (trace > 0)
	{
		float s = 0.5f / sqrtf(trace + 1.0f);
		this->w = 0.25f / s;
		this->x = (m32 - m23) * s;
		this->y = (m13 - m31) * s;
		this->z = (m21 - m12) * s;
	}
	else if (m11 > m22 && m11 > m33) {

		float s = 2.0f * sqrtf(1.0f + m11 - m22 - m33);
		this->w = (m32 - m23) / s;
		this->x = 0.25f * s;
		this->y = (m12 + m21) / s;
		this->z = (m13 + m31) / s;
	}
	else if (m22 > m33) {
		float s = 2.0f * sqrtf(1.0f + m22 - m11 - m33);
		this->w = (m13 - m31) / s;
		this->x = (m12 + m21) / s;
		this->y = 0.25f * s;
		this->z = (m23 + m32) / s;
	}
	else {
		float s = 2.0f * sqrtf(1.0f + m33 - m11 - m22);
		this->w = (m21 - m12) / s;
		this->x = (m13 + m31) / s;
		this->y = (m23 + m32) / s;
		this->z = 0.25f * s;
	}
}

// ***************************************************************************************************
// Returns the Norm (magnitude) of the quaternion
FLOAT Quaternion::Norm() const
{
	return sqrt(w * w + x * x + y * y + z * z);
}


// ***************************************************************************************************
// Normalizes a quaternion
void Quaternion::Normalize()
{
	// set all quaternions to unit vector
	FLOAT magnitude = sqrt(w * w + x * x + y * y + z * z);

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
Quaternion::Quaternion(FLOAT roll, FLOAT pitch, FLOAT yaw, ANGLE_TYPE units)
{
	if (units == ANGLE_TYPE::DEGREES)
	{
		yaw = yaw / 180.0f * fPI;
		pitch = pitch / 180.0f * fPI;
		roll = roll / 180.0f * fPI;
	}
	FLOAT cy = cos(yaw * 0.5f);
	FLOAT sy = sin(yaw * 0.5f);
	FLOAT cr = cos(roll * 0.5f);
	FLOAT sr = sin(roll * 0.5f);
	FLOAT cp = cos(pitch * 0.5f);
	FLOAT sp = sin(pitch * 0.5f);

	w = (FLOAT)(cy * cr * cp + sy * sr * sp);
	x = (FLOAT)(cy * sr * cp - sy * cr * sp);
	y = (FLOAT)(cy * cr * sp + sy * sr * cp);
	z = (FLOAT)(sy * cr * cp - cy * sr * sp);
}

// ***************************************************************************************************
// Assigns component values to a quaternion from r-p-y in degrees or radians
void Quaternion::Assign(FLOAT roll, FLOAT pitch, FLOAT yaw, ANGLE_TYPE units)
{
	if (units == ANGLE_TYPE::DEGREES)
	{
		yaw = yaw / 180.0f * fPI;
		pitch = pitch / 180.0f * fPI;
		roll = roll / 180.0f * fPI;
	}
	FLOAT cy = cos(yaw * 0.5f);
	FLOAT sy = sin(yaw * 0.5f);
	FLOAT cr = cos(roll * 0.5f);
	FLOAT sr = sin(roll * 0.5f);
	FLOAT cp = cos(pitch * 0.5f);
	FLOAT sp = sin(pitch * 0.5f);

	w = (FLOAT)(cy * cr * cp + sy * sr * sp);
	x = (FLOAT)(cy * sr * cp - sy * cr * sp);
	y = (FLOAT)(cy * cr * sp + sy * sr * cp);
	z = (FLOAT)(sy * cr * cp - cy * sr * sp);
}

// ***************************************************************************************************
// Assigns component values to a quaternion
void Quaternion::Assign(FLOAT ww, FLOAT xx, FLOAT yy, FLOAT zz)
{
	w = ww;
	x = xx;
	y = yy;
	z = zz;
}


// ***************************************************************************************************
FLOAT Quaternion::Roll(ANGLE_TYPE units) const
{
	// from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

	FLOAT retval;

	// roll (x-axis rotation)
	FLOAT sinr = +2.0f * (w * x + y * z);
	FLOAT cosr = +1.0f - 2.0f * (x * x + y * y);
	retval = atan2(sinr, cosr);

	if (units == ANGLE_TYPE::DEGREES)
		retval = retval * 180.0f / fPI;

	return retval;
}

// ***************************************************************************************************
FLOAT Quaternion::Pitch(ANGLE_TYPE units) const
{
	// from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

	FLOAT retval;

	// pitch (y-axis rotation)
	FLOAT sinp = +2.0f * (w * y - z * x);
	if (fabs(sinp) >= 1.0f)
		retval = sinp < 0.0f ? (FLOAT)fPI / -2.0f : (FLOAT)fPI / 2.0f; // use 90 degrees if out of range
	else
		retval = asin(sinp);

	if (units == ANGLE_TYPE::DEGREES)
		retval = retval * 180.0f / fPI;

	return retval;
}

// ***************************************************************************************************
FLOAT Quaternion::Yaw(ANGLE_TYPE units) const
{
	// from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

	FLOAT retval;

	// yaw (z-axis rotation)
	FLOAT siny = +2.0f * (w * z + x * y);
	FLOAT cosy = +1.0f - 2.0f * (y * y + z * z);
	retval = atan2(siny, cosy);

	if (units == ANGLE_TYPE::DEGREES)
		retval = retval * 180.0f / fPI;

	return retval;
}

// ***************************************************************************************************
FLOAT Quaternion::Heading(ANGLE_TYPE units /*= ANGLE_TYPE::DEGREES*/)
{
	float heading = 90.0f - Yaw();
	if (heading < 0.0f)
		heading += 360.0f;

	if (units == ANGLE_TYPE::DEGREES)
		return heading;
	else
		return heading / 180.0f * fPI;
}


// ***************************************************************************************************
// Multiply Quaternions   courtesy of Erik Beall
// quaternion product, q0 = q2 x q1
Quaternion Quaternion::operator * (Quaternion qr)
{
	Quaternion q;

	// from https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
	q.w = qr.w * w - qr.x * x - qr.y * y - qr.z * z;
	q.x = qr.w * x + qr.x * w - qr.y * z + qr.z * y;
	q.y = qr.w * y + qr.x * z + qr.y * w - qr.z * x;
	q.z = qr.w * z - qr.x * y + qr.y * x + qr.z * w;
	return q;
}

// ***************************************************************************************************
// Return the conjugate of the quaternion
Quaternion Quaternion::Conjugate() const
{
	Quaternion q = *this;

	q.x = -q.x;
	q.y = -q.y;
	q.z = -q.z;

	return q;
}

void Quaternion::Invert()
{
	x = -x;
	y = -y;
	z = -z;
}


// ***************************************************************************************************
// Return the reciprocal of the quaternion
Quaternion Quaternion::Reciprocal() const
{
	Quaternion q = this->Conjugate();

	FLOAT nsq = w * w + x * x + y * y + z * z;

	q.w /= nsq;
	q.x /= nsq;
	q.y /= nsq;
	q.z /= nsq;

	return q;
}

// ***************************************************************************************************
vec3 Quaternion::Rotate(const vec3& v) const
{
	FLOAT qi = x;
	FLOAT qj = y;
	FLOAT qk = z;
	FLOAT qr = w;

	FLOAT s = std::sqrt(qi * qi + qj * qj + qk * qk + qr * qr);
	FLOAT vx = v.x;
	FLOAT vy = v.y;
	FLOAT vz = v.z;
	vec3 result
	{
		(1 - 2 * s * (qj * qj + qk * qk)) * vx + 2 * s * (qi * qj - qk * qr) * vy + 2 * s * (qi * qk + qj * qr) * vz,
		2 * s * (qi * qj + qk * qr) * vx + (1 - 2 * s * (qi * qi + qk * qk)) * vy + (2 * s * (qj * qk - qi * qr)) * vz,
		2 * s * (qi * qk - qj * qr) * vx + 2 * s * (qj * qk + qi * qr) * vy + (1 - 2 * s * (qi * qi + qj * qj)) * vz
	};
	return result;
}

void Quaternion::setFromAxisAngle(const vec3& axis, float radians)
{
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm

	// assumes axis is normalized
	const float half_angle = radians / 2.0f;
	const float s = sinf(half_angle);

	x = axis.x * s;
	y = axis.y * s;
	z = axis.z * s;
	w = cosf(half_angle);
}
