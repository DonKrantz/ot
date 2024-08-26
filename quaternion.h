
/*
 * quaternion.h
 *
 *  Created on: Jul 2, 2018
 *      Author: Don
 */

#pragma once

#include <array>

typedef struct {
	double x;
	double y;
	double z;
} VECTOR3;

double dot(const VECTOR3& a, const VECTOR3& b);

// These types are used to make the constructors for roll/pitch/yaw/deg and the quaternions unambiguous
enum class ANGLE_TYPE { DEGREES, RADIANS };

#define DEG ANGLE_TYPE::DEGREES
#define RAD ANGLE_TYPE::RADIANS

class QUATERNION
{
public:
	double w;
	double x;
	double y;
	double z;

	// construct from loose quaternion values (normalization not done)
	QUATERNION(double pw, double px, double py, double pz) { w = pw; x = px; y = py; z = pz; }

	// construct from math Eulers
	QUATERNION(double roll, double pitch, double yaw, ANGLE_TYPE units = ANGLE_TYPE::DEGREES);

	QUATERNION();

	// The Euclidean Length, or Norm, of a Quaternion
	double Norm();

	// return = q1 * q0
	QUATERNION operator * (QUATERNION q0);

	// Normalize self
	void Normalize();

	// Return the conjugate of the quaternion
	QUATERNION Conjugate();

	// Return the reciprocal of the quaternion
	QUATERNION Reciprocal();

	// Returns Euler angles in radians in math coordinates frame from quaternion
	double Roll(ANGLE_TYPE units = ANGLE_TYPE::DEGREES);
	double Pitch(ANGLE_TYPE units = ANGLE_TYPE::DEGREES);
	double Yaw(ANGLE_TYPE units = ANGLE_TYPE::DEGREES);

	void Assign(double pw, double px, double py, double pz);
	void Assign(double roll, double pitch, double yaw, ANGLE_TYPE units = ANGLE_TYPE::DEGREES);

	// Rotates v by the instance quaternion
	VECTOR3 Rotate(const VECTOR3 v) const;
};
