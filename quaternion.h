/*
 * quaternion.h
 *
 *  Created on: Jul 2, 2018
 *  Revised August 2022
 *      Author: Don
 */

#pragma once

#include <array>
#include "vecs.h"

#define FLOAT float	// there are a lot of 0.123f style constants in the code anyway


#define fPI ((float)M_PI) 

 // QUATERNION
 //
 // Identifies the orientation of a body. Better than Euler angles because there is no gimbal-lock phenomenon.
 //
 // When Euler angles are read out, they obey "math angle" conventions, not compass conventions. In other words:
 //   +X is the axis that points to the "front" of the body, e.g., the bow of a boat.
 //   +Y is the axis that points to the "left" of the body, e.g., the port side of a boat
 //   +Z is the axis the points "up", e.g., the gravity vector toward the sky.
 //
 //	 Yaw is the rotation around Z. By convention, Yaw is zero when the body points east. Right hand rule.
 //   Pitch is the rotation around Y. By convention, Pitch is zero when the X axis is perpendicular to the
 //         gravity vector pointing ahead. Right hand rule.
 //   Roll is the rotation around X. By convention, Roll is zero when the Y axis is perpendicular to the
 //         gravity vector pointing to port. Right hand rule.
 //

 // These types are used to make the constructors for roll/pitch/yaw/deg and the quaternions unambiguous. Many
 // methods default to DEGREES. DEG and RAD are shorthand notations.
enum class ANGLE_TYPE { DEGREES, RADIANS };
#define DEG ANGLE_TYPE::DEGREES
#define RAD ANGLE_TYPE::RADIANS

// Quaternion class with operators
class Quaternion
{
public:
	FLOAT w;
	FLOAT x;
	FLOAT y;
	FLOAT z;

	// construct from loose quaternion values (normalization not done)
	Quaternion(FLOAT pw, FLOAT px, FLOAT py, FLOAT pz) { w = pw; x = px; y = py; z = pz; }

	// construct from math Eulers
	Quaternion(FLOAT roll, FLOAT pitch, FLOAT yaw, ANGLE_TYPE units = ANGLE_TYPE::DEGREES);

	// Construct a quaternion with a value of "1"
	Quaternion();

	void setFromAxisAngle(const vec3& axis, float radians);
	void setFromBasisVectors(const vec3& x_axis, const vec3& y_axis, const vec3& z_axis);

	// The Euclidean Length, or Norm, of a Quaternion
	FLOAT Norm() const;

	// returns = q1 * q0
	Quaternion operator * (Quaternion q0);

	// Normalize self
	void Normalize();

	// Congugate self
	void Invert();

	// Return the conjugate of the quaternion
	Quaternion Conjugate() const;

	// Return the reciprocal of the quaternion, which is sort of the same as the conjugate.
	Quaternion Reciprocal() const;

	// Returns Euler angles in radians or degree (default) in math coordinates frame from quaternion
	FLOAT Roll(ANGLE_TYPE units = ANGLE_TYPE::DEGREES) const;
	FLOAT Pitch(ANGLE_TYPE units = ANGLE_TYPE::DEGREES) const;
	FLOAT Yaw(ANGLE_TYPE units = ANGLE_TYPE::DEGREES) const;

	// Convenience method -- returns compass heading, north = 0, left-hand rule
	FLOAT Heading(ANGLE_TYPE units = ANGLE_TYPE::DEGREES);

	void Assign(FLOAT pw, FLOAT px, FLOAT py, FLOAT pz);
	void Assign(FLOAT roll, FLOAT pitch, FLOAT yaw, ANGLE_TYPE units = ANGLE_TYPE::DEGREES);

	// Rotates v by the instance quaternion
	vec3 Rotate(const vec3& v) const;
};
