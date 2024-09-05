
// vecs.h

#pragma once
#include <math.h>


#define DLV_EFF_ZERO 1e-20
#define DLV_EFF_ZERO_SQ (DLV_EFF_ZERO*DLV_EFF_ZERO)
#define PACKED_STRUCT __attribute__((__packed__))

template <class T>
class dl3v
{
public:
	T			x;
	T			y;
	T			z;
	dl3v() {}
	dl3v(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}
	dl3v(const dl3v<T>& P) : x(P.x), y(P.y), z(P.z) {}

	void operator+=(const dl3v& v) { x += v.x; y += v.y; z += v.z; }
	void operator-=(const dl3v& v) { x -= v.x; y -= v.y; z -= v.z; }
	void operator*=(T d) { x *= d; y *= d; z *= d; }
	void operator/=(T d) { x /= d; y /= d; z /= d; }
	dl3v operator-()const { return dl3v(-x, -y, -z); }
	bool isUnitized() const { return fabs(1.0 - lengthSq()) < 10.0 * DLV_EFF_ZERO; }

	/* these are need so to do things like: (V1 - V2).lengthSq() */
	dl3v operator+(const dl3v& v)const { return dl3v(x + v.x, y + v.y, z + v.z); }
	dl3v operator-(const dl3v& v)const { return dl3v(x - v.x, y - v.y, z - v.z); }
	T lengthSq()const { return x * x + y * y + z * z; }
	T length()const { return sqrt(x * x + y * y + z * z); }
	void reverse() { x = -x; y = -y; z = -z; }
	void unitize() { T d = length(); x /= d; y /= d; z /= d; }
	T dot(const dl3v<T>& other) const { return this->x * other.x + this->y * other.y + this->z * other.z; }

	//T angleBetween(const dl3v<T> & other) {return acos(this->dot(other)/(this->length() * other.length()));}

	bool normalize()
	{
		T lensq = x * x + y * y + z * z;
		if (lensq < DLV_EFF_ZERO_SQ)
			return false;
		T l = sqrt(lensq);
		x /= l; y /= l; z /= l;
		return true;
	}
	// use: vec3 Quaternion::Rotate(const vec3 & v) const;
	//	void rotateByQuaternion(const class Quaternion & Q);
	//	{
	//		const float qx = Q.x, qy = Q.y, qz = Q.z, qw = Q.w;
	//
	//		const float ix = qw * x + qy * z - qz * y;
	//		const float iy = qw * y + qz * x - qx * z;
	//		const float iz = qw * z + qx * y - qy * x;
	//		const float iw = - qx * x - qy * y - qz * z;
	//
	//		// calculate result * inverse quat
	//		x = ix * qw + iw * - qx + iy * - qz - iz * - qy;
	//		y = iy * qw + iw * - qy + iz * - qx - ix * - qz;
	//		z = iz * qw + iw * - qz + ix * - qy - iy * - qx;
	//	}

		// friend functions
	friend int operator==(const dl3v<T>& v1, const dl3v<T>& v2) { return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z; }
	friend int operator!=(const dl3v<T>& v1, const dl3v<T>& v2) { return v1.x != v2.x || v1.y != v2.y || v1.z != v2.z; }
	friend dl3v<T> operator*(T d, const dl3v<T>& v) { return dl3v<T>(d * v.x, d * v.y, d * v.z); }
	friend dl3v<T> operator/(const dl3v<T>& v, T d) { return dl3v<T>(v.x / d, v.y / d, v.z / d); }
	friend dl3v<T> unitize(const dl3v<T>& v) { return v / v.length(); }

	/* cross product */
	friend dl3v<T> cross(const dl3v<T>& a, const dl3v<T>& b) { return a * b; }
	friend dl3v<T> operator*(const dl3v<T>& a, const dl3v<T>& b) { return dl3v<T>(a.y * b.z - a.z * b.y, -a.x * b.z + a.z * b.x, a.x * b.y - a.y * b.x); }
	friend T dot(const dl3v<T>& a, const dl3v<T>& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
} PACKED_STRUCT;

typedef dl3v<float> vec3;


