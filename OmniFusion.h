#pragma once
#include "quaternion.h"


class OmniFusion {



	// Simple state variables for the location and attitude of drone. Keeping it very simple to start
	Quaternion	m_orientation;
	double		m_lat_meters = 0;
	double		m_lon_meters = 0;
	float		m_depth = 0;



	// Last orientation and location of the Omnitrack (Office as default coordinates)
	Quaternion	m_omni_orientation = Quaternion(0,0,30);
	double		m_omni_lat = 44.969650;
	double		m_omni_lon = -93.517509;


	bool		is_sim = false;

public:

	// Call function when data arrives. Very simple to start
	void fuseGnss(Quaternion gnss_orientation, float gnss_latitude, float gnss_longitude, bool simData = false);
	// Use apparent values because we want to use GNSS for IMU/heading
	void fuseRovl(float apparent_bearing_math, float apparent_elevation, float slant_range, bool simData = false);
	void fuseMavlink();
	void fuseDvl();

	void sendDatumToMap();

	void setSim(bool sim) { is_sim = sim; }

private:
	

	//void fixLatLon();*/
};