#pragma once
#include "quaternion.h"


class OmniFusion {

	// Rovl forward is -90 deg off from GNSS base-rover
	const Quaternion ROVL_GNSS_OFFSET = Quaternion(0,0,-90);

	// Simple state variables for the location and attitude of drone. Keeping it very simple to start
	Quaternion	m_rov_orientation;
	double		m_rov_lat_meters = 0;
	double		m_rov_lon_meters = 0;
	float		m_rov_depth = 0;



	// Last orientation and location of the Omnitrack 
	Quaternion	m_omni_orientation;
	double		m_omni_lat;
	double		m_omni_lon;
	float		m_omni_rr;
	float		m_omni_pr;
	float		m_omni_yr;
	float		m_omni_rotation_rate;

	bool		is_sim = false;

public:

	// Call function when data arrives. Very simple to start
	void fuseGnss(Quaternion gnss_orientation, float gnss_latitude, float gnss_longitude, bool positionValid, float deg_per_sec = 0, bool simData = false);
	// Use apparent values because we want to use GNSS for IMU/heading
	void fuseRovl(float apparent_bearing_math, float apparent_elevation, float slant_range, bool simData = false);

	// For comparing with fusion of gnss and rovl. Fuses Gnss position with ROVL true data
	void fuseRovlTrue(float true_bearing_math, float true_elevation , float slant_range);

	void fuseMavlink();
	void fuseDvl();

	void setSim(bool sim) { is_sim = sim; }
private:
	
	void sendDatumToMap();
	//void fixLatLon();*/
};