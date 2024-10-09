#pragma once
#include "quaternion.h"



// World frame is ENU
// Omnitrack should be ENU
// Mavlink frame is NED
// DVL is NED
// 



/*
Notes:
- ROVL and Gnss are offset vertically. Offset depth measurement?
- Frames and transformations need to be verified
- Any use in fusing Gnss height with ROVL/DVL depth?
- 
*/
class OmniFusion {

	// Simple state variables for the location and attitude of drone. Keeping it very simple to start
	Quaternion	m_rov_orientation;
	double		m_rov_lat_meters = 0;
	double		m_rov_lon_meters = 0;

	// Potentially fuse with Omnitrack height? Not sure that adds much value
	double		m_rov_depth = 0;


	// Last orientation and location of the Omnitrack 
	Quaternion	m_omni_orientation;
	double		m_omni_lat;
	double		m_omni_lon;
	float		m_omni_rr;
	float		m_omni_pr;
	float		m_omni_yr;
	float		m_omni_rotation_rate;

	//ROVL orientation offset from GNSS
	Quaternion	m_rovl_yaw_offset;

	// Used to set ROV location initially to GNSS receiver
	bool firstPos = true;

	// Time constant for ROVL updating
	float		RC = 10;
public:
	OmniFusion();

	// Call function when data arrives. Very simple to start
	void fuseGnss(Quaternion gnss_orientation, float gnss_latitude, float gnss_longitude, bool positionValid, float deg_per_sec = 0);
	// Use apparent values because we want to use GNSS for IMU/heading
	void fuseRovl(float apparent_bearing_math, float apparent_elevation, float slant_range);


	// Just using for orientation
	void fuseMavlinkOrientation(double roll, double pitch, double yaw);

	// For comparison
	void fuseBlueBoatLocation(double lat, double lon);

	void fuseDvl(bool valid, float pos_delta_x, float pos_delta_y, float pos_delta_z);

	// For comparing with fusion of gnss and rovl. Fuses Gnss position with ROVL true data
	void sendRovlTrueToMap(float true_bearing_math, float true_elevation, float slant_range);

	//Send raw ROVL data to map to compare to fusion
	void sendRovlRawToMap(float apparent_bearing_math, float apparent_elevation, float slant_range);

	//Used to reset ROV location to GNSS receiver
	void setFirstPos() { firstPos = true; }

private:
	
	void sendDatumToMap();

	void lowPassUpdateRovPos(double lon_meters, double lat_meters, double depth, float interval_sec);
	//void fixLatLon();*/
};

extern OmniFusion omnifusion;