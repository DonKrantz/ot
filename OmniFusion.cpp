#include "OmniFusion.h"
#include "vecs.h"
#include "utilities.h"
#include "configuration.h"
#include <cmath>

using std::to_string;


OmniFusion omnifusion;

namespace {
	//**************************************************************************

	double lat_m_p_d = 111318.84502145034; // meters per degree latitude

	//**************************************************************************

	double lat_meters(double latitude)
	{

		return latitude * lat_m_p_d;

	}

	//**************************************************************************

	// degrees longitude to meters
	// NOTE: Need to use the same value for latitude in this function and meters_to_lon
	// Or values are way off
	double lon_meters(double latitude, double longitude)
	{

		return (longitude * lat_m_p_d) * cos(latitude / 180.0 * fPI);

	}

	double meters_to_lat(double lat_meters)
	{
		return lat_meters / lat_m_p_d;
	}

	double meters_to_lon(double lat_deg, double lon_meters)
	{
		return lon_meters / (lat_m_p_d * cos(lat_deg / 180 * fPI));
	}


	void sendMapUpdate(string tag, string color, float heading, double lat, double lon)
	{
		string update = "MAP-UPDATE," + tag + "," + to_string(lat) + "," +
			to_string(lon) + "," + to_string(heading) + "," + color;

		send_message_to_udp_port(-1, update, "255.255.255.255", "65001");
	}

} // namespace

OmniFusion::OmniFusion() 
{
	m_rovl_yaw_offset = Quaternion(0, 0, std::stof(config.lookup("rovl-yaw-offset")));
}


void OmniFusion::fuseGnss(Quaternion gnss_orientation, float gnss_latitude, float gnss_longitude, bool positionValid, float deg_per_sec)
{
	m_omni_orientation = gnss_orientation;
	m_omni_rotation_rate = deg_per_sec;

	if (positionValid)
	{
		m_omni_lat = gnss_latitude;
		m_omni_lon = gnss_longitude;
			
		if (firstPos)
		{
			m_rov_lat_meters = lat_meters(m_omni_lat);
			m_rov_lon_meters = lon_meters(m_omni_lat, m_omni_lon);
			firstPos = false;
		}

		//TODO: This gets hit occasionally meaning we have a valid flag with totally invalid data. Need to look at gnss project 
		if (gnss_latitude < 44.9 || gnss_latitude > 50 || gnss_longitude < -94 || gnss_longitude > -93) {
			printf("BAD DATA\n");
		}
	}
	sendDatumToMap();
}

void OmniFusion::fuseRovl(float apparent_bearing_math, float apparent_elevation, float slant_range)
{
	//TODO: Glitch filter. Need to check bearing AFTER transformation with GNSS
	//if (apparent_elevation > 0)
	//{
	//	return;
	//}
	////Variable used to track glitches. If next bearing is off by 20 deg or more, ignore
	////TODO: The threshold of bearing could be inversely correlated with slant_range
	//static float lastBearing;

	//if (abs(apparent_bearing_math - lastBearing) > 20)
	//{
	//	//This will cause us to lose a good reading, but we may get stuck with lastBearing being a bad reading and invalidating everything otherwise
	//	lastBearing = apparent_bearing_math;
	//	return;
	//}
	//lastBearing = apparent_bearing_math;

	//Convert angles to radians
	apparent_bearing_math *= fPI / 180;
	apparent_elevation *= fPI / 180;

	// apparent_bearing_math increases as rov moves counterclockwise relative to receiver.
	// apparent_elevation decreases as rov moves down
	float map_radius = cos(apparent_elevation) * slant_range;

	// TODO: Omnitrack and ROVL frame are treated the same but they are actually off by 90 deg.
	
	// offset of the ROV in the ROVL frame in meters (Positive Z is up, Positive X is notch)
	vec3 apparent_location;
	apparent_location.x = map_radius * cos(apparent_bearing_math);
	apparent_location.y = map_radius * sin(apparent_bearing_math);
	apparent_location.z = slant_range * sin(apparent_elevation);

	// ROV in GNSS/Omnitrack frame
	vec3 rov_in_gnss_frame = m_rovl_yaw_offset.Rotate(apparent_location);

	// offset of the ROV in the world frame
	// TODO: Check that this rotation is correct and not inverted
	vec3 rov_location = m_omni_orientation.Rotate(rov_in_gnss_frame);


	// World space is ENU (x pointing east)
	/*m_rov_lat_meters = lat_meters(m_omni_lat) + rov_location.y;
	m_rov_lon_meters = lon_meters(m_omni_lat, m_omni_lon) + rov_location.x;

	m_rov_depth = rov_location.z;*/

	double rov_lon = lon_meters(m_omni_lat, m_omni_lon) + rov_location.x;
	double rov_lat = lat_meters(m_omni_lat) + rov_location.y;
	double rov_depth = rov_location.z; 

	//TODO: Testing without ROVL just using DVL.
	lowPassUpdateRovPos(rov_lon, rov_lat, rov_depth, 1);

}


void OmniFusion::lowPassUpdateRovPos(double lon_meters, double lat_meters, double depth, float interval_sec)
{
	//TODO: Modulate time constant by distance to ROVL pos and by first few data points 
	const double k = interval_sec / RC;
	m_rov_lon_meters += k * (lon_meters - m_rov_lon_meters);
	m_rov_lat_meters += k * (lat_meters - m_rov_lat_meters);
	m_rov_depth += k * (depth - m_rov_depth);
}

//NED frame
// Yaw is heading because this is NED
void OmniFusion::fuseMavlinkOrientation(double roll, double pitch, double yaw)
{
	last_mav_orientation_time = Clock().now();
	//TODO: Check mavlink frame
	m_rov_orientation = Quaternion(roll, pitch, yaw, ANGLE_TYPE::RADIANS);
	//m_rov_orientation = Quaternion(pitch, roll, fPI/2 - yaw, ANGLE_TYPE::RADIANS);
	sendDatumToMap();
}

void OmniFusion::fuseBlueBoatLocation(double lat, double lon)
{
	sendMapUpdate("boat", "VIOLET", 0, lat, lon);
}

//DVL and mavlink frames are NED
void OmniFusion::fuseDvl(bool valid, float pos_delta_x, float pos_delta_y, float pos_delta_z)
{
	//TODO: DELETE
	/*return;*/
	if (!valid || elapsed(last_mav_orientation_time) > dvl_orientation_timeout) return;

	vec3 pos_delta = vec3(pos_delta_x, pos_delta_y, pos_delta_z);
	//if (pos_delta_x != 0)
	//{
	//	printf("HERE\n");
	//}
	// Rotate position delta to NED world frame (because Rov orientation is in NED)
	// TODO: Check this rotation is correct
	pos_delta = m_rov_orientation.Rotate(pos_delta);

	// ROV frame 
	m_rov_lat_meters += pos_delta.x;
	m_rov_lon_meters += pos_delta.y;
	m_rov_depth = -pos_delta.z;
	sendDatumToMap();
}


void OmniFusion::sendDatumToMap()
{
	//sendMapUpdate("TOPSIDE", m_omni_rotation_rate > 0.3 ? "RED" : "GREEN", m_omni_orientation.Heading(), m_omni_lat, m_omni_lon);
	sendMapUpdate("TOPSIDE", "GREEN", m_omni_orientation.Heading(), m_omni_lat, m_omni_lon);
	float rov_heading = m_rov_orientation.Yaw();
	if (rov_heading < 0)
	{
		rov_heading += 360;
	}
	sendMapUpdate("ROV", "YELLOW", rov_heading, meters_to_lat(m_rov_lat_meters), meters_to_lon(m_omni_lat, m_rov_lon_meters));
}


void OmniFusion::sendRovlTrueToMap(float true_bearing_math, float true_elevation, float slant_range)
{
	printf("true bearing: %f, true elevation: %f, slant range; %f\n", true_bearing_math, true_elevation, slant_range);

	//Convert angles to radians
	true_bearing_math *= fPI / 180;
	true_elevation *= fPI / 180;

	// apparent_bearing_math increases as rov moves counterclockwise relative to receiver.
	// apparent_elevation decreases as rov moves down
	float map_radius = cos(true_elevation) * slant_range;

	// TODO: Omnitrack and ROVL frame are treated the same but they are actually off by 90 deg.

	// offset of the ROV in the ROVL frame in meters (Positive Z is up, Positive X is notch)
	vec3 true_location;
	true_location.x = map_radius * cos(true_bearing_math);
	true_location.y = map_radius * sin(true_bearing_math);
	true_location.z = slant_range * sin(true_elevation);

	// World space is ENU (x pointing east)
	double rov_lat_meter = lat_meters(m_omni_lat) + true_location.y;
	double rov_lon_meter = lon_meters(m_omni_lat, m_omni_lon) + true_location.x;

	sendMapUpdate("SEC", "GREEN", 0, meters_to_lat(rov_lat_meter), meters_to_lon(m_omni_lat, rov_lon_meter));
}

void OmniFusion::sendRovlRawToMap(float apparent_bearing_math, float apparent_elevation, float slant_range)
{
	//Convert angles to radians
	apparent_bearing_math *= fPI / 180;
	apparent_elevation *= fPI / 180;

	// apparent_bearing_math increases as rov moves counterclockwise relative to receiver.
	// apparent_elevation decreases as rov moves down
	float map_radius = cos(apparent_elevation) * slant_range;

	// TODO: Omnitrack and ROVL frame are treated the same but they are actually off by 90 deg.

	// offset of the ROV in the ROVL frame in meters (Positive Z is up, Positive X is notch)
	vec3 apparent_location;
	apparent_location.x = map_radius * cos(apparent_bearing_math);
	apparent_location.y = map_radius * sin(apparent_bearing_math);
	apparent_location.z = slant_range * sin(apparent_elevation);

	// ROV in GNSS/Omnitrack frame
	vec3 rov_in_gnss_frame = m_rovl_yaw_offset.Rotate(apparent_location);

	// offset of the ROV in the world frame
	// TODO: Check that this rotation is correct and not inverted
	vec3 rov_location = m_omni_orientation.Rotate(rov_in_gnss_frame);


	// World space is ENU (x pointing east)
	/*m_rov_lat_meters = lat_meters(m_omni_lat) + rov_location.y;
	m_rov_lon_meters = lon_meters(m_omni_lat, m_omni_lon) + rov_location.x;

	m_rov_depth = rov_location.z;*/

	double rov_lon = lon_meters(m_omni_lat, m_omni_lon) + rov_location.x;
	double rov_lat = lat_meters(m_omni_lat) + rov_location.y;
	double rov_depth = rov_location.z;

	sendMapUpdate("SEC", "RED", 0, meters_to_lat(rov_lat), meters_to_lon(m_omni_lat, rov_lon));
}


// If near poles or date line?
//void OmniFusion::fixLatLon()
//{
//	if (m_longitude > 180)
//	{
//		m_longitude -= 360;
//	}
//	else if (m_longitude < -180)
//	{
//		m_longitude += 360;
//	}
//
//	if (m_latitude > 90)
//	{
//		m_latitude = 180 - m_latitude;
//	}
//	else if (m_latitude < -90)
//	{
//		m_latitude = -180 - m_latitude;
//	}
//}