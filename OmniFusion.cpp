#include "OmniFusion.h"
#include "vecs.h"
#include "utilities.h"
#include <vector>

using std::to_string;
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

void OmniFusion::fuseGnss(Quaternion gnss_orientation, float gnss_latitude, float gnss_longitude, bool positionValid, float deg_per_sec, bool simData)
{
	if (simData ^ is_sim) {
		return;
	}

	m_omni_orientation = gnss_orientation;
	m_omni_rotation_rate = deg_per_sec;

	if (positionValid)
	{
		m_omni_lat = gnss_latitude;
		m_omni_lon = gnss_longitude;

		if (gnss_latitude < 44.9 || gnss_latitude > 50 || gnss_longitude < -94 || gnss_longitude > -93) {
			printf("BAD DATA\n");
		}
	}

	sendDatumToMap();
}

void OmniFusion::fuseRovl(float apparent_bearing_math, float apparent_elevation, float slant_range, bool simData)
{
	// Mismatch between simulation mode and using simulation data
	if (simData ^ is_sim) {
		return;
	}

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
	vec3 rov_in_gnss_frame = ROVL_GNSS_OFFSET.Rotate(apparent_location);

	// offset of the ROV in the world frame
	// TODO: Check that this rotation is correct and not inverted
	vec3 rov_location = m_omni_orientation.Rotate(rov_in_gnss_frame);

	// World space is ENU (x pointing east)
	m_rov_lat_meters = lat_meters(m_omni_lat) + rov_location.y;
	m_rov_lon_meters = lon_meters(m_omni_lat, m_omni_lon) + rov_location.x;

	m_rov_depth = rov_location.z;

	//double test_lat = meters_to_lat(m_lat_meters);

	////This value changes drastically if we use test_lat instead of m_omni_lat
	//double test_lon = meters_to_lon(m_omni_lat, m_lon_meters);
	//sendDatumToMap();
}

void OmniFusion::fuseRovlTrue(float true_bearing_math, float true_elevation, float slant_range)
{
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

	sendMapUpdate("SEC", "VIOLET", 0, meters_to_lat(rov_lat_meter), meters_to_lon(m_omni_lat, rov_lon_meter));
}

void OmniFusion::sendDatumToMap()
{
	//TODO: Just here for testing
	m_rov_orientation = m_omni_orientation;
	sendMapUpdate("TOPSIDE", m_omni_rotation_rate > 0.3 ? "RED" : "GREEN", m_omni_orientation.Heading(), m_omni_lat, m_omni_lon);
	sendMapUpdate("ROV", "YELLOW", m_rov_orientation.Heading(), meters_to_lat(m_rov_lat_meters), meters_to_lon(m_omni_lat, m_rov_lon_meters));
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

void OmniFusion::fuseMavlink()
{

}

void OmniFusion::fuseDvl()
{

}
