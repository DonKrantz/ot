#include "OmniFusion.h"
#include "vecs.h"
#include "utilities.h"
#include <vector>



//TODO: Just testing here
//OmniFusion f;
//f.fuseRovl(-90, 0, 300);
//f.sendDatumToMap();

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
} // namespace

void OmniFusion::fuseGnss(Quaternion gnss_orientation, float gnss_latitude, float gnss_longitude, bool simData)
{
	if (simData ^ is_sim) {
		return;
	}
	m_omni_orientation = gnss_orientation;
	m_omni_lat = gnss_latitude;
	m_omni_lon = gnss_longitude;

	sendDatumToMap();
}

void OmniFusion::fuseRovl(float apparent_bearing_math, float apparent_elevation, float slant_range, bool simData)
{
	if (simData ^ is_sim) {
		return;
	}
	//TODO: DELETE LINES. JUST FOR TESTING
	//slant_range = 100;
	//static int x = 0;
	//apparent_bearing_math = 360 - x;
	//x+= 10;

	//Convert angles to radians
	apparent_bearing_math *= fPI / 180;
	apparent_elevation *= fPI / 180;

	// apparent_bearing_math increases as rov moves counterclockwise relative to receiver.
	// apparent_elevation decreases as rov moves down
	float map_radius = cos(apparent_elevation) * slant_range;

	// TODO: Omnitrack and ROVL frame are treated the same but they are actually off by 90 deg.
	
	// offset of the ROV in the ROVL/Omnitrack frame in meters (Positive Z is up, Positive X is notch)
	vec3 apparent_location;
	apparent_location.x = map_radius * cos(apparent_bearing_math);
	apparent_location.y = map_radius * sin(apparent_bearing_math);
	apparent_location.z = slant_range * sin(apparent_elevation);


	// offset of the ROV in the world frame
	// TODO: Check that this rotation is correct and not inverted
	vec3 rov_location = m_omni_orientation.Rotate(apparent_location);

	// World space is ENU (x pointing east)
	m_lat_meters = lat_meters(m_omni_lat) + rov_location.y;
	m_lon_meters = lon_meters(m_omni_lat, m_omni_lon) + rov_location.x;

	m_depth = rov_location.z;

	//double test_lat = meters_to_lat(m_lat_meters);

	////This value changes drastically if we use test_lat instead of m_omni_lat
	//double test_lon = meters_to_lon(m_omni_lat, m_lon_meters);
	sendDatumToMap();
}

void OmniFusion::sendDatumToMap()
{
	//TODO: Get headings
	string tag = "TOPSIDE";
	string color = "GREEN";
	string heading = "0";
	string omni_lat = std::to_string(m_omni_lat);
	string omni_lon = std::to_string(m_omni_lon);


	string omni_loc = "MAP-UPDATE," + tag + "," + omni_lat + "," +
		omni_lon + "," + heading + "," + color;

	tag = "ROV";
	color = "YELLOW";
	heading = "0";
	string rov_lat = std::to_string(meters_to_lat(m_lat_meters));
	string rov_lon = std::to_string(meters_to_lon(m_omni_lat, m_lon_meters));

	string rov_loc = "MAP-UPDATE," + tag + "," + rov_lat + "," +
		rov_lon + "," + heading + "," + color;

	send_message_to_udp_port(-1, omni_loc, "255.255.255.255", "65001");
	send_message_to_udp_port(-1, rov_loc, "255.255.255.255", "65001");

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