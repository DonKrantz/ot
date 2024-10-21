#pragma once

#include "quaternion.h"
#include "string"
#include "curl/curl.h"
#include "utilities.h"


using std::string;

//=======================================================================================
// Interface to automatically connect to a MAVlink http server and send periodic JSON-
// format update messages. Note that the MAVlink server will likely close the connection
// if left idle for more than one second, so send HEARTBEAT messages to keep alive.


class MAVlink {

private:
	string				m_MAVlink_address = "";
	int					m_MAVlink_port = 6040;
	int					m_system_id = 254;
	int					m_vehicle_id = 1;
	float					m_holdoff_seconds;	// how much time to give the MAVlink guy to boot up
	TIMING				m_start_mavlink; 
	CURL*					m_handle;   // for POST
	CURL*					m_ghandle;	// for GET
	int					m_semaphore; 

public:
	bool				m_connected = false;

	// This message used for ROV position hold function
	bool send_mavlink_delta_position_data(float dx, float dy, float dz, float delta_t, int confidence);

	// This message sends sounder data. The quaternion is the angle of the sensor
	// with respect to the ROV frame, in NED coordinates.
	bool send_mavlink_distance_sensor(float d, int confidence, Quaternion quat);

	// This message keeps the link alive when otherwise idle.
	bool send_mavlink_heartbeat();

	// Send the global initial position or a position update
	bool send_mavlink_position(double latitude, double longitude);

	// Send a position or a position update
	bool send_mavlink_position_update(double origin_lat, double origin_lon, double new_lat, double new_lon);

	// Send a pressure/temperature reading. Sensor #2 is the water pressure reading on a Blue ROV.
	bool send_mavlink_scaled_pressure(double absolute_pressure, double temperature_C, int sensor_number = 2);

	// See what the autopilot thinks is the global origin. All positions are an offset from this.
	string const get_mavlink_global_origin(string& lat, string& lon);

	// See what the autopilot thinks is the attitude of the ROV. 
	bool get_mavlink_attitude(string& roll, string& pitch, string& yaw);

	// See what the autopilot thinks is the posiotn and velocities of the ROV.
	bool get_mavlink_local_position_ned(string& x, string& y, string& z, string& vx, string& vy, string& vz);

	// Get gps raw int
	bool get_global_position_int(string& lat, string& lon);

	// Get manual control message
	bool get_manual_control(string& cmd_x, string& cmd_y, string& cmd_z, string& cmd_r);

	// Constructor
	// If IP is 0.0.0.0 the link is disabled although you can still call the message-sending
	// methods. 
	MAVlink(string MAVlink_address, int MAVlink_port, int system_id, int vehicle_id,
		float holdoff_seconds);	// delay to let the MAVlink host get booted up
	~MAVlink();

private:
	const bool check_connect() const;
	bool send_json(char* json);
	char* get_json(string messagetype, string vehicle_id = "", string component_id = "1");
	const void wait_semaphore() const;
	const void give_semaphore() const;
};

// Messages we can send back to the MAVlink server through the serializer 
enum class MAVlinkIDs { HEARTBEAT, DELTA_POSITION, DISTANCE_SENSOR, POSITION, POSITION_UPDATE, SCALED_PRESSURE };

#define PACKED_STRUCT __attribute__((__packed__))

// This message used for ROV position hold function
struct delta_position_struct {
	float dx;
	float dy;
	float dz;
	float delta_t;
	int confidence;
}; // PACKED_STRUCT;

// This message sends sounder data. The quaternion is the angle of the sensor
// with respect to the ROV frame, in NED coordinates.
struct distance_sensor_struct {
	float d;
	int confidence;
	Quaternion quat;
}; // PACKED_STRUCT;

// Send the global initial position or a position update
struct position_struct {
	double latitude;
	double longitude;
}; // PACKED_STRUCT;

// send an update to the positon in terms of global origin
struct position_update_struct {
	double origin_lat; // the ROV's global origin
	double origin_lon; // the ROV's global origin
	double new_lat; // the new postion
	double new_lon; // the new postion
}; // PACKED_STRUCT;

// Sends a pressure reading
struct scaled_pressure_struct {
	double absolute_pressure;
	double temperature_C;
	int sensor_number; // typically use 2
};// PACKED_STRUCT;

struct MAVlink_internal_message_struct {
	enum MAVlinkIDs ID;
	union {
		delta_position_struct delta_position;
		distance_sensor_struct distance_sensor;
		position_struct position;
		position_update_struct position_update;
		scaled_pressure_struct scaled_pressure;
	} payload;
};//  PACKED_STRUCT;

