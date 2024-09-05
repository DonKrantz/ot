#pragma once

#include "quaternion.h"

// This should have been a class, but the elaboration order issues are daunting.

#include <string>
using std::string;

extern bool rovl_valid_rx;

extern int rovl_transmit_id;

extern string rovl_firmware_rx;

extern float rovl_magnetic_declination;

extern float rovl_speed_of_sound;

extern bool rovl_using_CIMU;

extern string rovl_polling_ids_human_readable;

extern uint16_t rovl_polling_ids_mask;

void rovl_reset_status();

void parse_rovlrx(string s, double timestamp);

struct usrth_message {

	bool ping_group_valid = false;
	float apparent_bearing_math = 0.0f;
	float apparent_bearing_compass = 0.0f;
	float apparent_elevation = 0.0f;
	float slant_range = 0.0f;
	float true_bearing_math = 0.0f;
	float true_bearing_compass = 0.0f;
	float true_elevation = 0.0f;

	bool imu_group_valid = false;
	float euler_roll = 0.0f;
	float euler_pitch = 0.0f;
	float euler_yaw = 0.0f;
	float heading = 0.0f;

	bool dummy = false;
	int adc_gain;
	string autosync_supported;
	string autosync_active;
	int seconds_since_sync;
	string imu_status;
	string channel;
	string id_decoded;
	string id_queried;
};

struct usimu_message { // also fits USIMX message
	float delta_t;
	vec3 accel;
	vec3 mag;
	vec3 gyro;
	bool is_pinging;
};






