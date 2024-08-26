#pragma once


#include "quaternion.h"

// This should have been a class, but the elaboration order issues are daunting.

#include <string>
using std::string;

extern bool t650_valid;

extern string t650_firmware;

extern string t650_dvnvm;

void t650_reset_status();

void parse_t650(string s, double timestamp);

struct dvpdx_message {
	int time_uS;
	int delta_time_uS;

	bool angle_delta_group_valid;
	float angle_delta_roll;
	float angle_delta_pitch;
	float angle_delta_yaw;

	bool position_group_valid;
	float position_delta_x;
	float position_delta_y;
	float position_delta_z;

	int confidence;
	int mode;

	float pitch;
	float roll;
	float standoff;
};

