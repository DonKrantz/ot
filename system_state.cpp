#include "system_state.h"
#include "rovl.h"
#include "tracker650.h"

bool rovl_valid_rx = false;
bool configured_correctly_rx = false;
int rovl_transmit_id = -1;
string rovl_firmware_rx = "";
string firmware_tx = "";
float rovl_magnetic_declination = 0.0f;
float rovl_speed_of_sound = 0.0f;
bool rovl_using_CIMU = false;
string rovl_polling_ids_human_readable = "";
uint16_t rovl_polling_ids_mask = 0;
bool rovl_comm_active = false;

struct usrth_message rovl_usrth;

bool t650_valid = false;
string t650_dvnvm;
struct dvpdx_message t650_dvpdx;
bool t650_comm_active = false;

bool mav_global_origin_valid = false;
double mav_global_origin_lat = 0.0;
double mav_global_origin_lon = 0.0;

double mav_roll = 0.0;
double mav_pitch = 0.0;
double mav_yaw = 0.0;

double mav_x = 0.0;
double mav_y = 0.0;
double mav_z = 0.0;
double mav_vx = 0.0;
double mav_vy = 0.0;
double mav_vz = 0.0;
bool mav_comm_active = false;

int gnss_status;
Quaternion gnss_orientation;
Quaternion gnss_offset;
float gnss_roll_rate;
float gnss_pitch_rate;
float gnss_yaw_rate;
float gnss_lat;
float gnss_lon;
bool gnss_comm_active = false;

void reset_state() 
{
	rovl_valid_rx = false;
	configured_correctly_rx = false;
	rovl_transmit_id = -1;
	rovl_firmware_rx = "";
	firmware_tx = "";
	rovl_magnetic_declination = 0.0f;
	rovl_speed_of_sound = 0.0f;
	rovl_using_CIMU = false;
	rovl_polling_ids_human_readable = "";
	rovl_polling_ids_mask = 0;
	rovl_comm_active = false;

	rovl_usrth = {};

	t650_valid = false;
	t650_dvnvm = {};
	t650_dvpdx = {};
	t650_comm_active = false;

	mav_global_origin_valid = false;
	mav_global_origin_lat = 0.0;
	mav_global_origin_lon = 0.0;

	mav_roll = 0.0;
	mav_pitch = 0.0;
	mav_yaw = 0.0;

	mav_x = 0.0;
	mav_y = 0.0;
	mav_z = 0.0;
	mav_vx = 0.0;
	mav_vy = 0.0;
	mav_vz = 0.0;
	mav_comm_active = false;

	gnss_status = 0;
	gnss_orientation = {};
	gnss_offset = {};
	gnss_roll_rate = 0;
	gnss_pitch_rate = 0;
	gnss_yaw_rate = 0;
	gnss_lat = 0;
	gnss_lon = 0;
	gnss_comm_active = false;
}