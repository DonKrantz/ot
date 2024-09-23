#include "system_state.h"
#include "rovl.h"
#include "tracker650.h"

OmniFusion omnifusion = OmniFusion();

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

