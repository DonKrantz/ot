#pragma once
#include "tracker650.h"
#include "rovl.h"


// A place to collect the current state of the system

#include <string>
using std::string;

// ROVL group
extern bool rovl_valid_rx;
extern bool configured_correctly_rx;
extern int rovl_transmit_id;
extern string rovl_firmware_rx;
extern float rovl_magnetic_declination;
extern float rovl_speed_of_sound;
extern bool rovl_using_CIMU;
extern string rovl_polling_ids_human_readable;
extern uint16_t rovl_polling_ids_mask;

extern bool rovl_comm_active;

// Tracker 650 group
extern bool t650_valid;
extern string t650_dvnvm;
extern struct dvpdx_message t650_dvpdx;
extern bool t650_comm_active;

// MAVlink group
extern bool mav_global_origin_valid;
extern double mav_global_origin_lat;
extern double mav_global_origin_lon;

extern double mav_roll;
extern double mav_pitch;
extern double mav_yaw;

extern double mav_x;
extern double mav_y;
extern double mav_z;
extern double mav_vx;
extern double mav_vy;
extern double mav_vz;

extern bool mav_comm_active;


// GNSS group
// todo: fix valid, lat, lon, time, orientation

extern bool gnss_comm_active;


