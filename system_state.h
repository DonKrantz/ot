#pragma once
#include "tracker650.h"
#include "rovl.h"
#include "gnss_ping_protocol.h"
#include "OmniFusion.h"


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

extern struct usrth_message rovl_usrth;
extern double usrth_timestamp;
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

extern int gnss_status;
#define gnss_heading_valid ((gnss_status & GNSS_gnss_heading_valid) != 0)
#define gnss_mag_valid ((gnss_status & GNSS_mag_valid) != 0)
#define gnss_position_valid ((gnss_status & GNSS_position_valid) != 0)
#define gnss_calibrating ((gnss_status & GNSS_calibrating) != 0)
#define gnss_calibration_valid ((gnss_status & GNSS_calibration_valid) != 0)
#define gnss_base_ok ((gnss_status & GNSS_base_ok) != 0)
#define gnss_rover_ok ((gnss_status & GNSS_rover_ok) != 0)
#define gnss_base_lock ((gnss_status & GNSS_base_lock) != 0)
#define gnss_rover_lock ((gnss_status & GNSS_rover_lock) != 0)


extern Quaternion gnss_orientation;
extern Quaternion gnss_offset;
extern float gnss_roll_rate;
extern float gnss_pitch_rate;
extern float gnss_yaw_rate;
extern float gnss_lat;
extern float gnss_lon;

extern double gnss_timestamp;
extern bool gnss_comm_active;

void reset_state();
