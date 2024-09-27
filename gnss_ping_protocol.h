#pragma once
#include "utilities.h"
#include "packet_defs.h"

#if false // use the enums in packet_def.h
// ping protocol message IDs
#define ID_ACK                1
#define ID_NACK               2
#define ID_DEVICE_INFORMATION 4
#define ID_PROTOCOL_VERSION   5
#define ID_GENERAL_REQUEST    6

#define ID_SET_RANGE          1001
#define ID_SET_MODE_AUTO      1003
#define ID_SET_PING_INTERVAL  1004
#define ID_SET_PING_ENABLE    1006
#define ID_SET_PING_PARAMS    1015

#define ID_GENERAL_INFO       1210
#define ID_ALTITUDE           1211     // ping calls this 'distance_simple'
#define ID_DISTANCE           1212
#define ID_PING_ENABLE        1215

#define ID_PROFILE            1300

#define ID_CONTINUOUS_START   1400
#define ID_CONTINUOUS_STOP    1401
#endif




//Status bits for Gnss
#define GNSS_gnss_heading_valid (0b1<<8)
#define GNSS_mag_valid (0b1<<7)
#define GNSS_position_valid (0b1<<6)
#define GNSS_calibrating (0b1<<5)
#define GNSS_calibration_valid (0b1<<4)
#define GNSS_base_ok (0b1<<3)
#define GNSS_rover_ok (0b1<<2)
#define GNSS_base_lock (0b1<<1)
#define GNSS_rover_lock (0b1<<0)


//==========================================================================================
// Asks for a specified PING message
void send_ping_request(uint16_t requested_id);

//==========================================================================================
// This is a Sounder-only message type
void set_ping_parameters(int16_t msec_per_ping);


void process_incoming_gnss(uint8_t* buffer, size_t length, double timestamp);


