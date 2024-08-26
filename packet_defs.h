// packet_defs.h

#pragma once

enum kind_of_device
{
	unknown_kind_of_device = 0,
	echosounder = 1,								// Ping1, S500
	sidescan = 2,									// vertical fan beam - Ping360, Omniscan FS/SS
	rx_array_1d = 3,								// Insight 240
	usbl = 4,
	multimode = 5,
	multibeam_sounder = 6,						// MiniMulti

	// etc....
};

enum product_ids	// aka "device type"
{
	unknown = 0,

	// 1..99 reserved for Blue Robotics products
	Ping1 = 1,
	Ping360 = 2,									// Blue Robotics Ping360

	S100 = 100,										// Cerulean Sounder S100, 115kHz
	OmniScan450 = 101,							// Cerulean OmniScan OS450
	Surveyor24016 = 103,							// Surveyor 240-16
	SounderV2 = 112,								// Cerulean Sounder V2 @500kHz (i.e. S500)
	Insight240 = 110,								// V1: 4 x STM32F3 + STM32F7 devices
	// V2: 4 x STM32H750 + H743 Beamformer board (on Nucleo Mar 2019)
	// V3: H743 Beamformer board + 4x8 STM32G473
};

// these should be supported by all devices
enum base_ids
{
	nop = 0,										// clients may ignore, sometimes useful to keep things awake
	ack = 1,										// 2 byte payload is packet being ack'd
	nack = 2,									// 2 byte payload is packet being nack'd, optionally followed by null terminated ascii message
	ascii_text = 3,							// n byte payload always includes a terminating 0
	device_information = 4,					// upgraded by B/R. used to be: u32[3] product_id enum, vers_major, vers_minor
	protocol_version = 5,					// u32 to accommodate backward compatibility as api evolves
	general_request = 6,						// u16 id of requested item
	br_set_device_id = 100,					// BR defined this for Ping1. same as set_device_id 1000 below.
};

// not specific to any particular device type
enum general_ids
{
	Vdrain_waveform = 107,					// debugging aid
	analog_gain_levels = 108,				// u32 num_gain_levels, float analog_gain_levels[num_gain_levels]
	nmea0183_wrapper = 109,					// payload is a single NMEA 0183 sentence
	device_enum_request = 110,				// this is allowed as a udp broadcast
	device_enum_response = 111,			// response to a device enum polling datagram broadcast
	set_gate_drive_volts = 112,			// float volts
	processor_mdegC = 113,					// u32 degC * 1000;
	ui_message_to_show = 114,				// a text message intended to be shown on a UI (mainly for dev/debugggin)
	leak_detected = 115,						// no payload
	set_SOS = 116,								// u32 mm/sec speed of sound

	fw_version_check = 132,					// u8 device_type(=1), u8 device_model(=1), u16 fw_version_major, u16 fw_version_minor, u16 fw_version_patch
	fw_update_begin = 133,
	fw_update_write_block = 134,			// response is fw_update_write_ack
	fw_update_write_ack = 135,				// u32 addr, u32 count, u32 result: 0=success, 1=write failed, 2=read failed, 3 = read verify failed
	fw_update_verify_block = 136,			// response is ack or nack
	fw_update_end = 139,

	mavlink_wrapper = 150,					// packet wrapper around a mavlink payload (which is JSON)

	//		imu_info = 160,							// floats: mounting r,p,y, declination: r,p,y, u8 cal status: mag, acc, gyr, sys
	//		imu_set_mounting = 161,					// float roll, pitch, yaw degrees - relative to vehicle
	//		set_declination_deg = 162,				// float - the number to add to magnetic north to get true north
	//		transducer_deg = 163,					// float - relative to vehicle
	//		set_transducer_deg = 164,				// float roll, pitch, yaw degrees - relative to vehicle
	//		declination_deg = 165,					// float
	//		imu_mounting = 166,						// float roll, pitch, yaw degrees
	//		imu_save_cal_data = 167,				// no payload. reads current cal data and saves to nv_data
	//		imu_reset = 168,							// doesn't work on BN055 apparently but does work on xsense.

	sync_channel_number = 169,				// u8 my_channel, u8 number_of_channels
	sync_set_channel_number = 170,		// u8 my_channel, u8 number_of_channels

	//		accel_gyro_zero_cal = 171,				// command to device to begin averaging accel and gyro zero levels - no payload
	//		accel_gyro_cal_status = 172,			// command w/o payload, response u8: 0 uncalibrated/failed, 1 have valid calibration.
};

enum imu_ids
{
	// GNSS Compass, Surveyor and future general IMU work
	imu_accel_gyro_zero_cal = 171,		// command to device to begin averaging accel and gyro zero levels - no payload
	imu_accel_gyro_cal_status = 172,		// command w/o payload, response u8: 0 uncalibrated/failed, 1 have valid calibration.
	imu_data = 500,							// command no payload, response 		struct imu_data_t	{u32	time_msec;Quaternion att_quaternion;vec3	fused_up_vec;vec3	fused_mag_vec;vec3	last_accel;vec3	last_gyro;vec3	last_mag;};
	imu_heave_mpss = 501,					// command no paylaod, response float meters/sec^2
	imu_gnss_status = 502,					// command no payload, response bitmask u16:
	imu_gnss_set_offsets = 503,			// command Quaternion imu_to_system_offset (accel, gyro, mag), Quaternion gnss_to_system_offset
	imu_gnss_compass_data = 504,			// command no payload, response struct {u16 status; Quaternion orientation; Quaternion offset; i32 roll_rate; i32 pitch_rate; i32 yaw_rate; i32 lat; i32 lon; Time time;}
};

enum echosounder_ids
{
	range = 1204,								// u32 start_mm; u32 length_mm;
};

enum array_rx_ids
{
	// 3000 range for ArrayRx
	cmd_prime_for_goertzel_ping = 3001,	// cmd_prime_for_ping
	cmd_prime_for_synthesized_ping = 3002,
	cmd_prime_for_raw_data_ping = 3003,
	cmd_set_view_params = 3017,
	cmd_set_ping_params = 3020,			// ax_ping_params_t

	arx_goertzel_results = 3008,			// this is what the app sends in the ping params
	arx_goertzel_data2 = 3009,				// what gets sent is 1 of these per channel
	arx_goertzel_data_end_ping = 3010,	// and then this after all the channels have been sent

	ax_declination_servo = 3021,

	ax_raw_data3 = 3022,

	arx_beam_data3 = 3016,
	arx_set_slave_dev_index = 3018,		// 1 byte device index payload

	ax_arx_fw_update_response = 3031,	// response to same
	ax_cmd_fw_update_slaves = 3032,		// cmd from Bbd to Xbd for Xbd master to update its slaves.
	ax_slave_fw_update_response = 3033,	// response from Xbd master to Bbd results of slave f/w update

	ax_all_processors_degc = 3040,		// u32 n processors, followed by n degC values, beamformer first, then all the arx processors
	ax_all_fw_versions = 3041,				// u32 n processors, followed by the beamformer version, then all the arx processor versions (u16 major, u16 minor)
	ax_ui_state = 3042,						// Surveyor UI parameters - req - returns surveyor_ui_state_t

	// general request items
	ax_pings_per_sec = 3100,				// 1 float

};
