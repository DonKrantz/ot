#include "gnss_ping_protocol.h"
#include "utilities.h"
#include "packet_defs.h"
#include "system_state.h"
#include "serializer_main.h"
#include "vecs.h"



#define PACKED_STRUCT __attribute__((__packed__))
// these ping messages are not defined in packet_def.h
#define ID_SET_PING_PARAMS    1015
#define ID_ALTITUDE           1211     // ping calls this 'distance_simple'

namespace {

	enum class PING_DEVICE_TYPE { NONE, PING1D, SOUNDER, UNKNOWN_PING_TYPE };

	uint8_t						m_buffer[13000];
	int							m_buffer_count = 0;

	// Common
	struct PROTOCOL_VERSION
	{
		uint8_t version_major;
		uint8_t version_minor;
		uint8_t version_patch;
		uint8_t reserved;
		uint16_t checksum;
	} PACKED_STRUCT;

	// Common
	struct DEVICE_INFORMATION
	{
		uint8_t device_type;
		uint8_t device_revision;
		uint8_t firmware_version_major;
		uint8_t firmware_version_minor;
		uint8_t firmware_version_patch;
		uint8_t reserved;
		uint16_t checksum;
	} PACKED_STRUCT;

	// Common
	struct ALTITUDE
	{
		uint32_t altitude_mm;
		uint8_t confidence;
		uint16_t checksum;
	} PACKED_STRUCT;

	// Common
	struct NACK
	{
		uint16_t nacked_id;
		char text[256];
		uint16_t checksum;
	} PACKED_STRUCT;

	// Common
	struct UINT8_MESSAGE
	{
		uint16_t value;
		uint16_t checksum;
	} PACKED_STRUCT;

	// Common
	struct UINT16_MESSAGE
	{
		uint16_t value;
		uint16_t checksum;
	} PACKED_STRUCT;

	// Common
	struct UINT32_MESSAGE
	{
		uint16_t value;
		uint16_t checksum;
	} PACKED_STRUCT;

	// Sounder
	struct SET_PING_PARAMS
	{
		uint32_t start_mm;
		uint32_t length_mm;
		int16_t gain_index;
		int16_t msec_per_ping;
		uint16_t ping_duration_usec;
		uint16_t report_id;
		uint16_t num_results_requested;
		uint8_t chirp;
		uint8_t decimation;
		uint16_t checksum;
	} PACKED_STRUCT;           // actually has effect, disregard compiler warning

	// Ping1
	struct SET_RANGE
	{
		uint32_t start_mm;
		uint32_t length_mm;
		uint16_t checksum;
	} PACKED_STRUCT;

	// Ping1
	struct GENERAL_INFO
	{
		uint16_t firmware_version_major;
		uint16_t firmware_version_minor;
		uint16_t voltage_5;
		uint16_t ping_interval;
		uint8_t  gain_setting;
		uint8_t mode_auto;
	} PACKED_STRUCT;

	// Ping1
	struct DISTANCE
	{
		uint32_t distance_mm;
		uint16_t confidence;
		uint16_t transmit_duration_us;
		uint32_t ping_number;
		uint32_t scan_start_mm;
		uint32_t scan_length_mm;
		uint32_t gain_Setting;
	} PACKED_STRUCT;

	// Ping1
	struct PROFILE
	{
		uint32_t distance_mm;
		uint16_t confidence;
		uint16_t transmit_duration_us;
		uint32_t ping_number;
		uint32_t scan_start_mm;
		uint32_t scan_length_mm;
		uint32_t gain_setting;
		uint16_t profile_data_length;
		uint8_t profile_data[2];
	} PACKED_STRUCT;

	// imu_gnss_compass_data
// defined in system_state.h	typedef float Quaternion[4]; // todo: placeholder for Michael's definition
	typedef char Time[8]; // todo: placeholder for Michael's definition
	//typedef float vec3[3]; // todo: placeholder for Michael's definition

	struct IMU_DATA
	{
		uint32_t	time_msec;
		Quaternion att_quaternion;
		vec3	fused_up_vec;
		vec3	fused_mag_vec;
		vec3	last_accel;
		vec3	last_gyro;
		vec3	last_mag;
	};

	struct IMU_GNSS_COMPASS_DATA
	{
		uint16_t status;
		uint16_t reserved;
		Quaternion orientation;
		Quaternion offset;
		float roll_rate;
		float pitch_rate;
		float yaw_rate;
		float lat;
		float lon;
		Time time;
	};


	// common
	typedef struct PING_MESSAGE_TYPE
	{
		uint8_t  B;
		uint8_t  R;
		uint16_t payload_length;
		uint16_t packet_id;
		uint8_t source_id;            // not implemented/ping1 seems to send '1'
		uint8_t destination_id;       // not implemented
		// end of header

		union
		{
			struct PROTOCOL_VERSION prot_vers;
			struct DEVICE_INFORMATION dev_info;
			struct ALTITUDE altitude;
			struct NACK nack;
			struct SET_PING_PARAMS ping_params;
			struct SET_RANGE set_range;
			struct UINT8_MESSAGE uint8;
			struct UINT16_MESSAGE uint16;
			struct UINT32_MESSAGE uint32;
			struct GENERAL_INFO gneral_info;
			struct DISTANCE distance;
			struct PROFILE profile;
			struct IMU_GNSS_COMPASS_DATA imu_gnss_compass_data;
			struct IMU_DATA imu_data;
		} payload;

	}  PING_MESSAGE; // PACKED_STRUCT;

#define SIZEOF_PING_HEADER 8
#define SIZEOF_CHECKSUM 2

	//==========================================================================================
	uint16_t checksum_of(void* message, int length)
	{
		uint32_t sum = 0;
		uint8_t* p = (uint8_t*)message;
		while (length--)
			sum += *(p++);

		return (uint16_t)sum;
	}

} // namespace

	//==========================================================================================
	void send_ping_request(uint16_t requested_id)
	{
		uint8_t buffer[256] = { 0 }; // so we don't try to make a whole PING message on the stack
		PING_MESSAGE* p = (PING_MESSAGE*)buffer;

		p->B = 'B';
		p->R = 'R';
		p->payload_length = 2;
		p->packet_id = general_request;
		p->source_id = 0;
		p->destination_id = 0;
		p->payload.uint16.value = requested_id;
		p->payload.uint16.checksum = checksum_of((void*)buffer, SIZEOF_PING_HEADER + sizeof(struct UINT16_MESSAGE) - SIZEOF_CHECKSUM);

		send_port_binary(PORTS::GNSS, p, SIZEOF_PING_HEADER + sizeof(struct UINT16_MESSAGE));
	}

	//==========================================================================================
	// This is a Sounder-only message type
	void set_ping_parameters(int16_t msec_per_ping)
	{
		uint8_t buffer[256] = { 0 }; // so we don't try to make a whole PING message on the stack
		PING_MESSAGE* p = (PING_MESSAGE*)buffer;

		p->B = 'B';
		p->R = 'R';
		p->payload_length = sizeof(struct SET_PING_PARAMS) - SIZEOF_CHECKSUM;
		p->packet_id = ID_SET_PING_PARAMS;
		p->source_id = 0;
		p->destination_id = 0;
		p->payload.ping_params.start_mm = 0;
		p->payload.ping_params.length_mm = 100 * 1000;
		p->payload.ping_params.gain_index = -1; // auto
		p->payload.ping_params.msec_per_ping = msec_per_ping;
		p->payload.ping_params.ping_duration_usec = 0;  // auto
		p->payload.ping_params.report_id = ID_ALTITUDE;
		p->payload.ping_params.num_results_requested = 0;
		p->payload.ping_params.chirp = 1;
		p->payload.ping_params.decimation = 0; // auto;
		p->payload.ping_params.checksum = checksum_of(buffer, SIZEOF_PING_HEADER + sizeof(struct SET_PING_PARAMS) - SIZEOF_CHECKSUM);

		send_port_binary(PORTS::GNSS, p, SIZEOF_PING_HEADER + sizeof(struct SET_PING_PARAMS));
	}

	namespace {

	//==========================================================================================
	uint16_t validate_ping_protocol_header(PING_MESSAGE* msg)
	{
		if ((msg->B != 'B') || (msg->R != 'R') /* || (msg->source_id != 0) */ || (msg->destination_id != 0))
			return 0xFFFF;

		uint16_t checksum = checksum_of(msg, msg->payload_length + SIZEOF_PING_HEADER);
		uint8_t* checksum_loc = (uint8_t*)msg + msg->payload_length + SIZEOF_PING_HEADER;
		if (checksum != *((uint16_t*)checksum_loc))
			return 0xFFFF;

		return msg->packet_id;
	}


	//==========================================================================================
	// indexes the m_buffer to the start of the next message 
	void next_message()
	{
		PING_MESSAGE* p = (PING_MESSAGE*)m_buffer;
		int msg_size = SIZEOF_PING_HEADER + p->payload_length + SIZEOF_CHECKSUM;
		if (msg_size > m_buffer_count)
			return;

		memcpy(m_buffer, m_buffer + msg_size, m_buffer_count - msg_size);
		m_buffer_count -= msg_size;
		return;
	}

	//==========================================================================================
	bool buffer_has_complete_message()
	{
		PING_MESSAGE* p = (PING_MESSAGE*)m_buffer;

		while (true)
		{
			bool invalid = false;

			if ((m_buffer_count < 10))
				return false; // not enough data to make a complete message yet

			if (p->B != 'B')
			{
				bool found = false;
				for (int i = 0; i < m_buffer_count - 4; i++)
					if ((m_buffer[i] == 'B') && (m_buffer[i + 1] == 'R') && (m_buffer[i + 3] == 0))
					{
						memcpy(m_buffer, m_buffer + i, m_buffer_count - i);
						m_buffer_count -= i;
						found = true;
					}
				if (found)
					continue;
				m_buffer_count = 0;
			}

			if ((p->B != 'B') || (p->R != 'R') || (p->destination_id != 0))
				invalid = true;

			unsigned int min_size = 0;

			min_size = SIZEOF_PING_HEADER + p->payload_length + SIZEOF_CHECKSUM;
			if (min_size > sizeof(struct PING_MESSAGE_TYPE))
				invalid = true;

			if ((!invalid) && (min_size > (unsigned)m_buffer_count))
				return false;                    // need more data

			if (invalid)  // we are lost, flush the buffer
			{
				m_buffer_count = 0;
				return false;
			}

			int rm = 0;
			if ((rm = validate_ping_protocol_header((PING_MESSAGE*)(m_buffer))) == 0xFFFF) // is it a good message or should we flush it?
			{
				next_message();
				continue;
			}

			if (rm == nack)
			{
				int msg_len = p->payload_length - SIZEOF_CHECKSUM;
				p->payload.nack.text[msg_len] = '\0';
				log_warning("Ping protocol device sent NACK of ID %d: \"%s\"", p->payload.nack.nacked_id, p->payload.nack.text);
			}

			return true;
		}
	}


	//==========================================================================================
	void process_imu_gnss_compass_data(PING_MESSAGE* p, double timestamp)
	{
		gnss_status = p->payload.imu_gnss_compass_data.status;
		gnss_orientation = p->payload.imu_gnss_compass_data.orientation;
		gnss_offset = p->payload.imu_gnss_compass_data.offset;
		float gnss_roll_rate = p->payload.imu_gnss_compass_data.roll_rate;
		float gnss_pitch_rate = p->payload.imu_gnss_compass_data.pitch_rate;
		float gnss_yaw_rate = p->payload.imu_gnss_compass_data.yaw_rate;
		float gnss_lat = p->payload.imu_gnss_compass_data.lat;
		float gnss_lon = p->payload.imu_gnss_compass_data.lon;

		char temp[400];
		snprintf(temp, sizeof(temp), "GNSS_CD, stat, %02X, Qor {, %f, %f, %f, %f, }, Qoff"
			" {, %f, %f, %f, %f, }, rr, %f, pr, %f, yr, %f, lat, %f, lon %f, r, %1.2f, p, %1.2f, y, %1.2f, h, %1.2f",
			gnss_status,
			gnss_orientation.w,
			gnss_orientation.x,
			gnss_orientation.y,
			gnss_orientation.z,
			gnss_offset.w,
			gnss_offset.x,
			gnss_offset.y,
			gnss_offset.z,
			gnss_roll_rate,
			gnss_pitch_rate,
			gnss_yaw_rate,
			gnss_lat,
			gnss_lon,
			gnss_orientation.Roll(),
			gnss_orientation.Pitch(),
			gnss_orientation.Yaw(),
			gnss_orientation.Heading()
		);

		printf("%s\n", temp);

		log_data("%s", temp);

		omnifusion.fuseGnss(gnss_orientation, gnss_lat, gnss_lon);
	}

	//==========================================================================================
	void process_imu_data(PING_MESSAGE* p)
	{
		uint32_t	time_msec = p->payload.imu_data.time_msec;
		Quaternion att_quaternion;
		att_quaternion = p->payload.imu_data.att_quaternion;
		vec3	fused_up_vec;
		fused_up_vec = p->payload.imu_data.fused_up_vec;
		vec3	fused_mag_vec;
		fused_mag_vec = p->payload.imu_data.fused_mag_vec;
		vec3	last_accel;
		last_accel = p->payload.imu_data.last_accel;
		vec3	last_gyro;
		last_gyro = p->payload.imu_data.last_gyro;
		vec3	last_mag;
		last_mag = p->payload.imu_data.last_mag;


		char temp[400];
		snprintf(temp, sizeof(temp), "GNSS ID, time, %d, att_q, {, %f, %f, %f, %f, }, fusedup, {, %f, %f, %f, }, fusedmag, {, %f, %f, %f, }, lastaccel, {, %f, %f, %f, }, lastgyro, {, %f, %f, %f, }, lastmag, {, %f, %f, %f, }",
			time_msec,
			att_quaternion.w,
			att_quaternion.x,
			att_quaternion.y,
			att_quaternion.z,
			fused_up_vec.x,
			fused_up_vec.y,
			fused_up_vec.z,
			fused_mag_vec.x,
			fused_mag_vec.y,
			fused_mag_vec.z,
			last_accel.x,
			last_accel.y,
			last_accel.z,
			last_gyro.x,
			last_gyro.y,
			last_gyro.z,
			last_mag.x,
			last_mag.y,
			last_mag.z);

		log_data("%s", temp);
	}



} // namespace


//==========================================================================================
void process_incoming_gnss(uint8_t* buffer, size_t length, double timestamp)
{
	if ((length + m_buffer_count) > (int)COUNT(m_buffer))
	{
		log_warning("ping protocol buffer overflow, restarting");
		m_buffer_count = 0;
	}

	memcpy(m_buffer + m_buffer_count, buffer, length);
	m_buffer_count += (int)length;

	while (buffer_has_complete_message())
	{
		PING_MESSAGE* p = (PING_MESSAGE*)m_buffer;

		switch (p->packet_id)
		{
		case imu_data:
			process_imu_data(p); // todo: the serializer delivers new GNSS/IMU data here. Now do something with it.
			break;

		case imu_heave_mpss: // todo: the serializer delivers new GNSS/IMU data here. Now do something with it.
			break;

		case imu_gnss_compass_data:
			process_imu_gnss_compass_data(p, timestamp); // todo: the serializer delivers new GNSS/IMU data here. Now do something with it.
			break;

		default:
			log_warning("Extraneous ping protocol gnss message, id:%d, len:%d\n", p->packet_id, p->payload_length);
			break;
		}

		next_message();
	}

}


