#include "gnss_ping_protocol.h"
#include "utilities.h"
#include "packet_defs.h"

//#include "serializer_main.h"



#define PACKED_STRUCT __attribute__((__packed__))


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
	typedef float Quaternion[4]; // todo: placeholder for Michael's definition
	typedef char Time[8]; // todo: placeholder for Michael's definition
	typedef float vec3[3]; // todo: placeholder for Michael's definition

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
	void process_imu_gnss_compass_data(PING_MESSAGE* p)
	{
		int status = p->payload.imu_gnss_compass_data.status;
		Quaternion orientation;
		memcpy(orientation, p->payload.imu_gnss_compass_data.orientation, sizeof(Quaternion));
		Quaternion offset;
		memcpy(offset, p->payload.imu_gnss_compass_data.offset, sizeof(Quaternion));
		float roll_rate = p->payload.imu_gnss_compass_data.roll_rate;
		float pitch_rate = p->payload.imu_gnss_compass_data.pitch_rate;
		float yaw_rate = p->payload.imu_gnss_compass_data.yaw_rate;
		float lat = p->payload.imu_gnss_compass_data.lat;
		float lon = p->payload.imu_gnss_compass_data.lon;

		char temp[400];
		snprintf(temp, sizeof(temp), "GNSS_CD, stat, %02X, Qor {, %f, %f, %f, %f, }, Qoff {, %f, %f, %f, %f, }, rr, %f, pr, %f, yr, %f, lat, %f, lon %f",
			status,
			orientation[0],
			orientation[1],
			orientation[2],
			orientation[3],
			offset[0],
			offset[1],
			offset[2],
			offset[3],
			roll_rate,
			pitch_rate,
			yaw_rate,
			lat,
			lon
		);

		log_data(":%s", temp);
	}

	//==========================================================================================
	void process_imu_data(PING_MESSAGE* p)
	{
		uint32_t	time_msec = p->payload.imu_data.time_msec;
		Quaternion att_quaternion;
		memcpy(att_quaternion, p->payload.imu_data.att_quaternion, sizeof(Quaternion));
		vec3	fused_up_vec;
		memcpy(fused_up_vec, p->payload.imu_data.fused_up_vec, sizeof(vec3));
		vec3	fused_mag_vec;
		memcpy(fused_mag_vec, p->payload.imu_data.fused_mag_vec, sizeof(vec3));
		vec3	last_accel;
		memcpy(last_accel, p->payload.imu_data.last_accel, sizeof(vec3));
		vec3	last_gyro;
		memcpy(last_gyro, p->payload.imu_data.last_gyro, sizeof(vec3));
		vec3	last_mag;
		memcpy(last_mag, p->payload.imu_data.last_mag, sizeof(vec3));


		char temp[400];
		snprintf(temp, sizeof(temp), "GNSS ID, time, %d, att_q, {, %f, %f, %f, %f, }, fusedup, {, %f, %f, %f, }, fusedmag, {, %f, %f, %f, }, lastaccel, {, %f, %f, %f, }, lastgyro, {, %f, %f, %f, }, lastmag, {, %f, %f, %f, }",
			time_msec,
			att_quaternion[0],
			att_quaternion[1],
			att_quaternion[2],
			att_quaternion[3],
			fused_up_vec[0],
			fused_up_vec[1],
			fused_up_vec[2],
			fused_mag_vec[0],
			fused_mag_vec[1],
			fused_mag_vec[2],
			last_accel[0],
			last_accel[1],
			last_accel[2],
			last_gyro[0],
			last_gyro[1],
			last_gyro[2],
			last_mag[0],
			last_mag[1],
			last_mag[2]);

		log_data(":%s", temp);
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
			process_imu_gnss_compass_data(p); // todo: the serializer delivers new GNSS/IMU data here. Now do something with it.
			break;

		default:
			log_warning("Extraneous ping protocol gnss message, id:%d, len:%d\n", p->packet_id, p->payload_length);
			break;
		}

		next_message();
	}

}


