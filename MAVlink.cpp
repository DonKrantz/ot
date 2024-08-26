#include "MAVlink.h"
#include "string.h"
#include "stdio.h"
#include "utilities.h"
#include "curl/curl.h"
#include <math.h>
#include <time.h>
#include <sys/sem.h>

#include <regex>

#include "tiny_json.h"


namespace {
	char error_type[132];
	char JSON_payload[1500] = "\n";
	char formerJSON[1500] = "\n";
	int sequence = 0;

	clock_t boot_time = clock();

	uint32_t GetTick()
	{
		return (uint32_t)((clock() - boot_time) / ((__clock_t)CLOCKS_PER_SEC / (__clock_t)1000));
	}

	TIMING time_last_sent = TIMENOW;

} // namespace

//==================================================================================
// Constructor
MAVlink::MAVlink(string MAVlink_address, int MAVlink_port, int vehicle_id, 
	float holdoff_seconds)
{
	if (m_MAVlink_address.substr(0, 7) == "0.0.0.0")
	{
		log_severe("MAVlink Fatal error: MAVlink disabled due to 0.0.0.0 IP address");
		strcpy(error_type, "MAVlink Fatal error: MAVlink disabled due to 0.0.0.0 IP address");
		m_connected = false;
		return;
	}

	m_MAVlink_address = MAVlink_address;
	m_MAVlink_port = MAVlink_port;
	m_vehicle_id = vehicle_id;
	m_holdoff_seconds = holdoff_seconds;
	m_handle = curl_easy_init();
	m_ghandle = curl_easy_init();
	m_start_mavlink = TIMENOW;
	m_semaphore = semget(0xABCD, 1, IPC_CREAT | 0666);
	if (m_semaphore < 0)
		log_severe("failed creating mavlink semaphore");
	if (semctl(m_semaphore, 0, SETVAL, 1))
		log_severe("failed starting semaphore");
}

//==================================================================================
// Destructor
MAVlink::~MAVlink()
{
	if (m_handle)
		curl_easy_cleanup(m_handle);

	if (m_ghandle)
		curl_easy_cleanup(m_ghandle);
}


//==================================================================================
const void MAVlink::wait_semaphore() const
{
	struct sembuf sem_op;
	sem_op.sem_num = 0;
	sem_op.sem_op = -1;
	sem_op.sem_flg = 0;
	semop(m_semaphore, &sem_op, 1);
}



//==================================================================================
const void MAVlink::give_semaphore() const
{
	struct sembuf sem_op;
	sem_op.sem_num = 0;
	sem_op.sem_op = 1;
	semop(m_semaphore, &sem_op, 1);
}


//==================================================================================
// see if we are connected. We may be too early.
const bool MAVlink::check_connect() const
{
	if (elapsed(m_start_mavlink) < m_holdoff_seconds)
		return false;

	return true;
}


//==================================================================================
string const MAVlink::get_mavlink_global_origin(string& lat, string& lon)
{
	char* jstring = get_json("GPS_GLOBAL_ORIGIN");

	if (jstring == NULL)
		return "failed";

	string js = jstring;

	lat = "";
	lon = "";

	string jn = js.substr(0, 4);

	if ( jn == "None")
		return "none";

	lat = value_of("latitude", js);
	lon = value_of("longitude", js);

	free(jstring);

	return "got lat/lon";
}


//==================================================================================
bool MAVlink::get_mavlink_attitude(string& roll, string& pitch, string& yaw)
{
	char* jstring = get_json("ATTITUDE");

	if (jstring == NULL)
		return false;

	string js = jstring;

	roll = value_of("roll", js);
	pitch = value_of("pitch", js);
	yaw = value_of("yaw", js);

	free(jstring);

	return true;
}


//==================================================================================
bool MAVlink::get_mavlink_local_position_ned(string& x, string& y, string& z, string& vx, string& vy, string& vz)
{
	char* jstring = get_json("LOCAL_POSITION_NED");

	if (jstring == NULL)
		return false;

	string js = jstring;

	x = value_of("x", js);
	y = value_of("y", js);
	z = value_of("z", js);
	vx = value_of("vx", js);
	vy = value_of("vy", js);
	vz = value_of("vz", js);

	free(jstring);

	return true;
}

//==================================================================================
namespace {
	char curl_error_string[CURL_ERROR_SIZE + 1]; 

#if false
	// CURL helper
	size_t read_function(char* bufptr, size_t size, size_t nitems, void* userp)
	{
	}
#endif

	// CURL helper
	size_t write_data_post(void* buffer, size_t size, size_t nmemb, void* userp)
	{
		static char last[132] = { 0 };

		strncpy(last, (char *)buffer, sizeof(last) - 1);

		return size * nmemb;
	}

	// CURL helper
	struct jdata
	{
		char* jstring;
	};

	// CURL helper
	size_t write_data_get(void* buffer, size_t size, size_t nmemb, void* userp)
	{
		struct jdata* jd = (struct jdata*)userp;

		jd->jstring = (char*)malloc(size * nmemb);

		if (jd->jstring != NULL)
			memcpy(jd->jstring, buffer, size * nmemb);

		return size * nmemb;
	}

} // namespace


//==================================================================================
// Core function to ask for a JSON reply from the MAVlink REST server
char* MAVlink::get_json(string messagetype)
{
	if (m_ghandle == NULL)
		return NULL;

	wait_semaphore();

	struct jdata jd = { 0 };

	string mavURL = "http://" + m_MAVlink_address + "/mavlink/vehicles/1/components/1/messages/" + messagetype;

	curl_easy_setopt(m_ghandle, CURLOPT_URL, mavURL.c_str());
	curl_easy_setopt(m_ghandle, CURLOPT_PORT, m_MAVlink_port);
	curl_easy_setopt(m_ghandle, CURLOPT_HTTPGET, 1);
	curl_easy_setopt(m_ghandle, CURLOPT_WRITEFUNCTION, write_data_get);
	curl_easy_setopt(m_ghandle, CURLOPT_WRITEDATA, &jd);
	curl_easy_setopt(m_ghandle, CURLOPT_ERRORBUFFER, curl_error_string);

	curl_easy_perform(m_ghandle);

	give_semaphore();

	return jd.jstring;
}



//==================================================================================
// Core function to POST a JSON message to the MAVlink REST server
bool MAVlink::send_json(char* json)
{
	if (m_handle == NULL)
		return false;

	wait_semaphore();

	string mavURL = "http://" + m_MAVlink_address + "/mavlink2rest/mavlink";

	struct curl_slist* slist1 = NULL;
	slist1 = curl_slist_append(slist1, "Content-Type: application/json");

//	curl_easy_setopt(m_handle, CURLOPT_VERBOSE, 1);
//	curl_easy_setopt(m_handle, CURLOPT_HEADER, 1);

	curl_easy_setopt(m_handle, CURLOPT_URL, mavURL.c_str());
	curl_easy_setopt(m_handle, CURLOPT_NOPROGRESS, 1L);
	curl_easy_setopt(m_handle, CURLOPT_POSTFIELDS, json);
	curl_easy_setopt(m_handle, CURLOPT_USERAGENT, "curl/7.38.0");
	curl_easy_setopt(m_handle, CURLOPT_HTTPHEADER, slist1);
	curl_easy_setopt(m_handle, CURLOPT_MAXREDIRS, 2L);
	curl_easy_setopt(m_handle, CURLOPT_CUSTOMREQUEST, "POST");
	curl_easy_setopt(m_handle, CURLOPT_TCP_KEEPALIVE, 1L);
	curl_easy_setopt(m_handle, CURLOPT_ERRORBUFFER, curl_error_string);
	curl_easy_setopt(m_handle, CURLOPT_WRITEFUNCTION, write_data_post);

	CURLcode ret = curl_easy_perform(m_handle);

	curl_slist_free_all(slist1);

	give_semaphore();

	return(ret == CURLE_OK);
}


//==================================================================================
// The delta position message.
bool MAVlink::send_mavlink_delta_position_data(float dx, float dy, float dz,
	float delta_t, int confidence)
{
	strcpy(formerJSON, JSON_payload); // save for debugging

	sprintf(JSON_payload,
		"{"
		"\"header\":{"
		"\"system_id\":%d,"
		"\"component_id\":0,"
		"\"sequence\":%d"
		"},"
		"\"message\":{"
		"\"type\":\"VISION_POSITION_DELTA\","
		"\"time_usec\":%ld000,"
		"\"time_delta_usec\":%d,"
		"\"angle_delta\":[0,0,0],"
		"\"position_delta\":[%f,%f,%f],"
		"\"confidence\":%d}}",
		m_vehicle_id,
		sequence++,
		(GetTick()),
		(int)(delta_t * 1000000.0f),
		dx,
		dy,
		dz,
		confidence // ((confidence > 30) ? 100 : 0)
	);

	return send_json(JSON_payload);
}

//==================================================================================
// for the MAV_DISTANCE_SENSOR_LASER message

namespace {

	const char* orthogonize(QUATERNION mounted)
	{
		VECTOR3 unit = { 0, 0, 1 };

		VECTOR3 b = mounted.Rotate(unit);

		b.x = round(b.x);
		b.y = round(b.y);
		b.z = round(b.z);

		if (b.x == 1)
			return "MAV_SENSOR_ROTATION_NONE";
		if (b.x == -1)
			return "MAV_SENSOR_ROTATION_PITCH_180";
		if (b.y == 1)
			return "MAV_SENSOR_ROTATION_YAW_90";
		if (b.y == -1)
			return "MAV_SENSOR_ROTATION_YAW_270";
		if (b.z == 1)
			return "MAV_SENSOR_ROTATION_PITCH_270";
		if (b.z == -1)
			return "MAV_SENSOR_ROTATION_PITCH_90";
		return "MAV_SENSOR_ROTATION_PITCH_270";
	}


} // namespace

//==================================================================================
bool MAVlink::send_mavlink_distance_sensor(float d, int confidence, QUATERNION quat)
{
	strcpy(formerJSON, JSON_payload); // save for debugging

	sprintf(JSON_payload,
		"{"
		"\"header\":{"
		"\"system_id\":%d,"
		"\"component_id\":0,"
		"\"sequence\":%d"
		"},"

		"\"message\": {"

		"\"covariance\": 0,"
		"\"current_distance\": %1.0f,"
		"\"horizontal_fov\": 0.0,"
		"\"id\": 0,"
		"\"mavtype\": {"
		"\"type\": \"MAV_DISTANCE_SENSOR_LASER\""
		"},"
		"\"max_distance\": 5000,"
		"\"min_distance\": 1,"
		"\"orientation\": {"
		"\"type\": \"%s\""
		"},"
		"\"quaternion\": ["
		"%f,"
		"%f,"
		"%f,"
		"%f"
		"],"
		"\"signal_quality\": %d,"
		"\"time_boot_ms\": %d,"
		"\"type\": \"DISTANCE_SENSOR\","
		"\"vertical_fov\": 0.0"
		"}}",
		m_vehicle_id,
		sequence++,
		d * 100.0f,					// current distance
		orthogonize(quat),
		0.0f, 0.0f, 0.0f, 0.0f, // quat.w, quat.x, quat.y, quat.z,
		((confidence < 20) ? 1 : confidence),
		(int)(GetTick()) // time msec
	);

	return send_json(JSON_payload);
}

//==================================================================================
bool MAVlink::send_mavlink_heartbeat()
{
	strcpy(formerJSON, JSON_payload); // save for debugging

	// only send at max 2 Hz
	if (elapsed(time_last_sent) < 1.0)
		return false;

	sprintf(JSON_payload,
		"{"
		"\"header\":{"
		"\"system_id\":%d,"
		"\"component_id\":0,"
		"\"sequence\":%d"
		"},"
		"\"message\":{"
		"\"autopilot\": {"
		"\"type\": \"MAV_AUTOPILOT_INVALID\""
		"},"
		"\"base_mode\": {"
		"\"bits\": 81"
		"},"
		"\"custom_mode\": 0," // was 19
		"\"mavlink_version\": 3,"
		"\"mavtype\": {"
		"\"type\": \"MAV_TYPE_GENERIC\""
		"},"
		"\"system_status\": {"
		"\"type\": \"MAV_STATE_ACTIVE\""
		"},"
		"\"type\": \"HEARTBEAT\""
		"}}",
		m_vehicle_id,
		sequence++);

	return send_json(JSON_payload);
}

//==================================================================================
bool MAVlink::send_mavlink_position(double latitude, double longitude)
{
	strcpy(formerJSON, JSON_payload); // save for debugging

	sprintf(JSON_payload,
		"{"
		"\"header\":{"
		"\"system_id\":%d,"
		"\"component_id\":0,"
		"\"sequence\":%d"
		"},"

		"\"message\": {"
		"\"type\":\"SET_GPS_GLOBAL_ORIGIN\","
		"\"target_system\":1,"
		"\"latitude\":%10.0f,"
		"\"longitude\":%10.0f,"
		"\"altitude\":0,"
		"\"time_usec\":%d"
		"}}",
		m_vehicle_id,
		sequence++,
		latitude * 1.0e7,
		longitude * 1.0e7,
		(int)(GetTick()) // time msec
	);

	return send_json(JSON_payload);
}


//**************************************************************************

double COE_meters = 40074000.0f; // circumference of earth in meters
double lat_m_p_d = 111318.84502145034;

//**************************************************************************
double lat_meters(double latitude)
{
	return latitude * lat_m_p_d;
}

//**************************************************************************
double lon_meters(double latitude, double longitude)
{
	return (longitude * lat_m_p_d) * cos(latitude / 180.0 * M_PI);
}



//==================================================================================
bool MAVlink::send_mavlink_position_update(double origin_lat, double origin_lon, double new_lat, double new_lon)
{
	strcpy(formerJSON, JSON_payload); // save for debugging

	double x = lat_meters(new_lat) - lat_meters(origin_lat);
	double y = lon_meters((new_lat + origin_lat) / 2.0, (new_lon - origin_lon));

	static int reset = 0;

	sprintf(JSON_payload,
		"{"
		"\"header\": {"
		"\"system_id\": %d,"  // system ID
		"\"component_id\": 0,"
		"\"sequence\": %d},"  // seq
		"\"message\": {"
		"\"type\": \"GLOBAL_VISION_POSITION_ESTIMATE\","
		"\"usec\": %d000,"  // microsenconds
		"\"x\": %lf,"
		"\"y\": %lf,"
		"\"z\": 0,"
		"\"roll\": 0,"
		"\"pitch\": 0,"
		"\"yaw\": 0,"
		"\"covariance\": ["
		"0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0"
		"],"
		"\"reset_counter\":%d}"  // reset counter
		"}",
		m_vehicle_id,
		sequence++,
		(int)(GetTick()),// time msec
		x,
		y,
		reset
	);

	return send_json(JSON_payload);
}


//==================================================================================
bool MAVlink::send_mavlink_scaled_pressure(double absolute_pressure, 
	double temperature_C, int sensor_number)
{
	strcpy(formerJSON, JSON_payload); // save for debugging

	string header = "";

	switch (sensor_number)
	{
	case 1:
		header = "SCALED_PRESSURE";
		break;

	case 2:
		header = "SCALED_PRESSURE2";
		break;

	case 3:
		header = "SCALED_PRESSURE3";
		break;

	default:
		return false;
	}

	sprintf(JSON_payload,
		"{"
		"\"header\":{"
		"\"system_id\":%d,"
		"\"component_id\":0,"
		"\"sequence\":%d"
		"},"

		"\"message\": {"
		"\"press_abs\":%10.5f,"
		"\"press_diff\":0,"
		"\"temperature\":%10.0f,"
		"\"time_boot_ms\":%d,"
		"\"type\":\"%s\""
		"}}",
		m_vehicle_id,
		sequence++,
		absolute_pressure * 1000.0,
		(temperature_C != 0) ? temperature_C * 100.0 : 1,
		(int)(GetTick()), // time msec
		header.c_str()
	);

	return send_json(JSON_payload);
}


