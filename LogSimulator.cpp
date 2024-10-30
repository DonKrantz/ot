#include "LogSimulator.h"
#include <fstream>
#include <iostream>
#include <string>
#include "utilities.h"
#include "quaternion.h"

#include "system_state.h"
#include "OmniFusion.h"
#include "serializer_main.h"
#include "tracker650.h"

#include "OmniUkf.h"

using namespace std;

void LogSimulator::disconnect_devices() {
	port_disconnect(PORTS::ROVL_RX);
	port_disconnect(PORTS::GNSS);
	port_disconnect(PORTS::MAVLINK_LISTENING);
	port_disconnect(PORTS::TRACKER650);
}

void LogSimulator::reconnect_devices() {
	port_reconnect(PORTS::ROVL_RX);
	port_reconnect(PORTS::GNSS);
	port_reconnect(PORTS::MAVLINK_LISTENING);
	port_reconnect(PORTS::TRACKER650);
}

void LogSimulator::runSimulation(string logPath) {
	ifstream logfile(logPath);
	string line;
	if (logfile.is_open()) 
	{
		setup();
		log_event("Beginning Log Simulation of %s. Disconnecting devices and resetting state...\n", logPath.c_str());
		disconnect_devices();
		reset_state();
		sim_start_time = Clock().now();
		omnifusion.setFirstPos();
		// Read each line from the file and store it in the
		// 'line' variable.
		while (getline(logfile, line)) 
		{
			float time;
			char messageTypeBuff[10];
			char dataTypeBuff[11];
			char dataBuff[300];
	
			int matched = sscanf(line.c_str(), "%f,%9[^,],%10[^, ]%299[^\n]", &time, &messageTypeBuff, &dataTypeBuff, &dataBuff);

			if (matched < 4) {
				continue;
			}

			string messageType(messageTypeBuff);
			string dataType(dataTypeBuff);
			string data(dataBuff);

			if (messageType != "DATA")
			{
				continue;
			}

			// Wait till time to send message through fusion. Not perfect but close enough
			//TODO: Divided by 4 to run faster
			while (elapsed(sim_start_time) < time/5)
			{
				delay(10);
			}

			if (dataType == "GNSS_CD") 
			{
				do_gnss(data);
			}
			else if (dataType == "rovl") 
			{
				do_rovl(data);
			}
			else if (dataType == "MAV") 
			{
				do_mav(data);
			}
			else if (dataType == "tracker650") 
			{
				do_tracker(data);
			}
			//TODO: DVL and Mavlink
		}
		// Close the file stream once all lines have been
		// read.
		logfile.close();
	}
	else
	{
		// Print an error message to the standard error
		// stream if the file cannot be opened.
		printf("Error\n");
	}
	log_event("Simulation over. Reconnecting devices and resetting state...\n", logPath);
	reconnect_devices();
	reset_state();
}

void LogSimulator::do_gnss(string data) {
	int stat;
	Quaternion Qor, Qoff;
	float rr, pr, yr, lat, lon, r, p, y, h;

	int matched = sscanf(data.c_str(), ", stat, %03X, Qor {, %f, %f, %f, %f, }, Qoff"
		" {, %f, %f, %f, %f, }, rr, %f, pr, %f, yr, %f, lat, %f, lon %f, r, %f, p, %f, y, %f, h, %f",
		&stat, &Qor.w, &Qor.x, &Qor.y, &Qor.z, &Qoff.w, &Qoff.x, &Qoff.y, &Qoff.z, &rr, &pr, &yr, &lat, &lon, &r, &p, &y, &h);

	if (matched < 18) {
		return;
	}
	gnss_status = stat;

	vec3 gyro = vec3(rr, pr, yr);

	gnss_orientation = Qor;

	omnifusion.fuseGnss(Qor, lat, lon, gnss_position_valid, gyro.length());

}

void LogSimulator::do_rovl(string data) {
	if (!contains("$USRTH", data))
	{
		return;
	}

	parse_usrth(data);

	loop();
	
	//omnifusion.fuseRovl(rovl_usrth.apparent_bearing_math, rovl_usrth.apparent_elevation, rovl_usrth.slant_range);


	//TODO: Testing here can delete later
	//omnifusion.sendRovlTrueToMap(rovl_usrth.true_bearing_math, rovl_usrth.true_elevation, rovl_usrth.slant_range);
	//omnifusion.sendRovlRawToMap(rovl_usrth.apparent_bearing_math, rovl_usrth.apparent_elevation, rovl_usrth.slant_range);
}

void LogSimulator::do_mav(string data) {
	if (contains("ORIENTATION", data))
	{
		head_of(data, ",", false); // lop off front of message 
		mav_roll = std::stod(head_of(data, ",", false));
		mav_pitch = std::stod(head_of(data, ",", false));
		mav_yaw = std::stod(head_of(data, ",", false));
		// todo: do something with mavlink message

		omnifusion.fuseMavlinkOrientation(mav_roll, mav_pitch, mav_yaw);
	}
	else if (contains("GLOBAL_POSITION_INT", data))
	{
		head_of(data, ",", false); // lop off front of message 
		double lat = std::stod(head_of(data, ",", false)) / 10000000;
		double lon = std::stod(head_of(data, ",", false)) / 10000000;
		omnifusion.fuseBlueBoatLocation(lat, lon);
	}
}

void LogSimulator::do_tracker(string data) {
	if (contains("$DVPDX", data))
	{
		parse_dvpdx(data, t650_dvpdx);

		omnifusion.fuseDvl(t650_dvpdx.position_group_valid, t650_dvpdx.position_delta_x, t650_dvpdx.position_delta_y, t650_dvpdx.position_delta_z);
	}
}