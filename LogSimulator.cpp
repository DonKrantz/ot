#include "LogSimulator.h"
#include <fstream>
#include <iostream>
#include <string>
#include "utilities.h"
#include "quaternion.h"

#include "system_state.h"

using namespace std;

void LogSimulator::runSimulation(string logPath) {
	ifstream logfile(logPath);
	string line;
	if (logfile.is_open()) 
	{
		sim_start_time = Clock().now();
		// Read each line from the file and store it in the
		// 'line' variable.
		while (getline(logfile, line)) 
		{
			float time;
			char messageTypeBuff[10];
			char dataTypeBuff[10];
			char dataBuff[300];
	
			int matched = sscanf(line.c_str(), "%f,%9[^,],%9[^,],%299[^\n]", &time, &messageTypeBuff, &dataTypeBuff, &dataBuff);

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


			if (dataType == "GNSS_CD") {
				do_gnss(time, data);
			}
			else if (dataType == "rovl") {
				do_rovl(time, data);
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
}

void LogSimulator::do_gnss(float time, string data) {
	int stat;
	Quaternion Qor, Qoff;
	float rr, pr, yr, lat, lon, r, p, y, h;

	int matched = sscanf(data.c_str(), " stat, %03X, Qor {, %f, %f, %f, %f, }, Qoff"
		" {, %f, %f, %f, %f, }, rr, %f, pr, %f, yr, %f, lat, %f, lon %f, r, %f, p, %f, y, %f, h, %f",
		&stat, &Qor.w, &Qor.x, &Qor.y, &Qor.z, &Qoff.w, &Qoff.x, &Qoff.y, &Qoff.z, &rr, &pr, &yr, &lat, &lon, &r, &p, &y, &h);

	if (matched < 18) {
		return;
	}
	gnss_status = stat;

	// Wait till time to send message through fusion
	while (elapsed(sim_start_time) < time)
		;;

	vec3 gyro = vec3(rr, pr, yr);

	omnifusion.fuseGnss(Qor, lat, lon, gnss_position_valid, gyro.length(), true);
}

void LogSimulator::do_rovl(float time, string data) {
	if (!contains("$USRTH", data))
	{
		return;
	}
	// Wait till time to send message through fusion
	while (elapsed(sim_start_time) < time)
		;;

	parse_usrth(data);
	omnifusion.fuseRovl(rovl_usrth.apparent_bearing_math, rovl_usrth.apparent_elevation, rovl_usrth.slant_range, true);

	//TODO: Testing here can delete later
	omnifusion.fuseRovlTrue(rovl_usrth.true_bearing_math, rovl_usrth.true_elevation, rovl_usrth.slant_range);
}