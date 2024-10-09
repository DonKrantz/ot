#pragma once
#include <string>
#include "utilities.h"

using std::string;
class LogSimulator {
	TIMING sim_start_time;

public:
	void runSimulation(string logPath);

private:

	void do_rovl(string data);
	void do_gnss(string data);
	void do_mav(string data);
	void do_tracker(string data);

	void disconnect_devices();
	void reconnect_devices();
};