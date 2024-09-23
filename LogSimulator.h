#pragma once
#include <string>
#include "utilities.h"

using std::string;
class LogSimulator {
	TIMING sim_start_time;

public:
	void runSimulation(string logPath);

private:

	void do_rovl(float time, string data);
	void do_gnss(float time, string data);
};