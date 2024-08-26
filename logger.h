#pragma once

#include <stdarg.h>
#include <stdio.h>
#include "utilities.h"

void log_write(const char* fmt, ...);

void log_initialize();	// only want to do this in the master thread...

#define LOGFILE_ROOT MYNAME 

