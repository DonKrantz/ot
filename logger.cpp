

#include "utilities.h"
#include "logger.h"
#include "configuration.h"
#include <assert.h>
#include <stdio.h>
#include <iostream>

namespace {

   FILE* logfile = NULL;

   long int bytes_out = 0;
   long int byte_limit = 0;

   //==========================================================================================
   void purge_log_files()
   {
      int maxnum = stoi(config.lookup("logger-maxnum"));

      for (int i = maxnum; i < 10; i++)
      {
         string cmd = "rm -f " + config.lookup("webhome") + "/logs/" + LOGFILE_ROOT + std::to_string(i) + ".log";
         system(cmd.c_str());
      }

      for (int i = maxnum - 2; i >= 0 ; i--)
      {
         string cmd = "mv -f " + config.lookup("webhome") + "/logs/" + LOGFILE_ROOT + std::to_string(i) + ".log" 
            + " " + config.lookup("webhome") + "/logs/" + LOGFILE_ROOT + std::to_string(i + 1) + ".log" + " >/dev/null 2>/dev/null";
         system(cmd.c_str());
      }
   }


} // namespace


//==========================================================================================


void log_write(const char* fmt, ...)
{
   va_list args{};
   va_start(args, fmt);

   if (bytes_out > byte_limit)
   {
      fclose(logfile);
      logfile = NULL;
      log_initialize();
   }

   char buffer[512]{ 0 };
   vsnprintf(buffer, sizeof(buffer), fmt, args);

   // for debugging purposes, writes to stdout
   if( !contains("DATA,", (string)buffer) )
      printf("%1.4f,%s\n", elapsed(mission_start_time), buffer);

   if (logfile == NULL)
      return;

   bytes_out += fprintf(logfile, "%1.4f,%s\n", elapsed(mission_start_time), buffer);
   fflush(logfile);

   va_end(args);
}

//==========================================================================================
void log_initialize()	// only want to do this once in the master thread...
{
   assert(logfile == NULL);

   if (config.lookup("logger") != "on")
      return;

   purge_log_files();

   string fn = config.lookup("webhome") + "/logs/" + LOGFILE_ROOT + "0.log";
   logfile = fopen( fn.c_str(), "w");

   system(("chgrp -f " + config.lookup("user") + " " + fn).c_str());

   bytes_out = 0;

   byte_limit = std::stol(config.lookup("logger-maxsize")) * 1024 * 1024;

   log_write("Log file started,%s", time_image("%y/%m/%d,%H:%M:%S").c_str());
}
