//#include <wiringPi.h>

#include <stdio.h>
#include <sys/ioctl.h>
#include <termio.h>
#include <termios.h>
#include <iostream>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <unistd.h> // write(), read(), close()
#include <string.h>
#include <chrono>
#include <thread>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <stdlib.h>
#include <signal.h>


//#include "GPIO.h"
#include "utilities.h"
//#include "bar-i2c.h"

#include "curl/curl.h"
#include "MAVlink.h"

#include "STbootloader_main.h"
#include "discovery_main.h"
#include "webserver.h"
#include "dispatcher.h"
#include "configuration.h"
#include "serializer_main.h"
#include "gnss_ping_protocol.h"
#include "rovl.h"

#include "LogSimulator.h"

#include <pwd.h>

#include <math.h>


#include "OmniUkf.h"

using namespace std;

using std::string;

namespace {

   std::thread discovery_thread;
   std::thread ports_thread;
   std::thread web_thread;


   //==========================================================================================
   bool master(int argc, char* argv[])
   {

      log_initialize();

      //   ports_thread = std::thread(ports_main);
      web_thread = std::thread(web_main);

      if (config.lookup("discovery-daemon") == "on")
      {
         discovery_thread = std::thread(discovery_main);
      }

      ports_main(); // should not return

      return false;
   }

}

//==========================================================================================
int main(int argc, char* argv[])
{
   user_mode();

   delay(2000);

   curl_global_init(CURL_GLOBAL_ALL);

   if ((argc > 3) && (strcmp(argv[1], "discover") == 0))
   {
      return discover_main(argc, argv);
   }

   kill_the_zombies(); // required for debugging
   prctl(PR_SET_NAME, "masterZombie");

   return !master(argc, argv);

   log_severe("Ran off end of main()\n", argv[1]);
}



