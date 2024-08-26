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

#include <pwd.h>

#include <math.h>

using namespace std;

using std::string;

namespace {

#if false
   bool print_usage()
   {
      printf(
         "ot type ...\n"
         "   type is {master, web, discdem, discover, ports} \n"
         "\n"
         "\n"
      );

      return false;
   }
#endif


//   LINE_BUFFER* disc_lb = NULL;
//   LINE_BUFFER* web_lb = NULL;
//   LINE_BUFFER* ports_lb = NULL;

//   bipipe_control_block pcb0; // web server
//   bipipe_control_block pcb4; // discovery daemon
//   bipipe_control_block ports_pcb;

} // namespace

Queue main_queue;
std::thread discovery_thread;
std::thread ports_thread;
std::thread web_thread;



void kill_kids()
{
//   if (pcb0.p != 0)
//      system(((string)"sudo kill -9 " + std::to_string(pcb0.p)).c_str());

//   if (pcb4.p != 0)
//      system(((string)"sudo kill -9 " + std::to_string(pcb4.p)).c_str());

//   if (ports_pcb.p != 0)
//      system(((string)"sudo kill -9 " + std::to_string(ports_pcb.p)).c_str());
}

//==========================================================================================

void std_callback(int fd, int revents, LINE_BUFFER* lb)
{
   while (true)
   {
      string s = lb->next_line();

      if (s == "")
         return;

      if (contains("kill-self", s))
      {
         log_write("Restarted by user");
         kill_event_loop = true;
         sudo_mode();
         kill_my_children();
         string cmd = (string)argvzero + " master 0 1 &";
         system(cmd.c_str());
         exit(0);
      }

      printf("%s\n", s.c_str());
   }
}


//==========================================================================================
bool master(int argc, char* argv[])
{
   log_initialize();

   ports_thread = std::thread(ports_main);
   web_thread = std::thread(web_main);


//   gnss_main(); // maybe not a thread.


   if (config.lookup("discovery-daemon") == "on")
   {
      discovery_thread = std::thread(discovery_main);
   }

   while (true)
   {
//      string s = main_queue.dequeue();

//      if (contains("rovlrx", s))
//         parse_rovlrx((uint8_t *)s.c_str(), s.length());

//      else if (contains("rovltx", s))
//         parse_rovltx((uint8_t*)s.c_str(), s.length());

//      else
//         printf("Main: %s\n", s.c_str());

      delay(5000);
   }

   return false;
}


//==========================================================================================
int main(int argc, char* argv[])
{
   user_mode();

   // give time for the old master to kill itself
   if (strcmp(argv[1], "master") == 0)
      delay(2000);

   argvzero = (char*)malloc(strlen(argv[0]) + 1);
   strcpy(argvzero, argv[0]);

   curl_global_init(CURL_GLOBAL_ALL);

#if false
   if ((mavlink == NULL) && (config.lookup("mavlink") == "on"))
   {
      string mav_ip_port = config.lookup("mavlink-ip");
      string mav_ip = head_of(mav_ip_port, ":");
      int mav_port = std::stoi(mav_ip_port);
      int mav_id = std::stoi(config.lookup("mavlink-sysid"));
      mavlink = new MAVlink(mav_ip, mav_port, mav_id, 60.0f);
   }
#endif

   if ((argc == 1) || (strcmp(argv[1], "master") == 0))
   {
      kill_the_zombies(); // required for debugging
      prctl(PR_SET_NAME, "masterZombie");
      return !master(argc, argv);
   }

   string compname = argv[1];

//   strncpy(component_id, argv[1], sizeof(component_id));
   prctl(PR_SET_NAME, (compname + "Zombie").c_str());
   printf("Setting process name to %sZombie\n", argv[1]);

   if (strcmp(argv[1], "discdem") == 0)
      return(!discovery_main());

   else if (strcmp(argv[1], "web") == 0)
      return(!web_main());

   else if (strcmp(argv[1], "ports") == 0)
      return(!ports_main());

   log_severe("Ran off end of main(), argv[1] == %s\n", argv[1]);
}



