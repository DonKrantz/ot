#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/ioctl.h>
#include <termio.h>
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <sstream>      // std::ostringstream
#include <math.h>
#include <stdlib.h>
#include <pwd.h>
#include <cassert>
#include <netinet/in.h>
#include <poll.h>
#include <sys/prctl.h>
#include <signal.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>

#include <array>

#include "utilities.h"
#include "configuration.h"
#include "serializer_main.h"

#include "rovl.h"
#include "gnss_ping_protocol.h"
#include "tracker650.h"

#include "MAVlink.h"

#include "system_state.h"

namespace {

#define ROVL_NAME "rovl"
#define GNSS_NAME "gnss"
#define TRACKER650_NAME "tracker650"
#define MAVLINK_NAME "mavlink"
#define REPLICATION_NAME "replication"
#define INTERNAL_NAME "sending"
#define WATCHDOG_NAME "watchdog"


MAVlink* mavlink;

double mavlink_last_received_mission_time = 0.0;

   // **************************************************************************************
   // 
   struct ports_type
   {
      enum PORT_TYPE port_type;
      int fd;
      int replicate_fd;
      string serialname;
      const char* devicename;
      short listening_port;
      int baud_rate;
      struct sockaddr_in cliaddr;
      void (*callto)(uint8_t*, size_t len, double timestamp);
      bool line_mode;
      bool switched_off;
      double last_received_mission_time;
   };

   void send_to_null(uint8_t* buffer, size_t len, double timestamp);
   void send_to_internal(uint8_t* buffer, size_t len, double timestamp);
   void incoming_rovl(uint8_t* buffer, size_t len, double timestamp);
   void incoming_gnss(uint8_t* buffer, size_t len, double timestamp);
   void incoming_tracker650(uint8_t* buffer, size_t len, double timestamp);


   // **************************************************************************************
   // The active ports in the system. Serial are bi-directional, UDP are either for sending
   // or receiving, the TCP option is not implementsed.
   struct ports_type port_list[] =
   {
      { PORT_TYPE::UDP, -1, -1, "internal-listening", WATCHDOG_NAME, 0, B115200, {0}, send_to_internal, true, false },
      { PORT_TYPE::SERIAL,  -1, -1, "port-rovl-rx", ROVL_NAME, 0, B115200, {0}, incoming_rovl, true, false },
      { PORT_TYPE::SERIAL, -1, -1, "port-gnss-control", GNSS_NAME, 0, B115200, {0}, incoming_gnss, false, false },
      { PORT_TYPE::UDP, -1, -1, "port-tracker650", TRACKER650_NAME, 0, B115200, {0}, incoming_tracker650, true, false },
      { PORT_TYPE::UDP, -1, -1, "gnss-replicate", REPLICATION_NAME, 0, B115200, {0}, send_to_null, true, false },
      { PORT_TYPE::UDP, -1, -1, "rovl-replicate", REPLICATION_NAME, 0, B115200, {0}, send_to_null, true, false },
      { PORT_TYPE::UDP, -1, -1, "tracker650-replicate", REPLICATION_NAME, 0, B115200, {0}, send_to_null, true, false },
      { PORT_TYPE::UDP, -1, -1, "mavlink-replicate", REPLICATION_NAME, 0, B115200, {0}, send_to_null , true, false },
      { PORT_TYPE::UDP, -1, -1, "internal-sending", INTERNAL_NAME, 0, B115200, {0}, send_to_null, true, false },
   };



   // **************************************************************************************
   void incoming_rovl(uint8_t* buffer, size_t len, double timestamp)
   {
      string s = "";

      if (len == 0)
         s = "";
      else if (len == 1)
         s = "" + buffer[0];
      else
      {
         string ss((char*)buffer, (size_t)len - 2); // assumes there is a crlf ar the end!
         s = ss;
      }

      log_data("%s,%s", ROVL_NAME, s.c_str());
      
      parse_rovlrx(s, timestamp);
   }

   // **************************************************************************************
   void incoming_gnss(uint8_t* buffer, size_t len, double timestamp)
   {
      process_incoming_gnss(buffer, len, timestamp);
   }

   // **************************************************************************************
   void rovl_connected()
   {
      //todo: anything needed when ROVL connects
      log_event("ROVL connected");
   }

   // **************************************************************************************
   void rovl_disconnected()
   {
      //todo: anything needed when ROVL connects
      log_event("ROVL disconnected");
   }

   // **************************************************************************************
   void gnss_connected()
   {
      //todo: anything needed when ROVL connects
      log_event("GNSS connected");
   }

   // **************************************************************************************
   void gnss_disconnected()
   {
      //todo: anything needed when ROVL connects
      log_event("GNSS disconnected");
   }

   // **************************************************************************************
   void mavlink_connected()
   {
      //todo: anything needed when ROVL connects
      log_event("MAVlink connected");
   }

   // **************************************************************************************
   void mavlink_disconnected()
   {
      //todo: anything needed when ROVL connects
      log_event("MAVlink disconnected");
   }

   // **************************************************************************************
   void t650_connected()
   {
      //todo: anything needed when ROVL connects
      log_event("Tracker 650 connected");
   }

   // **************************************************************************************
   void t650_disconnected()
   {
      //todo: anything needed when ROVL connects
      log_event("Tracker 650 disconnected");
   }

   // **************************************************************************************
   void incoming_tracker650(uint8_t* buffer, size_t len, double timestamp)
   {
      string s = "";

      if (len == 0)
         s = "";
      else if (len == 1)
         s = "" + buffer[0];
      else
      {
         if (*((char*)buffer + len - 2) == '\r')
         {
            string ss((char*)buffer, (size_t)len - 2); // assumes there is a crlf at the end!
            s = ss;
         }
         else
         {
            string ss((char*)buffer, (size_t)len - 1); // assumes there is a only lf at the end!
            s = ss;
         }
      }
         

      log_data("%s,%s", TRACKER650_NAME, s.c_str());

      parse_t650(s, timestamp);
   }


   // **************************************************************************************
   void incoming_mavlink(string message, double timestamp)
   {
      log_data("%s", message.c_str());

      mavlink_last_received_mission_time = timestamp; // handled differently than the others

      if (contains("GLOBAL_ORIGIN", message))
      {
         head_of(message, ",", false); // lop off front of message 
         mav_global_origin_lat = std::stod(head_of(message, ",", false));
         mav_global_origin_lon = std::stod(head_of(message, ",", false));
         // todo: do something with mavlink message
      }
      else if (contains("LOCAL_POSITION", message))
      {
         head_of(message, ",", false); // lop off front of message 
         mav_x = std::stod(head_of(message, ",", false));
         mav_y = std::stod(head_of(message, ",", false));
         mav_z = std::stod(head_of(message, ",", false));
         mav_vx = std::stod(head_of(message, ",", false));
         mav_vy = std::stod(head_of(message, ",", false));
         mav_vz = std::stod(head_of(message, ",", false));
         // todo: do something with mavlink message
      }
      else if (contains("ORIENTATION", message))
      {
         head_of(message, ",", false); // lop off front of message 
         mav_roll = std::stod(head_of(message, ",", false));
         mav_pitch = std::stod(head_of(message, ",", false));
         mav_yaw = std::stod(head_of(message, ",", false));
         // todo: do something with mavlink message
      }

   }

   // **************************************************************************************
   void check_comm(double last_touched, double timestamp, double limit, bool& active, 
      void (*connected)(void), void (*disconnected)(void))
   {
      bool ok = (timestamp - last_touched) < limit;
      if ((active && ok) || (!active && !ok))
      {
         // no action needed
      }
      else if (active && !ok)
      {
         disconnected();
         active = false;
      }
      else // !active && OK
      {
         connected();
         active = true;
      }
   }

   // **************************************************************************************
   void check_watchdog(double timestamp)
   {
      check_comm(port_list[(int)PORTS::ROVL_RX].last_received_mission_time,
         timestamp, 2.0, rovl_comm_active, rovl_connected, rovl_disconnected);

      check_comm(port_list[(int)PORTS::GNSS].last_received_mission_time,
         timestamp, 2.0, gnss_comm_active, gnss_connected, gnss_disconnected);

      check_comm(port_list[(int)PORTS::TRACKER650].last_received_mission_time,
         timestamp, 2.0, t650_comm_active, t650_connected, t650_disconnected);

      check_comm(mavlink_last_received_mission_time,
         timestamp, 2.0, mav_comm_active, mavlink_connected, mavlink_disconnected);
   }

   // **************************************************************************************
   void send_to_internal(uint8_t* buffer, size_t len, double timestamp)
   {
      string msg((char*)buffer, len);
      if (contains("WDG", msg))
      {
         check_watchdog(timestamp);
         return;
      }
      else if (contains("MAV", msg))
      {
         incoming_mavlink(msg, timestamp);
      }
   }

   // **************************************************************************************
   void send_to_null(uint8_t* buffer, size_t len, double timestamp)
   {
   }

   
   void replicate(int fd, uint8_t* buffer, int n, struct sockaddr_in& cliaddr )
   {
      if (fd < 0)
         return;

      if (cliaddr.sin_port != 0)
      {
         ssize_t nn = sendto(fd, buffer, n, MSG_CONFIRM, (const sockaddr*)&cliaddr, sizeof(cliaddr));
         if (nn < n)
            log_warning("data loss replicating fd %d: %s\n", fd, strerror(errno));
      }
   }
   
   // **************************************************************************************
   void process_message(int index)
   {
      uint8_t buffer[1600];
      double timestamp;

      // save a timestamp and keep for later
      timestamp = elapsed( mission_start_time );
      port_list[index].last_received_mission_time = timestamp;

      switch (port_list[index].port_type)
      {

      case PORT_TYPE::SERIAL:
      {
         ssize_t i = read(port_list[index].fd, buffer, sizeof(buffer) - 1);

         // send the data for processing
         port_list[index].callto(buffer, i, timestamp);

         // replicate as needed
         if (index == (int)PORTS::ROVL_RX)
            replicate(port_list[(int)PORTS::ROVL_REPLICATE].replicate_fd, buffer, (int)i, port_list[(int)PORTS::ROVL_REPLICATE].cliaddr);
         else if (index == (int)PORTS::GNSS)
            replicate(port_list[(int)PORTS::GNSS_REPLICATE].replicate_fd, buffer, (int)i, port_list[(int)PORTS::GNSS_REPLICATE].cliaddr);
         break;
      }

      case PORT_TYPE::TCP:
         break;

      case PORT_TYPE::UDP:
      {
         struct sockaddr_in tempaddr;

         socklen_t len = sizeof(port_list[index].cliaddr);
         ssize_t i = recvfrom(port_list[index].fd, buffer, sizeof(buffer), 0, (sockaddr*)&tempaddr, &len);

         if (tempaddr.sin_addr.s_addr != htonl(INADDR_LOOPBACK))
            port_list[index].cliaddr = tempaddr;

         // skip null used to start replication connection
         if ((i == 1) && (buffer[0] == 0))
            return;

         // replicate as needed
         else if (index == (int)PORTS::TRACKER650)
            replicate(port_list[(int)PORTS::TRACKER650_REPLICATE].replicate_fd, buffer, (int)i, port_list[(int)PORTS::TRACKER650_REPLICATE].cliaddr);
         else if (index == (int)PORTS::INTERNAL_RX)
         {
            string msg((char*)buffer, i);
            if( contains("MAV", msg))
               replicate(port_list[(int)PORTS::MAVLINK_REPLICATE].replicate_fd, buffer, (int)i, port_list[(int)PORTS::MAVLINK_REPLICATE].cliaddr);
         }

         port_list[index].callto(buffer, i, timestamp);

         break;
      }

      default:
         break;
      }
   }

   //==========================================================================================
   void open_replicate(PORTS index, string key)
   {
      if (config.lookup(key) != "on")
         return;

      if ((port_list[(int)index].replicate_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
      {
         log_severe("Opening replication socket! Error is: %s", strerror(errno));
      }
   }


   // **************************************************************************************
   std::thread watchdog_thread;
   void watchdog_thread_task()
   {
      while (true)
      {
         delay(1000);
         send_port_message(PORTS::INTERNAL_RX_SENDING, "WDG tick");
      }
   }

   // **************************************************************************************
   std::thread mavlink_thread;
   void mavlink_thread_task()
   {
      bool got_global_origin = false;

      string roll = "", pitch = "", yaw = "";
      string lat = "", lon = "";
      string x = "", y = "", z = "", vx = "", vy = "", vz = "";

      while (true)
      {
         delay(500); // not quite 2 Hz

         // check for global origin
         if (!got_global_origin)
         {
            string result = mavlink->get_mavlink_global_origin(lat, lon);
            if ((result != "failed") && (result != "none"))
            {
               string message = "MAV GLOBAL_ORIGIN," + lat + "," + lon + "\r\n";
               send_port_message(PORTS::INTERNAL_RX_SENDING, message);
            }
         }

         // check for position
         bool result_pos = mavlink->get_mavlink_local_position_ned(x, y, z, vx, vy, vz);
         if (result_pos)
         {
            string message = "MAV LOCAL_POSITION," + x + "," + y + "," + z + "," + vx + "," + vy + "," + vz + "\r\n";
            send_port_message(PORTS::INTERNAL_RX_SENDING, message);
         }

         bool result_or = mavlink->get_mavlink_attitude(roll, pitch, yaw);
         if (result_or)
         {
            string message = "MAV ORIENTATION," + roll + "," + pitch + "," + yaw + "\r\n";
            send_port_message(PORTS::INTERNAL_RX_SENDING, message);
         }
      }
   }


   //==========================================================================================
   void ports()
   {
      // open the various types in inputs
      for (unsigned int i = 0; i < COUNT(port_list); i++)
      {
         switch (port_list[i].port_type)
         {

         case PORT_TYPE::SERIAL:
         {
            sudo_mode();
            string filename = config.lookup(port_list[i].serialname);
            port_list[i].fd = openSerialDevice(filename, true, port_list[i].baud_rate,
               port_list[i].line_mode, false, false, false);
            user_mode();
            if (port_list[i].fd < 0)
            {
               log_severe("Opening serial device %d in ports: %s", i, strerror(errno));
               continue;
            }
         }
            break;

         case PORT_TYPE::TCP:
            break;

         case PORT_TYPE::UDP:
            if (config.lookup(port_list[i].serialname) == "off")
            {
               log_event("Skipping port setup for %s", port_list[i].devicename);
               continue;
            }
            {
               string port = "";
               if (contains("replicate", port_list[i].serialname) || contains("internal", port_list[i].serialname))
                  port = config.lookup(port_list[i].serialname + "-port");
               else
                  port = config.lookup(port_list[i].serialname);

               port_list[i].listening_port = (short)std::stoi(port);
               if ((port_list[i].fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
               {
                  log_severe("Opening receiving socket! Error is: %s", strerror(errno));
                  continue;
               }


               memset(&port_list[i].cliaddr, 0, sizeof(port_list[i].cliaddr));

               struct sockaddr_in servaddr;
               servaddr.sin_family = AF_INET; // IPv4 
               servaddr.sin_addr.s_addr = INADDR_ANY;
               servaddr.sin_port = htons(port_list[i].listening_port);

               int optval = 1;
               setsockopt(port_list[i].fd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
               setsockopt(port_list[i].fd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));

               // Bind the socket with the server address 
               if (bind(port_list[i].fd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
               {
                  log_severe("Binding socket! Error is: %s", strerror(errno));
                  close(port_list[i].fd);
                  continue;
               }
            }

            break;
         }
      }

      // Add replications as needed
      open_replicate(PORTS::ROVL_REPLICATE, "rovl-replicate");
      open_replicate(PORTS::GNSS_REPLICATE, "gnss-replicate");
      open_replicate(PORTS::TRACKER650_REPLICATE, "tracker650-replicate");
      open_replicate(PORTS::MAVLINK_REPLICATE, "mavlink-replicate");

      port_list[(int)PORTS::INTERNAL_RX_SENDING].cliaddr.sin_family = AF_INET;
      port_list[(int)PORTS::INTERNAL_RX_SENDING].cliaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      port_list[(int)PORTS::INTERNAL_RX_SENDING].cliaddr.sin_port = htons((int16_t)std::stoi(config.lookup("internal-listening-port")));

      // set up the polling list
      struct pollfd poll_list[COUNT(port_list)] = { 0 };

      // start watchdog
       watchdog_thread = std::thread(watchdog_thread_task);

      //start MAVlink REST interface
      string mavlink_port = config.lookup("mavlink-ip");
      string mavlink_ip = head_of(mavlink_port, ":", false);
      string vehicle_id = config.lookup("mavlink-sysid");
      mavlink = new MAVlink(mavlink_ip, std::stoi(mavlink_port), std::stoi(vehicle_id), 10.0f);

      mavlink_thread = std::thread(mavlink_thread_task);

      // go do the poll
      while (true)
      {
         for (int i = 0; i < (int)(COUNT(port_list)); i++)
         {
            poll_list[i].fd = port_list[i].fd;
            if(port_list[i].switched_off )
               poll_list[i].events = 0;
            else
               poll_list[i].events = POLLIN;
         }

         poll(poll_list, (int)COUNT(poll_list), 2000);

         for (int i = 0; i < (int)(COUNT(poll_list)); i++)
         {
            if (poll_list[i].revents != 0)
            {
               process_message(i);
//               printf(" =  poll %d\n", i);
            }
         }
      }
   }


} // namespace

// **************************************************************************************
void disconnect(enum PORTS device)
{
   port_list[(int)device].switched_off = true;
   send_port_message(PORTS::INTERNAL_RX_SENDING, "");
}

// **************************************************************************************
void reconnect(enum PORTS device)
{
   port_list[(int)device].switched_off = false;
   send_port_message(PORTS::INTERNAL_RX_SENDING, "");
}

// **************************************************************************************
bool send_port_binary(enum PORTS device, void* message, size_t sizeis)
{
   // map device to order of ports_list
   int mapped_device = (int)device;

   ssize_t s = -1;

   switch (port_list[(int)device].port_type)
   {
   case PORT_TYPE::SERIAL:
      s = write(port_list[mapped_device].fd, message, sizeis);
      break;

   case PORT_TYPE::UDP:
      if (port_list[mapped_device].cliaddr.sin_port == 0)
         break;

      s = sendto(port_list[mapped_device].fd, message, sizeis, MSG_CONFIRM, 
         (const sockaddr*)&(port_list[mapped_device].cliaddr), sizeof(port_list[mapped_device].cliaddr));

   default:
      break;
   }

   if (s < 0)
   {
      log_warning("failed sending binary in port");
      return false;
   }

   return true;
}

// **************************************************************************************
bool send_port_message(enum PORTS device, string message)
{
   // map device to order of ports_list
   int mapped_device = (int)device;

   ssize_t s;
   
   switch (port_list[(int)device].port_type)
   {
   case PORT_TYPE::SERIAL:
      s = write(port_list[mapped_device].fd, message.c_str(), message.length());
      break;

   case PORT_TYPE::UDP:
      if (port_list[mapped_device].cliaddr.sin_port == 0)
         break;
      sudo_mode();
      s = sendto(port_list[mapped_device].fd, message.c_str(), message.length(), MSG_CONFIRM, 
         (const sockaddr*)&(port_list[mapped_device].cliaddr), sizeof(port_list[mapped_device].cliaddr));
      if( s < 0)
         log_warning("Sending message in port to %d: %s\n", (int)device, strerror(errno));
      user_mode();

   default:
      break;
   }

   if (s < 0)
   {
      log_warning("failed sending string in port: %s", strerror(errno));
      return false;
   }

   return true;
}

// **************************************************************************************
bool set_baud_rate(enum PORTS device, string baud_rate)
{
   // map device to order of ports_list
   int mapped_device = (int)device;

   return setSerialDevice(port_list[mapped_device].fd, baud_rate, "8", "none", "1");
}


// **************************************************************************************
bool ports_main()
{
   log_event("Serializer starting");
   prctl(PR_SET_NAME, "serializerZombie");

   ports(); // normally does not return

   return false;
}

