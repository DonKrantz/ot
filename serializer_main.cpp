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
#include "LogSimulator.h"

#include "OmniUkf.h"

namespace {

#define ROVL_NAME "rovl"
#define GNSS_NAME "gnss"
#define TRACKER650_NAME "tracker650"
#define MAVLINK_NAME "mavlink"
#define REPLICATION_NAME "replication"
#define INTERNAL_NAME "sending"
#define WATCHDOG_NAME "watchdog"
#define MAVLINK_NAME "mavlink"


MAVlink* mavlink;

double mavlink_last_received_mission_time = 0.0;

   // **************************************************************************************
   // The database struct used to manage the serialization of messages by "port" or "channel"
   struct ports_type
   {
      enum PORT_TYPE port_type; // the mechanism used for the port
      int fd; // the fd of the message channel (UDP, serial, etc,)
      int replicate_fd; // the replication UDP fd, if appropriate
      string serialname;  // has multiple uses to identify the port mechanisms
      const char* devicename;  // human-readable name of port
      short listening_port;  // used on UDP ports
      int baud_rate; // used only on serial devices
      struct sockaddr_in cliaddr;   // source of UDP messages
      void (*callto)(uint8_t*, size_t len, double timestamp); // callback with data
      bool line_mode; // for serial devices -- linux canonical or non-canonical
      bool switched_off; // id incoming poll is connected or not -- does not affect outgoing !!!!!
      double last_received_mission_time; // timestamp of last received on this channel
   };

   // callto funciton prototypes
   void send_to_null(uint8_t* buffer, size_t len, double timestamp);
   void send_to_rovl(uint8_t* buffer, size_t len, double timestamp);
   void send_to_gnss(uint8_t* buffer, size_t len, double timestamp);
   void send_to_t650(uint8_t* buffer, size_t len, double timestamp);
   void send_to_internal(uint8_t* buffer, size_t len, double timestamp);
   void incoming_rovl(uint8_t* buffer, size_t len, double timestamp);
   void incoming_gnss(uint8_t* buffer, size_t len, double timestamp);
   void incoming_tracker650(uint8_t* buffer, size_t len, double timestamp);


   // **************************************************************************************
   // The active ports in the system. Serial are bi-directional, UDP are either for sending
   // or receiving, the TCP option is not yet implemented.
   // 
   // These are in the order of the PORTS class enum!
   struct ports_type port_list[] =
   {
      { PORT_TYPE::UDP, -1, -1, "internal-listening", WATCHDOG_NAME, 0, B115200, {0}, send_to_internal, true, false, 0.0 },
      { PORT_TYPE::SERIAL,  -1, -1, "port-rovl-rx", ROVL_NAME, 0, B115200, {0}, incoming_rovl, true, false, 0.0 },
      { PORT_TYPE::SERIAL, -1, -1, "port-gnss-control", GNSS_NAME, 0, B115200, {0}, incoming_gnss, false, false, 0.0 },
      { PORT_TYPE::UDP, -1, -1, "port-tracker650", TRACKER650_NAME, 0, B115200, {0}, incoming_tracker650, true, false, 0.0 },
      { PORT_TYPE::UDP, -1, -1, "gnss-replicate", REPLICATION_NAME, 0, B115200, {0}, send_to_gnss, true, false, 0.0 },
      { PORT_TYPE::UDP, -1, -1, "rovl-replicate", REPLICATION_NAME, 0, B115200, {0}, send_to_rovl, true, false, 0.0 },
      { PORT_TYPE::UDP, -1, -1, "tracker650-replicate", REPLICATION_NAME, 0, B115200, {0}, send_to_t650, true, false, 0.0 },
      { PORT_TYPE::UDP, -1, -1, "mavlink-replicate", REPLICATION_NAME, 0, B115200, {0}, send_to_null , true, false, 0.0 },
      { PORT_TYPE::UDP, -1, -1, "internal-sending", INTERNAL_NAME, 0, B115200, {0}, send_to_null, true, false, 0.0 },
      { PORT_TYPE::UDP, -1, -1, "mavlink-sending", MAVLINK_NAME, 0, B115200, {0}, send_to_null, true, false, 0.0 },
      { PORT_TYPE::UDP, -1, -1, "mavlink-listening", MAVLINK_NAME, 0, B115200, {0}, send_to_null, true, true, 0.0 },
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
         mav_global_origin_valid = true;
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

         omnifusion.fuseMavlinkOrientation(mav_roll, mav_pitch, mav_yaw);
      }
      else if (contains("GLOBAL_POSITION_INT", message))
      {
          head_of(message, ",", false); // lop off front of message 
          double lat = std::stod(head_of(message, ",", false)) / 10000000;
          double lon = std::stod(head_of(message, ",", false)) / 10000000;

          omnifusion.fuseBlueBoatLocation(lat, lon);
      }

      else if (contains("MANUAL_CONTROL", message))
      {
          head_of(message, ",", false); // lop off front of message 
          int16_t x = std::stoi(head_of(message, ",", false));
          int16_t y = std::stoi(head_of(message, ",", false));
          int16_t z = std::stoi(head_of(message, ",", false));
          int16_t r = std::stoi(head_of(message, ",", false));
      }
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
      mav_global_origin_valid = false;
      log_event("MAVlink connected");
   }

   // **************************************************************************************
   void mavlink_disconnected()
   {
      //todo: anything needed when ROVL connects
      mav_global_origin_valid = false;
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
   // Checks a single port/channel to see if it has timed out waiting for a mesage.
   // Calls disconnected() parameter on timeout and connected() parameter on re-connect
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
   // processed the ports/channels that need timeout monitoring
   void check_watchdog(double timestamp)
   {
      check_comm(port_list[(int)PORTS::ROVL_RX].last_received_mission_time,
         timestamp, 3.0, rovl_comm_active, rovl_connected, rovl_disconnected);

      check_comm(port_list[(int)PORTS::GNSS].last_received_mission_time,
         timestamp, 3.0, gnss_comm_active, gnss_connected, gnss_disconnected);

      check_comm(port_list[(int)PORTS::TRACKER650].last_received_mission_time,
         timestamp, 2.0, t650_comm_active, t650_connected, t650_disconnected);

      check_comm(mavlink_last_received_mission_time,
         timestamp, 2.0, mav_comm_active, mavlink_connected, mavlink_disconnected);
   }

   // **************************************************************************************
   // sends a message to the "internal" port of the serializer.
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
   // Puts messages into the bit bucket. Used as a placeholdeer call in the database.
   void send_to_null(uint8_t* buffer, size_t len, double timestamp)
   {
   }

   // **************************************************************************************
   // Sends a message to the PORTS::ROVL_RX device. Used by the replicators.
   void send_to_rovl(uint8_t* buffer, size_t len, double timestamp)
   {
      send_port_binary(PORTS::ROVL_RX, buffer,len);
   }

   // **************************************************************************************
    // Sends a message to the PORTS::GNSS device. Used by the replicators.
   void send_to_gnss(uint8_t* buffer, size_t len, double timestamp)
   {
      send_port_binary(PORTS::GNSS, buffer, len);
   }

   // **************************************************************************************
   // Sends a message to the PORTS::TRACKER650 device. Used by the replicators.
   void send_to_t650(uint8_t* buffer, size_t len, double timestamp)
   {
      send_port_binary(PORTS::TRACKER650, buffer, len);
   }


   // **************************************************************************************
   // Driven by the database table. Sends data to the external replication monitoring, if
   // appropriate and connected.
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
   // Called by the serilization message loop when a message becomes available. Sorts out
   // the correct mecahnism to get the message, replicates as needed, and processes the
   // message using the correct processor for the port.
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

      case PORT_TYPE::TCP: // not implemented
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
   // Sets up a socket for a UDP port replication connection.
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
   // Generates an internal message every second, which can be used to drive periodic
   // processing.
   std::thread watchdog_thread;
   void watchdog_thread_task()
   {
      log_event("Watchdog thread starting");
      prctl(PR_SET_NAME, "watchdogZombie");

      while (true)
      {
         delay(1000);
         send_port_message(PORTS::INTERNAL_RX_SENDING, "WDG tick");

         struct MAVlink_internal_message_struct msg = { MAVlinkIDs::HEARTBEAT, {0, 0} };

         send_port_binary(PORTS::MAVLINK_SENDING, &msg, sizeof(msg));
      }
   }

   std::thread ukf;
   void ukf_thread_task()
   {
       while (1)
       {
           loop();
           delay(10);
       }
   }

   // **************************************************************************************
   // Loops through MAVlink requests to periodically refresh the MAVlink ROV data. 
   std::thread mavlink_thread;
   void mavlink_thread_task()
   {
      log_event("MAVlink thread starting");
      prctl(PR_SET_NAME, "mavlinkZombie");

      bool got_global_origin = false;

      string roll = "", pitch = "", yaw = "";
      string lat = "", lon = "";
      string x = "", y = "", z = "", vx = "", vy = "", vz = "";

      // set up the MAVling sending-out listing channel
      struct pollfd poll_list[1] = { 0 };
      struct sockaddr_in tempaddr;
      poll_list[0].fd = port_list[(int)PORTS::MAVLINK_LISTENING].fd;
      poll_list[0].events = POLLIN;
      const int loop_period = 500; // not quite 2 Hz

      while (true)
      {
         // First see if we need to send stuff out, also sets the period of the loop.
         while (true)
         {
            // check for request to send MAVlink a message
            poll(poll_list, 1, loop_period);

            struct MAVlink_internal_message_struct msg = { MAVlinkIDs::HEARTBEAT, {0, 0} };

            if (poll_list[0].revents != 0)
            {
               socklen_t len = sizeof(tempaddr);
               ssize_t i = recvfrom(port_list[(int)PORTS::MAVLINK_LISTENING].fd, &msg, sizeof(msg), 0, (sockaddr*)&tempaddr, &len);
               UNUSED(i);
               switch (msg.ID)
               {
               case MAVlinkIDs::SCALED_PRESSURE:
                  mavlink->send_mavlink_scaled_pressure(
                     msg.payload.scaled_pressure.absolute_pressure,
                     msg.payload.scaled_pressure.temperature_C,
                     msg.payload.scaled_pressure.sensor_number);
                  break;

               case MAVlinkIDs::HEARTBEAT:
                  mavlink->send_mavlink_heartbeat();
                  break;

               case MAVlinkIDs::DELTA_POSITION:
                  mavlink->send_mavlink_delta_position_data(
                     msg.payload.delta_position.dx,
                     msg.payload.delta_position.dy,
                     msg.payload.delta_position.dz,
                     msg.payload.delta_position.delta_t,
                     msg.payload.delta_position.confidence);
                  break;

               case MAVlinkIDs::DISTANCE_SENSOR:
                  mavlink->send_mavlink_distance_sensor(
                     msg.payload.distance_sensor.d,
                     msg.payload.distance_sensor.confidence,
                     msg.payload.distance_sensor.quat);
                  break;

               case MAVlinkIDs::POSITION:
                  mavlink->send_mavlink_position(
                     msg.payload.position.latitude,
                     msg.payload.position.longitude);
                  break;

               case MAVlinkIDs::POSITION_UPDATE:
                  mavlink->send_mavlink_position_update(
                     msg.payload.position_update.origin_lat,
                     msg.payload.position_update.origin_lon,
                     msg.payload.position_update.new_lat,
                     msg.payload.position_update.new_lon);
                  break;

               default:
                  log_severe("unexpected mavlink send request");
                  break;
               }
            }
            else
               break; // no message pending
         }

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


         // Just for getting Blue boat position
         /*string lat, lon;
         bool result_boat_pos = mavlink->get_global_position_int(lat, lon);
         if (result_boat_pos && (lat != ""))
         {
             string message = "MAV GLOBAL_POSITION_INT," + lat + ',' + lon + "\r\n";
             send_port_message(PORTS::INTERNAL_RX_SENDING, message);
         }*/

         // Check for manual control message
         string cmd_x, cmd_y, cmd_z, cmd_r;
         bool result_manual = mavlink->get_manual_control(cmd_x, cmd_y, cmd_z, cmd_r);
         if (result_manual && x != "")
         {
             string message = "MAV MANUAL_CONTROL," + cmd_x + "," + cmd_y + "," + cmd_z + "," + cmd_r + "\r\n";
             send_port_message(PORTS::INTERNAL_RX_SENDING, message);
         }

         // check for position
         bool result_pos = mavlink->get_mavlink_local_position_ned(x, y, z, vx, vy, vz);
         if (result_pos && (x != ""))
         {
            string message = "MAV LOCAL_POSITION," + x + "," + y + "," + z + "," + vx + "," + vy + "," + vz + "\r\n";
            send_port_message(PORTS::INTERNAL_RX_SENDING, message);
         }

         bool result_or = mavlink->get_mavlink_attitude(roll, pitch, yaw);
         if (result_or && (roll != ""))
         {
            string message = "MAV ORIENTATION," + roll + "," + pitch + "," + yaw + "\r\n";
            send_port_message(PORTS::INTERNAL_RX_SENDING, message);
         }
      }
   }

   //TODO: Instantiate Omnifusion some time before this is called
   // **************************************************************************************
   // Sets up a linux poll() to look for all the possible incoming messages.
   void message_loop()
   {
      // set up the polling list
      struct pollfd poll_list[COUNT(port_list)] = { 0 };
      
      //TODO: DELETE LATER
      //setup();
      // go do the poll
      while (true)
      {
          //TODO: DELETE LATER
          
          //loop();

         for (int i = 0; i < (int)(COUNT(port_list)); i++)
         {
            poll_list[i].fd = port_list[i].fd;
            if (port_list[i].switched_off)
               poll_list[i].events = 0;
            else
               poll_list[i].events = POLLIN;
            poll_list[i].revents = 0;
         }

         poll(poll_list, (int)COUNT(poll_list), 2000);

         for (int i = 0; i < (int)(COUNT(poll_list)); i++)
         {
            if (!port_list[i].switched_off)
            {
               if (poll_list[i].revents == 0)
               {
               }
               else if ((poll_list[i].revents & (POLLHUP | POLLRDHUP | POLLERR)) != 0)
               {
               }
               else if (poll_list[i].revents == POLLIN)
               {
                  process_message(i);
               }
               else
               {
                  log_severe("Serializer message moll triggered by unexpected revents value\n");
               }
            }
         }
      }
   }

   //==========================================================================================
   // open the various types of input ports/channels on startup
   void ports()
   {
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

         case PORT_TYPE::TCP: // not implemented
            break;

         case PORT_TYPE::UDP:
            if (config.lookup(port_list[i].serialname) == "off")
            {
               log_event("Skipping port setup for %s", port_list[i].devicename);
               continue;
            }
            {
               string port = "";
               if (contains("replicate", port_list[i].serialname) || contains("internal", port_list[i].serialname) || contains("mavlink", port_list[i].serialname))
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

      // Set up the destination UDP for the internal message channel
      port_list[(int)PORTS::INTERNAL_RX_SENDING].cliaddr.sin_family = AF_INET;
      port_list[(int)PORTS::INTERNAL_RX_SENDING].cliaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      port_list[(int)PORTS::INTERNAL_RX_SENDING].cliaddr.sin_port = htons((int16_t)std::stoi(config.lookup("internal-listening-port")));

      // Set up the destination UDP for the MAVlink outgoing message channel
      port_list[(int)PORTS::MAVLINK_SENDING].cliaddr.sin_family = AF_INET;
      port_list[(int)PORTS::MAVLINK_SENDING].cliaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      port_list[(int)PORTS::MAVLINK_SENDING].cliaddr.sin_port = htons((int16_t)std::stoi(config.lookup("mavlink-listening-port")));

      // start watchdog
       watchdog_thread = std::thread(watchdog_thread_task);

      //start MAVlink REST interface
      string mavlink_port = config.lookup("mavlink-ip");
      string mavlink_ip = head_of(mavlink_port, ":", false);
      string system_id = config.lookup("mavlink-sysid");
      string vehicle_id = config.lookup("mavlink-vehid");
      mavlink = new MAVlink(mavlink_ip, std::stoi(mavlink_port), std::stoi(system_id), std::stoi(vehicle_id), 10.0f);

      mavlink_thread = std::thread(mavlink_thread_task);

      setup();
      ukf = std::thread(ukf_thread_task);

      message_loop();
   }


} // namespace

//TODO: Disconnect and reconnect for sim. Reset system state before and after

// **************************************************************************************
// Disconnects a port/channel from the message receive loop. IMPORTANT: does not change
// message sending, so yo may neet to checked 'switched off' status when sending.  
void port_disconnect(enum PORTS device)
{
   port_list[(int)device].switched_off = true;
   send_port_message(PORTS::INTERNAL_RX_SENDING, "");
}

// **************************************************************************************
// Reconnects a port/channel to the message receive loop.
void port_reconnect(enum PORTS device)
{
   port_list[(int)device].switched_off = false;
   send_port_message(PORTS::INTERNAL_RX_SENDING, "");
}

// **************************************************************************************
// Sends a binary message to a port/channel using the appropriate mechanism.
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
// Sends a string message to a port/channel using the appropriate mechanism.
// todo: could probably re-implement in terms of send_port_binary().
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
         log_warning("failed sending message in port to %d: %s\n", (int)device, strerror(errno));
      user_mode();

   default:
      break;
   }

   if (s < 0)
   {
      log_warning("failed sending string in port to %d: %s", (int)device, strerror(errno));
      return false;
   }

   return true;
}


// **************************************************************************************
// This is used by the ST Firmware bootloader to slam the ROVL_RX port between the
// normal mode 115200,n,8,1 (cononical/line mode) and bootloader mode 
// 115200,e,8,1 (non/canonical raw)
bool port_set_canonical(enum PORTS device, bool canonical, bool set_even_parity)
{
   // map device to order of ports_list
   int mapped_device = (int)device;

   ztclose(port_list[mapped_device].fd);

   sudo_mode();
   string filename = config.lookup(port_list[mapped_device].serialname);

   port_list[mapped_device].fd = openSerialDevice(filename, true, port_list[mapped_device].baud_rate,
      canonical, set_even_parity, false, false);
   user_mode();

   if (port_list[mapped_device].fd < 0)
   {
      log_severe("Opening serial device %d in ports: %s", mapped_device, strerror(errno));
      return false;
   }

   tcflush(port_list[mapped_device].fd, TCIOFLUSH);

   return true;
}


// **************************************************************************************
// Entry point
bool ports_main()
{
   log_event("Serializer starting");
   prctl(PR_SET_NAME, "serializerZombie");

   ports(); // normally does not return

   return false;
}

// **************************************************************************************
// Used by external users of a port to receive messages. The port should be disconneted
// from the serializer message loop before calling this.
int receive_from(enum PORTS device, void* message, size_t sizeis, int timeout_ms)
{
   if (port_list[(int)device].port_type == PORT_TYPE::SERIAL)
   {
      struct pollfd poll_list[1] = { port_list[(int)device].fd, POLLIN, 0 };
      poll(poll_list, 1, timeout_ms);

      if (poll_list[0].revents != 0)
      {
         ssize_t i = read(port_list[(int)device].fd, message, sizeis);
         return (int)i;
      }
      else
      {
         return 0;
      }
   }

   else if (port_list[(int)device].port_type != PORT_TYPE::UDP)
   {
      struct pollfd poll_list[1] = { 0 };
      struct sockaddr_in tempaddr;

      poll_list[0].fd = port_list[(int)device].fd;
      poll_list[0].events = POLLIN;
      poll(poll_list, 1, timeout_ms);

      if (poll_list[0].revents != 0)
      {
         socklen_t len = sizeof(tempaddr);
         ssize_t i = recvfrom(port_list[(int)device].fd, message, sizeis, 0, (sockaddr*)&tempaddr, &len);
         return (int)i;
      }
      else
      {
         return 0;
      }
   }

   else
   {
      log_severe("unimplemented receive type %d in serilaizer", (int)device);
      return 0;
   }
}

// **************************************************************************************
// String version. The port should be disconneted from the serializer message loop
// before calling this.
string receive_from(enum PORTS device, int timeout_ms)
{
   uint8_t buffer[1600];
   int len = receive_from(device, buffer, sizeof(buffer), timeout_ms);

   string result((char*)buffer, len);
   return result;   
}


