
// Tiny Web Server

#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <linux/usbdevice_fs.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <errno.h>
#include <map>

#include <stdlib.h>
#include <algorithm>
#include <vector>
#include <signal.h>
#include <termio.h>
#include <sys/prctl.h>
#include <chrono>

#include <filesystem>
#include <iostream>
#include <fstream>

#include "webserver.h"
#include "utilities.h"
#include "configuration.h"
#include "firmware_update.h"
#include "serializer_main.h"
#include "rovl.h"
#include "tracker650.h"
#include "STbootloader_main.h"

#include "LogSimulator.h"

#include "system_state.h"

#define PORT ((uint16_t)std::stoi(config.lookup("http-port")))
#define slash (config.lookup("webhome"))

char* arg0name = NULL;

char page_header[] = 
"<HTML>"
"<HEAD>"
"<TITLE>Cerulean " MYNAME "</TITLE>\r\n"
"<meta charset = \"utf-8\">\r\n"
"<style>\r\n"
".aligned { display: flex; align-items center; }\r\n"
"span { padding: 10px;}\r\n"
"</style>\r\n"
"</HEAD>\r\n"

"<body id = \"page-top\">\r\n"
"<body onunload=\"\">"
"<div class=\"aligned\">"
"<a href=\"/?%s\">\r\n"
"<img src=\"img/favicon.ico\" alt=\"" MYNAME " Home\" style=\"width:75px;height:50px;\">\r\n"
"</a>\r\n"
"<span>\r\n"
"<font size='5'><b>" MYNAME "</b></font> <font size='3'><i> by Cerulean Sonar</i></font>\r\n"
"</span>\r\n"
"</div>\r\n"
;

bool send_file(int fd, char image_path[], char head[]);
void do_file(int fd, string inpath, string request_type, string options = "");

char http_ok_header[] = "HTTP/1.1 200 Ok\r\n";

//==============================================================================
string mangle_t()
{
   using namespace std::chrono;
   milliseconds ms = duration_cast<milliseconds>(
      system_clock::now().time_since_epoch()
   );
   return std::to_string(ms.count());
}

//==============================================================================
string mangle_opt()
{
   return "seq=" + mangle_t();
}

//==============================================================================
void br(int fd, int n = 1)
{
   for (int i = 0; i < n; i++)
      wprintf(fd, "<br>\r\n");
}

//==============================================================================
void do_ip_port_address(int fd, string id, string ip_address, bool needs_port)
{
   for (int i = 0; i < 4; i++)
   {
      string oct = head_of(ip_address, ".:");
      wprintf(fd, "<input type='number' id='%s' name='%s.%d' value='%s' min='0' max='255' style='text-align:center; width:4em' title='IP octet' required>\r\n",
         id.c_str(), id.c_str(), i, oct.c_str());
   }

   if (needs_port)
   {
      string port = ip_address;
      wprintf(fd, ": <input type='number' id='%s' name='%s.%d' value='%s' min='0' max='65535' style='text-align:center; width:5em' title='IP Port' required>\r\n",
         id.c_str(), id.c_str(), 4, port.c_str());
   }
}

//==============================================================================
void do_euler(int fd, string id, string euler)
{
   string labels[3] = { "roll =", "pitch =", "yaw =" };
   for (int i = 0; i < 3; i++)
   {
      string angle = head_of(euler, ";");
      wprintf(fd, "%s<input type='number' id='%s' name='%s.%d' value='%s' min='-180' max='180' step='0.001' style='text-align:center; width:5em' title='Euler angle between -180 and 180 degrees' required>\r\n",
         labels[i].c_str(), id.c_str(), id.c_str(), i, angle.c_str());
   }
}

//==============================================================================
void do_on_off_value(int fd, string id, string value)
{
   wprintf(fd, "<input type='text' size='3' id='%s' name='%s' value='%s' pattern='ON|OFF|On|Off|on|off' style='text-align:center;' title='Must be ON or OFF'>\r\n",
      id.c_str(), id.c_str(), value.c_str());
}

//==============================================================================
string ip_from_opts(string id, string options, string firstval = "")
{
   string val = extract(id + ".0", options, "&");

   if (val == "")
      val = firstval;

   string vals[5] = { "","","","","" };
   vals[0] = val;

   for (int i = 1; i < 5; i++)
   {
      string r = extract(id + "." + std::to_string(i), options, "&");
      vals[i] = r;
   }

   if ((vals[0] == "") || (vals[1] == "") || (vals[2] == "") || (vals[3] == ""))
      return "";

   string result = vals[0] + "." + vals[1] + "." + vals[2] + "." + vals[3];
   if (vals[4] != "")
      result += (":" + vals[4]);

   return result;
}


//==============================================================================
void commit_reset(int fd, string button_label = "Commit", bool reboot = false)
{
   wprintf(fd, "<input type='hidden' id='fseq' name='fseq' value='%s'>\r\n",
      mangle_t().c_str());
   
   wprintf(fd, "<input type=\"submit\" value=\"%s\">\r\n", button_label.c_str());
   wprintf(fd, "<span padding='2'></span>\r\n");
   wprintf(fd, "<input type=\"reset\">\r\n");
   wprintf(fd, "<span padding='3'>\r\n");

   if( reboot)
      wprintf(fd, "</span><a href='reboot'><font size = '4'>Reboot Now</font></a>\r\n");
   else
      wprintf(fd, "</span><a href='restart'><font size = '4'>Restart Now</font></a>\r\n");

   br(fd, 1);

   wprintf(fd, "<p><b>Changes are not saved until committed.</b> Most saved/committed changes\r\n");
   wprintf(fd, "do not take effect until the system restarts. Restarting or rebooting before committing undoes any changes you have\r\n");
   wprintf(fd, "made to this page, as does navigating away from the page.</p>\r\n");
}

//==============================================================================
void standard_header(int fd, string page_title)
{
   string header = (string)http_ok_header + (string)"HTTP/1.1 200 Ok\r\nContent-Type: text/html\r\n\r\n";
   wprintf(fd, header.c_str());
   wprintf(fd, page_header, mangle_opt().c_str());
   if (page_title != "")
      wprintf(fd, "<h2>%s</h2>\r\n", page_title.c_str());
}

//==============================================================================
void standard_footer(int fd)
{
   wprintf(fd, "</body>");
   wprintf(fd, "</HTML>");
}

//==============================================================================
void radio(int fd, string label, string name, string value, bool checked, int breaks, bool ok_to_use = true, string caveat = "")
{
   static int n = 0;
   string id = "r" + std::to_string(n++);

   wprintf(fd, "<input type=\"radio\" id=\"%s\" name=\"%s\" value=\"%s\" %s %s\r\n",
      id.c_str(), name.c_str(), value.c_str(), checked ? "checked" : "", ok_to_use ? "" : "disabled");
   wprintf(fd, "<label for=\"%s\"> %s</label>\r\n", id.c_str(), label.c_str());
   wprintf(fd, "%s\r\n", caveat.c_str());
   
   if (breaks >= 0)
      br(fd, breaks);


   if (breaks < 0)
      wprintf(fd, "<span padding='%d'></span>\r\n", -breaks);
}


//==============================================================================
void checkbox(int fd, string label, string name, bool checked, int breaks)
{
   wprintf(fd, "<input type=\"checkbox\" name=\"%s\" %s>", name.c_str(), checked ? "checked" : "");
   wprintf(fd, "<label for=\"%s\">%s</label>\r\n", name.c_str(), label.c_str());

   if (breaks >= 0)
      br(fd, breaks);

   if (breaks < 0)
      wprintf(fd, "<span padding='%d'></span>\r\n", -breaks);
}


//==============================================================================
bool list_serial_input_options(int fd, string device)
{
   string usb1 = lookup_usb(1);
   string usb2 = lookup_usb(2);
   
   string s1 = config.reverse_lookup("serial1");
   string s2 = config.reverse_lookup("serial2");
   string su1 = config.reverse_lookup("usb1");
   string su2 = config.reverse_lookup("usb2");

   if (usb1 == "") su1 = "";
   if (usb2 == "") su2 = "";

   bool s1ok = ((s1 == "") || (s1 == device));
   bool s2ok = ((s2 == "") || (s2 == device));
   bool u1ok = ((su1 == "") || (su1 == device));
   bool u2ok = ((su2 == "") || (su2 == device));

   string u1okfail = "";
   if (su1 != "") u1okfail = " - in use by " + su1;
   if (usb1 == "") { u1okfail = " - no USB device connected to this input"; u1ok = false; }
   string u2okfail = "";
   if (su2 != "") u2okfail = " - in use by " + su2;
   if (usb2 == "") { u2okfail = " - no USB device connected to this input"; u2ok = false; }

   bool check = false;
   bool need_check = true;

   if (s1 == device) { check = " checked"; need_check = false; }
   else check = false;
   radio(fd, "Connected to Serial 1", device, "serial1", check, 1, s1ok, s1ok ? "" : (" - in use by " + s1).c_str());

   if (s2 == device) { check = " checked"; need_check = false; }
   else check = false;
   radio(fd, "Connected to Serial 2", device, "serial2", check, 1, s2ok, s2ok ? "" : (" - in use by " + s2).c_str());

   if (su1 == device) { check = " checked"; need_check = false; }
   else check = false;
   radio(fd, "Connected to USB 1", device, "usb1", check, 1, u1ok, u1ok ? "" : u1okfail.c_str());

   if (su2 == device) { check = " checked"; need_check = false; }
   else check = false;
   radio(fd, "Connected to USB 2", device, "usb2", check, 1, u2ok, u2ok ? "" : u2okfail.c_str());

   if (need_check) { check = " checked"; need_check = false; }
   else check = false;
   radio(fd, "None connected", device, "none", check, 2, true, "");

   return need_check;
}

//==============================================================================
void list_firmware_files(int fd, string title, string pattern, string mark, string model, string fw_date, string opts )
{
   // list firmware files

  wprintf(fd, "<h2>Cached Firmware Files Suitable for %s</h2>\r\n", title.c_str());

   STRINGLIST files = GetFilesInDirectory(config.lookup("userhome") + "/firmware", pattern, true);

   for (string s : files)
   {
      if (contains(mark, s) && contains(model, s))
      {
         string basename = "";
         string basedate = "202" + free_split(" 202", s);
         if (basedate != "")
            basename = s.substr(0, s.find(" 202"));

         string s_vers = fw_version_from_yyyy_mm_dd(basedate);
         
         wprintf(fd, "<p>%s <i>version</i> %s<span padding='3'></span>\r\n", basename.c_str(), s_vers.c_str());
         wprintf(fd, "<a href='delete_file?name=%s'>Delete from Cache</a><span padding='3'></span>\r\n", replace(s, ' ', '+').c_str());
         wprintf(fd, "<a href='burn_file?name=%s%s'>Install in Device</a><span padding='3'></span>\r\n",
            replace(s, ' ', '+').c_str(), opts.c_str());

         string dfw = fw_version(fw_date);
         string ffw = fw_version_from_yyyy_mm_dd(basedate);

         string comp = "(currently in the device)";
         if (newer_than(dfw, ffw)) comp = "(older than what is in device)";
         if (newer_than(ffw, dfw)) comp = "(<b>newer than what is in device</b>)";

         wprintf(fd, "%s</p>\r\n", comp.c_str());
      }
   }

   wprintf(fd, "<p><i>End of cached firmware file list</i>\r\n");
   br(fd, 1);
   wprintf(fd, " Go to the Home page for an option to update cached files from the web</p>\r\n");
}

//==============================================================================
bool send_file(int fd, char image_path[], char head[])
{
   int fdfile = open(image_path, O_RDONLY);

   if (fdfile < 0)
   {
      log_warning("Cannot Open file path : %s with error %s", image_path, strerror(errno));
      write(fd, "HTTP/1.1 404 Not Found\r\n", strlen("HTTP/1.1 404 Not Found\r\n"));

      return false;
   }

   struct stat stat_buf;  /* hold information about input file */

   write(fd, head, strlen(head));

   fstat(fdfile, &stat_buf);

   off_t total_size = stat_buf.st_size;

   off_t offset = 0;

   //   if (!other_end_closed(fd))
   //   {
   while (total_size > 0)
   {

      int done_bytes = (int)sendfile(fd, fdfile, &offset, total_size);
      total_size -= done_bytes;
   }
   //   }

      // log_event("send file: %s \n", image_path);
   ztclose(fdfile);
   return true;
}

//==============================================================================
void do_index(int fd, string path, string options)
{
   standard_header(fd, MYNAME " Home");

   br(fd, 1);

   wprintf(fd, "<a href = \"system_state?%s\">Show system state snapshot</a>\r\n", mangle_opt().c_str());
   br(fd, 2);

   wprintf(fd, "<h3>" MYNAME " Settings</h3>");

   wprintf(fd, "<a href = \"setup?%s\">Basic Set Up " MYNAME "</a>\r\n", mangle_opt().c_str());
   br(fd, 2);
   wprintf(fd, "<a href = \"logger?%s\">Set up system mission logger</a>\r\n", mangle_opt().c_str());
   br(fd, 2);
   wprintf(fd, "<a href = \"update?%s\">Update firmware cache (must be connected to internet)</a>\r\n", mangle_opt().c_str());
   br(fd, 3);

   wprintf(fd, "<a href = \"rovl?%s\">Configure USBL/ROVL</a>\r\n", mangle_opt().c_str());
   br(fd, 2);
   wprintf(fd, "<a href = \"mavlink?%s\">Add/Manage Mavlink REST Server Connection</a>\r\n", mangle_opt().c_str());
   br(fd, 2);

   wprintf(fd, "<h3>External Devices</h3>");

   wprintf(fd, "<a href = \"discover?%s\">Discover Devices on the Network</a>\r\n", mangle_opt().c_str());
   br(fd, 4);

   wprintf(fd, "<a href='restart?%s'><font size = '4'>Restart Now</font></a>\r\n", mangle_opt().c_str());
   br(fd, 3);

   wprintf(fd, "<p>This website interfaces with external devices with their own\r\n");
   wprintf(fd, "agendas. In general, avoid refreshing pages or using your browser\r\n");
   wprintf(fd, "back arrow. Clicking the Cerulean logo in the upper left corner\r\n");
   wprintf(fd, "always returns to the home page.</p>\r\n");

   wprintf(fd, "</body>");
   wprintf(fd, "</HTML>");
}


//==============================================================================
void do_submitted(int fd, string nextpage)
{
   standard_header(fd, "");

   br(fd, 1);
   wprintf(fd, "<h1>Committed!</h1>");

   wprintf(fd, "<a href=\"%s?%s\">Continue</a>\r\n", nextpage.c_str(), mangle_opt().c_str());

   wprintf(fd, "<span padding = 5></span>");
   wprintf(fd, "<a href='restart?%s'>Restart Now</a>\r\n", mangle_opt().c_str());

   br(fd, 1);

   standard_footer(fd);
}

//==============================================================================
void do_repost(int fd)
{
   standard_header(fd, "");

   br(fd, 1);
   wprintf(fd, "<h1>Repost</h1>");

   br(fd, 2);

   wprintf(fd, "It appears you or your brower are trying to repost data. We are blocking this action.");

   br(fd, 2);

   wprintf(fd, "<a href=\"%s?%s\">Continue</a>\r\n", "/", mangle_opt().c_str());

   wprintf(fd, "<span padding = 5></span>");
   wprintf(fd, "<a href='restart?%s'>Restart Now</a>\r\n", mangle_opt().c_str());

   br(fd, 1);

   standard_footer(fd);
}

//==============================================================================
void do_reboot(int fd, string path, string options)
{
   config.write_configuration();

   standard_header(fd, "Rebooting...");

   wprintf(fd, "<p>System is rebooting. Wait 30 or 45 seconds and then try to reconnect.</p>\r\n");

   wprintf(fd, "<a href=\"/?%s\">Re-connect</a>\r\n", mangle_opt().c_str());

   standard_footer(fd);

   system("sudo reboot");
}


//==============================================================================
void do_restart(int fd, string path, string options)
{
   config.write_configuration();
   standard_header(fd, "Restarting...");

   wprintf(fd, "<p>System is restarting. Wait 10 or 20 seconds and then try to reconnect.</p>\r\n");
   wprintf(fd, "<p>Do not refresh or re-submit this page or it will restart again.</p>\r\n");

   wprintf(fd, "<a href=\"/?%s\">Re-connect</a>\r\n", mangle_opt().c_str());

   system("sudo reboot");

   standard_footer(fd);

 //  pprintf(out_pipe_fd, "kill-self");
}


//==============================================================================
void do_setup(int fd, string path, string options)
{
   standard_header(fd, "Set Up " MYNAME );

   wprintf(fd, "<form action='setup' method='POST'>");

   wprintf(fd, "<input type='hidden' id='next-webpage' name='next-webpage' value='setup'>\r\n");

   string staticip = config.lookup("static-ip");
   string secondip = get_my_second_ip();
   if (secondip == "")
      secondip = "(none assigned)";
   string hostname = get_my_hostname();
   string username = config.lookup("user");
   string autoboot = config.lookup("reboot-on-change-static-ip");

   wprintf(fd, "<input type='hidden' id='old-static-ip' name='old-static-ip' value='%s'>\r\n", staticip.c_str());
   wprintf(fd, "<input type='hidden' id='old-hostname' name='old-hostname' value='%s'>\r\n", hostname.c_str());
   wprintf(fd, "<input type='hidden' id='old-username' name='old-username' value='%s'>\r\n", username.c_str());

   wprintf(fd, "<p><b><i>You should not experiment with these values. It might be hard to recover from zany values.</i></b></p>");
   br(fd, 2);

   wprintf(fd, "<label for=\"new-static-ip\">My Static IP: </label>\r\n");
   do_ip_port_address(fd, "new-static-ip", staticip, false);
   wprintf(fd, " (may force a reboot after committing)\r\n");
   br(fd, 2);

   string check = (config.lookup("reboot-on-change-static-ip") == "on") ? " checked" : "";
   wprintf(fd, "%s%s>\r\n", "<input type=\"checkbox\" name=\"reboot-on-change-static-ip\"", check.c_str());
   wprintf(fd, "<label for=\"reboot-on-change-static-ip\">Automatically reboot when static IP is changed (including remote/discover changes)</label>");
   br(fd, 2);

   wprintf(fd, "<label for=\"dhcp\">Secondary IP (from DHCP): </label>\r\n");
   wprintf(fd, "<input type='text' readonly id='dhcp' name='dhcp' value='%s' size='%d' style='text-align:center;' title='DHCP Address'>\r\n",
      secondip.c_str(), secondip.length());
   br(fd, 2);

   wprintf(fd, "<label for=\"new-hostname\">Linux Hostname/Network Name: </label>\r\n");
   wprintf(fd, "<input type='text' id='new-hostname' name='new-hostname' value='%s' size='%d' style='text-align:center;' title='Hostname' required>\r\n",
      hostname.c_str(), hostname.length());
   wprintf(fd, " (will force a reboot after committing)\r\n");
   br(fd, 2);

   wprintf(fd, "<label for=\"new-username\">Linux User Name: </label>\r\n");
   wprintf(fd, "<input type='text' id='new-username' name='new-username' value='%s' size='%d' style='text-align:center;' title='Username' required>\r\n",
      username.c_str(), username.length());
   br(fd, 2);

   checkbox(fd, "Manually set time (24-HR UTC)\r\n", "set-utc", false, 1);

   const char* labels[6] = { "Yr", "Mo", "Dy", "Hr", "Mn", "Sd" };
   const char* names[6] =
   { "time-year", "time-month", "time-day", "time-hour", "time-minute", "time-second" };
   const int mins[6] = { 1970, 1, 1, 0, 0, 0 };
   const int maxes[6] = { 2100, 12, 31, 23, 59, 59 };
   const char* titles[6] = {
      "Year, 1970-2100", "Month, 1-12", "Day, 1-31", "Hour 0-23", "Minute, 0-59", "Second, 0-59" };
   const char* spacer[6] = {" / ", " / ", " - ", " : ", " : ", ""};

   using namespace std::chrono;
   time_t ttNow = time(NULL);

   tm* ptmNow = gmtime(&ttNow);

   int values[6] = { ptmNow->tm_year + 1900, ptmNow->tm_mon + 1, ptmNow->tm_mday, 
      ptmNow->tm_hour, ptmNow->tm_min, ptmNow->tm_sec };

   for (int i = 0; i < 6; i++)
   {
      wprintf(fd, "<input type='number' id='%s' name='%s' value='%d' min='%d' max='%d' style='text-align:center;' width:2em' title='%s' required>%s\r\n",
         labels[i], names[i], values[i], mins[i], maxes[i], titles[i], spacer[i]);
   }

   br(fd, 3);

   commit_reset(fd, "Commit", true);

   wprintf(fd, "</form>");

   standard_footer(fd);

//   post_completed = false;
}


//==============================================================================
void do_setupprocessing(int fd, string path, string options)
{
   // try to prevent re-posting
   static long int seq_seen = 0;
   string seq_str = extract("seq", options, "&");
   long int seq = std::stol(seq_str);
   if (seq <= seq_seen)
   {
      do_repost(fd);
      return;
   }
   seq_seen = seq;

   string oldstaticip = extract("old-static-ip", options, "&");
   string oldhostname = extract("old-hostname", options, "&");
   string oldusername = extract("old-username", options, "&");
   string newhostname = extract("new-hostname", options, "&");
   string newusername = extract("new-username", options, "&");
   string newstaticip = ip_from_opts("new-static-ip", options);
//   post_completed = true;

   config.update("reboot-on-change-static-ip", "off");
   string autoboot = extract("reboot-on-change-static-ip", options, "&");
   config.update("reboot-on-change-static-ip", tolower(autoboot));

   config.update("user", newusername);

   // time
   string update_time = extract("set-utc", options, "&");
   if (update_time == "on")
   {
      string cmd = "sudo timedatectl set-time \"yyyy-MM-dd hh:mm:ss\"";

      cmd.replace(cmd.find("yyyy"), 4, extract("time-year", options, "&"));
      cmd.replace(cmd.find("MM"), 2, extract("time-month", options, "&"));
      cmd.replace(cmd.find("dd"), 2, extract("time-day", options, "&"));

      cmd.replace(cmd.find("hh"), 2, extract("time-hour", options, "&"));
      cmd.replace(cmd.find("mm"), 2, extract("time-minute", options, "&"));
      cmd.replace(cmd.find("ss"), 2, extract("time-second", options, "&"));

      log_event("Time reset");

      system("sudo timedatectl set-ntp 0");
      system(cmd.c_str());
   }

   bool must_reboot = false;

   if (oldhostname != newhostname)
   {
      string cmd = "hostnamectl set-hostname " + newhostname;
      string cmd2 = "sed -i.bak 's/" + (string)oldhostname.c_str() + "/" + (string)newhostname.c_str() + "/' /etc/hosts";
      sudo_mode();
      system(cmd.c_str());
      system(cmd2.c_str());
      user_mode();
      must_reboot = true;
   }

   if (must_reboot || (autoboot == "on"))
      config.write_configuration();

   // do me last in case reboot is turned on
   if (oldstaticip != newstaticip)
   {
      config.update("static-ip", newstaticip);
      write_ip_boot_script(newstaticip);
   }

   if (must_reboot)
   {
      sudo_mode();
      do_reboot(fd, path, options);
      user_mode(); // may not come back here!
   }

   string next = extract("next-webpage", options, "&");
   if (next != "")
      do_submitted(fd, next);
}

#if false
//==============================================================================
void do_gps(int fd, string path, string options)
{
   standard_header(fd, "Add/Manage USB or Serial GPS");

   wprintf(fd, "<form action='gps' method='POST'>");

   wprintf(fd, "<input type='hidden' id='next-webpage' name='next-webpage' value='gps'>\r\n");

   list_serial_input_options(fd, "gps");

   wprintf(fd, "<label for=\"gps-baud\">GPS Baud Rate: </label>\r\n");
   wprintf(fd, "<input type='number' id='gps-baud' name='gps-baud' value='%s' style='text-align:center; width:5em' required pattern='9600|115200' title='GPS baud rate, must be 9600 or 115200'>\r\n",
      config.lookup("gps-baud").c_str());
   br(fd, 2);

   checkbox(fd, "Send position to MAVLink REST server when satellites acquired",
      "gps-mavlink", (config.lookup("gps-mavlink") == "on"), 1);
   wprintf(fd, "<label for=\"gps-mav-ip\">IP for MAVLink server: </label>\r\n");

   br(fd, 2);

   checkbox(fd, "Forward GPS/GNSS Messages to UDP client",
      "gps-fwd-udp", (config.lookup("gps-fwd-udp") == "on"), 1);
   wprintf(fd, "<label for=\"gps-udp-ip\">IP for UDP client: </label>\r\n");
   do_ip_port_address(fd, "gps-udp-ip", config.lookup("gps-udp-ip"), true);

   br(fd, 2);

   checkbox(fd, "Use GPS Time to Update System Clock",
      "gps-set-time", (config.lookup("gps-set-time") == "on"), 1);

   br(fd, 2);

   wprintf(fd, "<p><b><font size='4'>Select GPS/GNSS messages to forward</font></b></p>");
   for (string msg : { (string)"gll", (string)"rmc", (string)"gga",
      (string)"vtg", (string)"gsa", (string)"gsv", (string)"other" })
   {
      string check = (config.lookup("gps-" + msg) == "on") ? " checked" : "";
      wprintf(fd, "<input type=\"checkbox\" name=\"gps-%s\" %s>\r\n", msg.c_str(), check.c_str());
      wprintf(fd, "<label for=\"gps-%s\">Forward $xx%s GPS/GNSS Messages</label>", msg.c_str(), toupper(msg).c_str());
      br(fd, 1);
   }

   br(fd, 1);

   checkbox(fd, "Also send selected messages to mission log",
      "gps-log-messages", (config.lookup("gps-log-messages") == "on"), 1);

   br(fd, 2);

   commit_reset(fd);

   wprintf(fd, "</form>");

   standard_footer(fd);

//   post_completed = false;
}
#endif

#if false
//==============================================================================
void do_bar(int fd, string path, string options)
{
   standard_header(fd, "Add/Manage Bar30/Bar100/Bar02 Driver");

   wprintf(fd, "<form action='bar' method='POST'>");

   wprintf(fd, "<input type='hidden' id='next-webpage' name='next-webpage' value='bar30'>\r\n");

   string bar = config.lookup("bar");

   wprintf(fd, "Device Type");
   br(fd, 1);
   radio(fd, "None connected", "bar", "none", ((bar == "none") || (bar == "")), 1, true, "");
   radio(fd, "Bar30", "bar", "bar30", (bar == "bar30"), 1, true, "");
   radio(fd, "Bar100", "bar", "bar100", (bar == "bar100"), 1, true, "");
   radio(fd, "Bar02", "bar", "bar02", (bar == "bar02"), 1, true, "");

   br(fd, 1);
   wprintf(fd, "Bus Connection");
   br(fd, 1);
   string iface = config.lookup("bar-i2c-port");
   radio(fd, "I2C-1 bus", "bar-i2c-port", "i2c-1", ((iface == "i2c-1") || (iface == "")), 1, true, "");
   radio(fd, "I2C-2 bus", "bar-i2c-port", "i2c-2", ((iface == "i2c-2") || (iface == "")), 1, true, "");

   br(fd, 1);

   checkbox(fd, "Send pressure and temperature to MAVLink REST server", "bar-mavlink", 
      (config.lookup("bar-mavlink") == "on"), 1);

   br(fd, 1);

   string bar_message = config.lookup("bar-message");
   wprintf(fd, "MAVLink message type (suggestion, Blue ROV BAR30 should be PRESSURE2)");
   br(fd, 1);

   radio(fd, "PRESSURE", "bar-message", "pressure", (bar_message == "pressure"), 1, true, "");
   radio(fd, "PRESSURE2", "bar-message", "pressure2", (bar_message == "pressure2"), 1, true, "");
   radio(fd, "PRESSURE3", "bar-message", "pressure3", (bar_message == "pressure3"), 1, true, "");

   br(fd, 1);

   checkbox(fd, "Forward Depth/Pressure/Temperature Messages to UDP client",
      "bar-fwd-udp", (config.lookup("bar-fwd-udp") == "on"), 1);

   br(fd, 1);
   
   wprintf(fd, "<label for=\"bar-udp-ip\">IP for UDP client: </label>\r\n");
   do_ip_port_address(fd, "bar-udp-ip", config.lookup("bar-udp-ip"), true);

   br(fd, 1);


   checkbox(fd, "Send Depth/Pressure/Temperature Messages to mission log",
      "bar-log-messages", (config.lookup("bar-log-messages") == "on"), 1);

   br(fd, 2);

   commit_reset(fd);

   wprintf(fd, "</form>");

   standard_footer(fd);

//   post_completed = false;
}
#endif


//==============================================================================

void do_logger(int fd, string path, string options)
{
//   post_completed = false;

   standard_header(fd, "Manage Mission Logger");

   br(fd, 1);

   wprintf(fd, "<form action='logger' method='POST'>");

   wprintf(fd, "<input type='hidden' id='next-webpage' name='next-webpage' value='logger'>\r\n");

   string check = "";

   checkbox(fd, "Mission logger enabled", "logger", (config.lookup("logger") == "on"), 1);   

   br(fd, 2);

   wprintf(fd, "<label for=\"logger-maxnum\">Maximum number of logfiles to keep: </label>\r\n");
   wprintf(fd, "<input type='number' id='logger-maxnum' name='logger-maxnum' value='%s' min='1' max='10' style='text-align:center; width:5em' required title='Maximum files to keep, 1..10'>\r\n",
      config.lookup("logger-maxnum").c_str());
   br(fd, 2);

   wprintf(fd, "<label for=\"logger-maxsize\">Maximum size of log file (Mbytes): </label>\r\n");
   wprintf(fd, "<input type='number' id='logger-maxsize' name='logger-maxsize' value='%s' min='1' max='1000' style='text-align:center; width:5em' required title='Maximum fies to keep, 1..10'>\r\n",
      config.lookup("logger-maxsize").c_str());
   br(fd, 2);

   commit_reset(fd);

   wprintf(fd, "</form>");

   br(fd, 1);

   wprintf(fd, "<p><b>Temp log files on system:</p></b>\r\n");

   // now list out logfiles
   string cmd = "ls -lh " + config.lookup("webhome") + "/logs/" LOGFILE_ROOT "*.log";
   FILE* f = popen(cmd.c_str(), "r");
   if (f == NULL)
   {
      log_severe("Unable to open ls process in logger page");
   }
   else
   {
      char line[132];

      while (fgets(line, sizeof(line), f) != NULL)
      {
         string sline = line;
         if (sline.find("total") != string::npos)
            continue;

         string p = head_of(sline, " "); // take off permissions
         string d = head_of(sline, " "); // take off file/dir digit
         string o = head_of(sline, " "); // take off owner
         string g = head_of(sline, " "); // take off group
         string s = head_of(sline, " ");  // take off size

         string fn = sline;
         string m = head_of(fn, " ");  // take off month
         string t = head_of(fn, " ");  // take off time or year
         string n = head_of(fn, " "); // get the base filename into fn


         fn = fn.substr(fn.find_last_of("/") + 1, string::npos); // strip off the path
         fn = fn.substr(0, fn.find_first_of("\n")); // strip off the trailing newline(s)

         string pathname = "/logs/" + fn;

         string time = m + "-" + t + "_" + n;

         string display = fn + " - " + m + " " + t + " " + n + " - " + s;

         wprintf(fd, "<form action='savelog' method='POST' target='hiddenFrame' onsubmit='setTimeout(function() { location.reload(); }, 100);'>%s\r\n", display.c_str());

         wprintf(fd, "<span padding=2></span><a href='delete?file=%s'>delete</a>\r\n", fn.c_str());
         wprintf(fd, "<span padding=2></span><a href='%s' download='%s'>download</a>\r\n", pathname.c_str(), fn.c_str());
   
         // Dumb but couldn't find a simpler way to avoid redirecting page with save button
         wprintf(fd, "<iframe name='hiddenFrame' style='display:none;'></iframe>\r\n");

         //TODO: Allowing saving while writing to file?
         wprintf(fd, "<input type='hidden' name='fn' value='%s' />\r\n", fn.c_str());
         wprintf(fd, "<input type='hidden' name='info' value='%s' />\r\n", time.c_str());
         wprintf(fd, "<span padding=2></span><input type='submit' name='save' value='Save' />");
         wprintf(fd, "</form>\r\n");

      }
   }
   wprintf(fd, "<p>End of list</b>\r\n");

   wprintf(fd, "<p><b>Saved log files:</p></b>\r\n");

   // now list out logfiles
   cmd = "ls -lh " + config.lookup("webhome") + "/savedLogs/*.log";
   f = popen(cmd.c_str(), "r");
   if (f == NULL)
   {
       log_severe("Unable to open ls process in logger page");
   }
   else
   {
       char line[132];

       while (fgets(line, sizeof(line), f) != NULL)
       {
           string sline = line;
           if (sline.find("total") != string::npos)
               continue;

           string p = head_of(sline, " "); // take off permissions
           string d = head_of(sline, " "); // take off file/dir digit
           string o = head_of(sline, " "); // take off owner
           string g = head_of(sline, " "); // take off group
           string s = head_of(sline, " ");  // take off size

           string fn = sline;
           string m = head_of(fn, " ");  // take off month
           string t = head_of(fn, " ");  // take off time or year
           string n = head_of(fn, " "); // get the base filename into fn


           fn = fn.substr(fn.find_last_of("/") + 1, string::npos); // strip off the path
           fn = fn.substr(0, fn.find_first_of("\n")); // strip off the trailing newline(s)

           string display = fn + " - " + s;

           wprintf(fd, "<form action='replaylog' method='POST' target='hiddenFrame'>%s\r\n", display.c_str());


           // Dumb but couldn't find a simpler way to avoid redirecting page with save button
           wprintf(fd, "<iframe name='hiddenFrame' style='display:none;'></iframe>\r\n");

           wprintf(fd, "<input type='hidden' name='fn' value='%s' />\r\n", fn.c_str());
           wprintf(fd, "<span padding=2></span><input type='submit' name='replay' value='Replay' />");
           wprintf(fd, "</form>\r\n");

       }
   }
   wprintf(fd, "<p>End of list</b>\r\n");

   standard_footer(fd);
}

//==============================================================================
void do_optionsinput(int fd, string path, string options)
{
   string next = extract("next-webpage", options, "&");

   static long int seq_seen = 0;
   string seq_str = extract("seq", options, "&");
   long int seq = std::stol(seq_str);
   if (seq <= seq_seen)
   {
      do_repost(fd);
      return;
   }
   seq_seen = seq;

   if (next == "gps")
      for (auto& text : { "gps-mavlink", "gps-fwd-udp", "gps-fwd-tcp", "gps-gll", "gps-rmc", "gps-gga", "gps-vtg", 
         "gps-gsa", "gps-gsv", "gps-other", "gps-set-time", "gps-log-messages" })
         config.update(text, "off");

   if (next == "ping2")
      for (auto& text : { "ping2-mavlink", "ping2-fwd-udp", "ping2-fwd-tcp", "ping2-log-messages", "ping2-bridge" })
         config.update(text, "off");

   if(next == "bar")
      for (auto& text : { "bar-mavlink", "bar-fwd-udp", "bar-log-messages" })
         config.update(text, "off");

   if (next == "logger")
      for (auto& text : { "logger" })
         config.update(text, "off");

   if (next == "mavlink")
      for (auto& text : { "mavlink" })
         config.update(text, "off");

   if (next == "replicate")
      for (auto& text : {"replicate-1-hardware-handshake"})
         config.update(text, "off");

   string token = "";

   while ( (token = head_of(options, "& \t\r\n")) != "")
   {
      string key = head_of(token, "=");
      string val = token;

      if (key.find(".0") != string::npos)
      {
         string id = head_of(key, ".");
         val = ip_from_opts(id, options, val);
         key = id;
      }
      else if (key.find(".") != string::npos)
      {
         continue;
      }
      else if (key == "next-webpage")
      {
         continue;
      }

      config.update(key, val);
   }

   log_event("option processing completed");

   config.write_configuration();

   if (next != "")
      do_submitted(fd, next);

   return;
}

//==============================================================================
std::vector<string> targets = {};

void do_discover(int fd, string path, string options)
{
   standard_header(fd, "Discover Devices on the Network");

   wprintf(fd, "<p><b>Looking for devices...</b></p>");

   FILE* f;

   string cmd = "sudo " + config.lookup("userhome") + "/ot discover 0 1";

   sudo_mode();
   f = popen(cmd.c_str(), "r"); 
   user_mode();

   targets.clear();

   if (f == NULL)
   {
      wprintf(fd, "<p>Unable to open discovery process</p>");
   }
   else
   {
      char line[1024];

      while (!feof(f))
      {
         char *n = fgets(line, sizeof(line), f);

         if (n == NULL)
            continue;

         string sline = line;

         if (contains("Zombie", sline))
            continue;

         sline = head_of(sline, "\r\n");

         if (sline == "")
            continue;

//         string sender = head_of(sline, ",");

         if (sline.find("poll completed") != string::npos)
            break;

         if (toupper(sline).find("ERROR") != string::npos)
         {
            wprintf(fd, "<p>%s</p>\r\n", sline.c_str());
            continue;
         }

         head_of(sline, ":"); // eat "discovered:"
         string device = head_of(sline, "|"); head_of(sline, "|");
         string manufacturer = head_of(sline, "|"); head_of(sline, "|"); 
         string mac_address = head_of(sline, "|"); head_of(sline, "|"); head_of(mac_address, "-");
         string ip_address = head_of(sline, "|"); head_of(sline, "|"); head_of(ip_address, "-");
         string port_address = head_of(sline, "|"); head_of(sline, "|"); head_of(port_address, "-");
         string version = head_of(sline, "|"); head_of(sline, "|"); head_of(version, "-");

         targets.insert(targets.end(), device + "," + manufacturer + "," + mac_address + "," + ip_address
            + "," + port_address + "," + version);

         size_t pos = targets.size();

         wprintf(fd, "<p><b>%s</b><span padding = 5></span>IP: %s:%s<span padding = 5></span>MAC: %s<span padding = 5></span>Version: %s\r\n", device.c_str(),
            ip_address.c_str(), port_address.c_str(), mac_address.c_str(), version.c_str());

         if (tolower(manufacturer).find("cerulean") != string::npos)
         {
            wprintf(fd, "<span padding = 5></span>\r\n");
            wprintf(fd, "<a href='chgremoteip?line=%d&%s'>Change IP Address</a>\r\n", pos, mangle_opt().c_str());

            if ((tolower(device).find("tracker") != string::npos)|| (tolower(device).find("dvl") != string::npos))
            {
               wprintf(fd, "<span padding = 5></span>\r\n");
               wprintf(fd, "<a href='setremoteparams?line=%d&%s'>Firmware or Configure</a>\r\n", pos, mangle_opt().c_str());
            }
            wprintf(fd, "</p>");
         }
      }
      fclose(f);
   }

   wprintf(fd, "<p><b>No more responders<//b></p>");

   standard_footer(fd);
}


//==============================================================================
void do_chgremoteip(int fd, string path, string options)
{
   head_of(options, "=");
   unsigned int pos = stoi(options);
   if ((pos < 1) || (pos > targets.size()))
      return;

   string spec = targets[pos - 1];

   string device = head_of(spec, ","); 
   string manufacturer = head_of(spec, ",");
   string mac_address = head_of(spec, ",");
   string ip_address = head_of(spec, ",");
   string port = head_of(spec, ",");

   standard_header(fd, "Change a Network Device IP address");

   br(fd, 2);

   wprintf(fd, "<form action='newipaddress' method='POST'>\r\n");

   wprintf(fd, "<p><b>%s</b> at MAC %s</label></p>\r\n",device.c_str(), mac_address.c_str());
   wprintf(fd, "<label for=\"newipaddress\">New IP address: </label>\r\n");
   do_ip_port_address(fd, "newipaddress", ip_address, false);

   br(fd, 2);

   commit_reset(fd);

   wprintf(fd, "<input type='hidden' id='next-webpage' name='next-webpage' value='discover'>\r\n");
   wprintf(fd, "<input type='hidden' id='mac-address' name='mac-address' value='%s'>\r\n", mac_address.c_str());
   wprintf(fd, "<input type='hidden' id='port-address' name='port-address' value='%s'>\r\n", port.c_str());

   wprintf(fd, "</form>\r\n");

   br(fd, 2);

   wprintf(fd, "<p>After committing, the network device will probably reboot and be off the net "
      "for 30 seconds or so. Wait and then try refreshing the browser to find it at its new address.</p>\r\n");

   standard_footer(fd);

//   post_completed = false;
}


//==============================================================================
void do_newipaddress(int fd, string path, string options)
{
   string ip = ip_from_opts("newipaddress", options);
   if (ip != "")
   {
      string command = "SET-IP-FOR-MAC-ADDRESS MAC:" + extract("mac-address", options, "&") + " IP:" + ip;

      string port = extract("port-address", options, "&");

      // send to both possiblities
      send_message_to_udp_port(-1, command, "255.255.255.255", "30303");
      send_message_to_udp_port(-1, command, "255.255.255.255", port);
   }

//   post_completed = true;

   string next = extract("next-webpage", options, "&");
   if (next != "")
      do_submitted(fd, next);
}

//==============================================================================
void do_genericDVLparms(int fd, string message, string& master_date, string& channel_date)
{
   while (true)
   {
      string keyval = head_of(message, ",");
      if ((keyval[0] == '*') || (keyval == ""))
         return;

      string key = head_of(keyval, "=");
      string val = keyval;
      if ((key == "") || (val == ""))
         return;

      key = replace(key, ' ', '-'); // don't need no stinkin' spaces

      const string ips_withports[] = { "Host-IP-address", "MAVlink-IP-address" };
      const string ips_withoutports[] = { "Fallback-IP-address", "Static-IP-address" };
      const string on_off[] = { "DVKFC-messages", "DVPDL-messages", "DVPDX-messages", "MAVlink-send-distance", "MAVlink-auto-origin", "Pool-Mode",
      "GPRMC-messages", "DVEXT-messages", "Freeform-messages", "DVKFA-messages", "DVKFB-messages", "Retweet-GPS", "Retweet-IMU"};
      const string readonly[] = { "Model", "MAVlink-status", "Firmware", "DVL-Channel" };
      const string scalars[] = { "Speed-of-sound", "Velocity-adjustment", "Magnetic-declination" };
      const string eulers[] = { "Sensor-Orientation" };
      const string text[] = { "Baud-rate", "Coordinate-frame", "Heading-offset" };

      if ((key != "Coordinate-frame") && (key != "Heading-offset"))
         wprintf(fd, "<p>%s<span padding = 5></span>\r\n", key.c_str());

      if (std::find(ips_withports, ips_withports + COUNT(ips_withports), key) != ips_withports + COUNT(ips_withports))
      {
         do_ip_port_address(fd, key, val, true);
      }
      else if (std::find(ips_withoutports, ips_withoutports + COUNT(ips_withoutports), key) != ips_withoutports + COUNT(ips_withoutports))
      {
         do_ip_port_address(fd, key, val, false);
      }
      else if (std::find(on_off, on_off + COUNT(on_off), key) != on_off + COUNT(on_off))
      {
         do_on_off_value(fd, key, toupper(val));
      }
      else if (std::find(readonly, readonly + COUNT(readonly), key) != readonly + COUNT(readonly))
      {
         if (key == "Firmware") master_date = val;
         if (key == "DVL-Channel") channel_date = val;

         wprintf(fd, "<input type='text' id='skip' name='%s' value='%s' size='%d' style='text-align:center;' readonly>\r\n",
            key.c_str(), val.c_str(), val.length());
      }
      else if (std::find(eulers, eulers + COUNT(eulers), key) != eulers + COUNT(eulers))
      {
         do_euler(fd, key, val);
      }
      else if (std::find(scalars, scalars + COUNT(scalars), key) != scalars + COUNT(scalars))
      {
         string min = "0"; string max = "0"; 
         if (key == "Speed-of-sound") { min = "700"; max = "2000"; }
         if (key == "Velocity-adjustment") { min = "0.5"; max = "1.5"; }
         if (key == "Magnetic-declination") { min = "-10"; max = "10"; }
         if (key == "Heading-offset") { min = "-10"; max = "10"; }
         string title = "Must be between " + min + " and " + max;
         wprintf(fd, "<input type='number' id='%s' name='%s' value='%s' size='%d' style='text-align:center;' min='%s' max='%s' title='%s' step='0.0001'>\r\n",
            key.c_str(), key.c_str(), val.c_str(), val.length(), min.c_str(), max.c_str(), title.c_str());
      }
      else if (std::find(text, text + COUNT(text), key) != text + COUNT(text))
      {
         if (key == "Baud-rate")
         {
            string title = "Must be 115200 or 921600";
            wprintf(fd, "<input type='text' id='%s' name='%s' value='%s' size='%d' style='text-align:center;' title='%s' required pattern='115200|921600'>\r\n",
               key.c_str(), key.c_str(), val.c_str(), val.length(), title.c_str());
         }
         else if ((key == "Coordinate-frame") || (key == "Heading-offset"))
         {
            // this is deprecated
         }
      }
      else
      {
         wprintf(fd, "<b>Unsupported tuple </b> {%s} {%s}\r\n", key.c_str(), val.c_str());
      }

      wprintf(fd, "<p>\r\n");
   }
}

// string saved_dvnvm = "";

//==============================================================================
void do_setremoteparams(int fd, string path, string options)
{
   head_of(options, "=");
   unsigned int pos = stoi(options);
   if ((pos < 1) || (pos > targets.size()))
      return;

   string spec = targets[pos - 1];

   string device = head_of(spec, ",");
   string manufacturer = head_of(spec, ",");
   string mac_address = head_of(spec, ",");
   string ip_address = head_of(spec, ",");
   string port_address = head_of(spec, ",");
   string fw_rev = head_of(spec, ",");

   standard_header(fd, "Set Parameters for Network Device " + device + " at MAC " + mac_address);

   wprintf(fd, "<form action='setremoteparams' method='POST'>\r\n");

   wprintf(fd, "<input type='hidden' id='ip_address' name='ip_address' value='%s'>\r\n", ip_address.c_str());
   wprintf(fd, "<input type='hidden' id='port_address' name='port_address' value='%s'>\r\n", port_address.c_str());

   t650_reset_status();
   send_port_message(PORTS::TRACKER650, "?\n");
   delay(1000);

   string master_date = "";
   string channel_date = "";

   if (t650_valid)
   {
      while (t650_dvnvm != "")
      {
         string message = head_of(t650_dvnvm, "\n");
         string message_type = head_of(message, ",");
         if (message_type == "$DVNVM")
         {
//            saved_dvnvm = message;
            do_genericDVLparms(fd, message, master_date, channel_date);
         }
      }
   }
   else
   {
      wprintf(fd, "Could not establish connection to Tracker 650.\r\n");
      br(fd, 2);
   }

   commit_reset(fd);

   wprintf(fd, "</form>\r\n");

   wprintf(fd, " After committing, the network device may also reboot by itself.\r\n");
   br(fd, 1);

   if (tolower(device) == "tracker 650")
   {
      string dvlopts = "&ip=" + ip_address + "&port=" + port_address;
      list_firmware_files(fd, "Tracker 650 Master", "Tracker-650-Master", "Master", "Tracker-650", master_date, "&device=tracker650" + dvlopts);
   }
   else if ((tolower(device.substr(0, 6)) == "dvl-75") || (contains(tolower(device.substr(0, 6)), "sonar dvl")))
   {
      string dvlopts = "&ip=" + ip_address + "&port=" + port_address + "&mask=";
      list_firmware_files(fd, "DVL-75 Master", "DVL-Master", "Master", "DVL", master_date, "&device=dvlmaster" + dvlopts + "1,0,0,0,0");
      list_firmware_files(fd, "DVL-75 Channel", "DVL-Channel", "Channel", "DVL", free_split( "f/w:",channel_date), "&device=dvlchannel" + dvlopts + "0,1,1,1,1");
   }
   else
   {
   }

   standard_footer(fd);
}

#if false
//==============================================================================
void do_ping2(int fd, string path, string options)
{
   standard_header(fd, "Add/Manage Serial Ping2 Device");

   wprintf(fd, "<form action='ping2' method='POST'>");

   wprintf(fd, "<input type='hidden' id='next-webpage' name='next-webpage' value='ping2'>\r\n");

   list_serial_input_options(fd, "ping2");

   wprintf(fd, "<label for=\"ping2-baud\">Ping2 Baud Rate: </label>\r\n");
   wprintf(fd, "<input type='number' id='ping2-baud' name='ping2-baud' value='%s' style='text-align:center; width:5em' required pattern='9600|115200' title='Ping2 baud rate, must be 9600 or 115200'>\r\n",
      config.lookup("ping2-baud").c_str());
   wprintf(fd, " - recommend 115200");
   br(fd, 2);

   checkbox(fd, "Expose a serial to UDP transparent bridge", "ping2-bridge", config.lookup("ping2-bridge") == "on", 1);
   wprintf(fd, "<label for=\"ping2-port\">UDP port number: </label>\r\n");
   wprintf(fd, "<input type='number' id='ping2-port' name='ping2-port' value='%s' min='1024' max='49151' style='text-align:center; width:5em' required title='1024 to 49151 inclusive'>\r\n",
      config.lookup("ping2-port").c_str());
   br(fd, 2);

   wprintf(fd, "<b><i>Note: If the transparent UDP bridge is enabled, the options below become irrelevant.</i></b>");
   br(fd, 2);

#if false // in autonomous mode, no obvious place to send the profile.
   wprintf(fd, "Message requested from Ping device");
   br(fd, 1);
   string msg = config.lookup("ping2-message");
   radio(fd, "Get altitude only", "ping2-message", "altitude", ((msg == "altitude") || (msg == "")), 1, true, "");
   radio(fd, "Get altitude and profile", "ping2-message", "profile", (msg == "profile"), 1, true, "");
   br(fd, 1);
#endif

#if false // range only works in manual mode
   wprintf(fd, "<label for=\"ping2-min\">Minimum scan start distance: </label>\r\n");
   wprintf(fd, "<input type='number' id='ping2-min' name='ping2-min' value='%s' min='0.3' max='100.0' step='0.1' style='text-align:center; width:5em' required title='Minimum Scan Start Dist (m)'>\r\n",
      config.lookup("ping2-min").c_str());
   br(fd, 1);

   wprintf(fd, "<label for=\"ping2-len\">Scan length: </label>\r\n");
   wprintf(fd, "<input type='number' id='ping2-len' name='ping2-len' value='%s' min='1.0' max='100.0' step='0.1' style='text-align:center; width:5em' required title='Scan Length (m)'>\r\n",
      config.lookup("ping2-len").c_str());
   br(fd, 2);
#endif

   wprintf(fd, "<label for=\"ping2-rate\">Minimum ping period (seconds): </label>\r\n");
   wprintf(fd, "<input type='number' id='ping2-rate' name='ping2-rate' value='%s' min='0.25' max='5.0' step='0.01' style='text-align:center; width:5em' required title='0.25 to 5.0 inclusive'>\r\n",
      config.lookup("ping2-rate").c_str());
   br(fd, 2);


   checkbox(fd, "Send depth to MAVLink REST server when bottom acquired",
      "ping2-mavlink", (config.lookup("ping2-mavlink") == "on"), 1);
   br(fd, 1);

   checkbox(fd, "Forward Depth Messages (in NMEA 'xxDBT' sentence format) to UDP client",
      "ping2-fwd-udp", (config.lookup("ping2-fwd-udp") == "on"), 1);
   wprintf(fd, "<label for=\"ping2-udp-ip\">IP for UDP client: </label>\r\n");
   do_ip_port_address(fd, "ping2-udp-ip", config.lookup("ping2-udp-ip"), true);
   br(fd, 1);

   wprintf(fd, "<label for=\"ping2-nmea-prefix\">NMEA label prefix for 'xx': </label>\r\n");
   wprintf(fd, "<input type='text' id='ping2-nmea-prefix' name='ping2-nmea-prefix' value='%s' style='text-align:center; width:5em' required pattern='[A-Z][A-Z]' title='xxDBT xx prefix, must be two UC letters'>\r\n",
      config.lookup("ping2-nmea-prefix").c_str());
   wprintf(fd, " - recommend GP if unsure");
   br(fd, 2);

   checkbox(fd, "Also send xxDBT messages to mission log",
      "ping2-log-messages", (config.lookup("ping2-log-messages") == "on"), 1);

   br(fd, 2);

   commit_reset(fd);

   wprintf(fd, "</form>");

   standard_footer(fd);

//   post_completed = false;
}
#endif

#if false
//==============================================================================
void do_s500(int fd, string path, string options)
{
   standard_header(fd, "Add/Manage Serial/USB S500 SBES Device");

   wprintf(fd, "<form action='s500' method='POST'>");

   wprintf(fd, "<input type='hidden' id='next-webpage' name='next-webpage' value='s500'>\r\n");

   list_serial_input_options(fd, "s500");

   wprintf(fd, "<label for=\"s500-baud\">S500 Baud Rate: </label>\r\n");
   wprintf(fd, "<input type='number' id='s500-baud' name='s500-baud' value='%s' style='text-align:center; width:5em' required pattern='9600|115200' title='s500 baud rate, must be 9600 or 115200'>\r\n",
      config.lookup("s500-baud").c_str());
   wprintf(fd, " - recommend 115200");
   br(fd, 2);

   checkbox(fd, "Expose a serial/USB to UDP transparent bridge", "s500-bridge", config.lookup("s500-bridge") == "on", 1);
   wprintf(fd, "<label for=\"s500-port\">UDP port number: </label>\r\n");
   wprintf(fd, "<input type='number' id='s500-port' name='s500-port' value='%s' min='1024' max='49151' style='text-align:center; width:5em' required title='1024 to 49151 inclusive'>\r\n",
      config.lookup("s500-port").c_str());
   br(fd, 2);

   wprintf(fd, "<b><i>Note: If the transparent UDP bridge is enabled, the options below become irrelevant.</i></b>");
   br(fd, 2);

#if false // in autonomous mode, no obvious place to send the profile.
   wprintf(fd, "Message requested from Ping device");
   br(fd, 1);
   string msg = config.lookup("s500-message");
   radio(fd, "Get altitude only", "s500-message", "altitude", ((msg == "altitude") || (msg == "")), 1, true, "");
   radio(fd, "Get altitude and profile", "s500-message", "profile", (msg == "profile"), 1, true, "");
   br(fd, 1);
#endif

#if false // range only works in manual mode
   wprintf(fd, "<label for=\"s500-min\">Minimum scan start distance: </label>\r\n");
   wprintf(fd, "<input type='number' id='s500-min' name='s500-min' value='%s' min='0.3' max='100.0' step='0.1' style='text-align:center; width:5em' required title='Minimum Scan Start Dist (m)'>\r\n",
      config.lookup("s500-min").c_str());
   br(fd, 1);

   wprintf(fd, "<label for=\"s500-len\">Scan length: </label>\r\n");
   wprintf(fd, "<input type='number' id='s500-len' name='s500-len' value='%s' min='1.0' max='100.0' step='0.1' style='text-align:center; width:5em' required title='Scan Length (m)'>\r\n",
      config.lookup("s500-len").c_str());
   br(fd, 2);
#endif

   wprintf(fd, "<label for=\"s500-rate\">Minimum ping period (seconds): </label>\r\n");
   wprintf(fd, "<input type='number' id='s500-rate' name='s500-rate' value='%s' min='0.25' max='5.0' step='0.01' style='text-align:center; width:5em' required title='0.25 to 5.0 inclusive'>\r\n",
      config.lookup("s500-rate").c_str());
   br(fd, 2);

   checkbox(fd, "Send depth to MAVLink REST server when bottom acquired",
      "s500-mavlink", (config.lookup("s500-mavlink") == "on"), 1);
   br(fd, 1);

   checkbox(fd, "Forward Depth Messages (in NMEA 'xxDBT' sentence format) to UDP client",
      "s500-fwd-udp", (config.lookup("s500-fwd-udp") == "on"), 1);
   wprintf(fd, "<label for=\"s500-udp-ip\">IP for UDP client: </label>\r\n");
   do_ip_port_address(fd, "s500-udp-ip", config.lookup("s500-udp-ip"), true);
   br(fd, 1);

   wprintf(fd, "<label for=\"s500-nmea-prefix\">NMEA label prefix for 'xx': </label>\r\n");
   wprintf(fd, "<input type='text' id='s500-nmea-prefix' name='s500-nmea-prefix' value='%s' style='text-align:center; width:5em' required pattern='[A-Z][A-Z]' title='xxDBT xx prefix, must be two UC letters'>\r\n",
      config.lookup("s500-nmea-prefix").c_str());
   wprintf(fd, " - recommend GP if unsure");
   br(fd, 2);

   checkbox(fd, "Also send xxDBT messages to mission log",
      "s500-log-messages", (config.lookup("s500-log-messages") == "on"), 1);

   br(fd, 2);

   commit_reset(fd);

   wprintf(fd, "</form>");

   standard_footer(fd);

//   post_completed = false;
}
#endif

//==============================================================================
void do_mavlink(int fd, string path, string options)
{
   standard_header(fd, "Add/Manage Mavlink REST Server Connection");

   wprintf(fd, "<form action='mavlink' method='POST'>");

   br(fd, 1);

   wprintf(fd, "<input type='hidden' id='next-webpage' name='next-webpage' value='mavlink'>\r\n");

   checkbox(fd, "Enable MAVLink REST server interface",
      "mavlink", (config.lookup("mavlink") == "on"), 1);
   wprintf(fd, "<label for=\"mavlink-ip\">IP for MAVLink server: </label>\r\n");
   do_ip_port_address(fd, "mavlink-ip", config.lookup("mavlink-ip"), true);

   br(fd, 1);

   wprintf(fd, "<label for=\"mavlink-sysid\">MAVLink system ID: </label>\r\n");
   wprintf(fd, "<input type='number' id='mavlink-sysid' name='mavlink-sysid' value='%s' min='0' max='255' style='text-align:center; width:5em' required title='MAVLink System ID: 0..255, try 254 if unsure'>\r\n",
      config.lookup("mavlink-sysid").c_str());

   br(fd, 3);

   commit_reset(fd);

   wprintf(fd, "</form>");

   standard_footer(fd);

//   post_completed = false;
}

//==============================================================================
void do_savelog(int fd, string path, string options)
{
    string webHome = config.lookup("webhome");
    string fn = extract("fn", options, "&");
    string newName = extract("info", options, "&");


    string encodedColon = "%3A";

    size_t pos = newName.find(encodedColon);
    if (pos != std::string::npos) {
        newName.replace(pos, encodedColon.length(), ".");
    }

    newName = "OT_" + newName;
    
    string sourceFile = webHome + "/logs/" + fn;
    string destinationFile = webHome + "/savedLogs/" + newName + ".log";

	try
    {
		std::ifstream src(sourceFile, std::ios::binary);
		if (!src) 
        {
			throw std::runtime_error("Failed to open source file: " + sourceFile);
		}

		std::ofstream dst(destinationFile, std::ios::binary);
		if (!dst) 
        {
			throw std::runtime_error("Failed to open destination file: " + destinationFile);
		}

		// Copy contents from source to destination
		dst << src.rdbuf();

        src.close();
        dst.close();

	}
	catch (const std::exception& e) 
    {
        log_severe("Failure saving log. Error is: %s", e.what());
	}
}

//==============================================================================
void do_replaylog(int fd, string path, string options) {
    string fn = extract("fn", options, "&");

    string logPath = config.lookup("webhome") + "/savedLogs/" + fn;

    LogSimulator logSim;
    logSim.runSimulation(logPath);

}


//==============================================================================
struct option_command {
   string option;
   string command;
   string typeis;
};

//==============================================================================
struct option_command genericDVLopts[] =
{
   {"MAVlink-IP-address",    "MAVLINK-ADDRESS",         "ip5"},
   {"Fallback-IP-address",   "FALLBACK-ADDRESS",        "ip4"},
   {"Static-IP-address",     "IP-ADDRESS",              "ip4",},
   {"DVKFC-messages",        "SEND-DVKFC",              "bool"},
   {"DVPDL-messages",        "SEND-DVPDL",              "bool"},
   {"DVPDX-messages",        "SEND-DVPDX",              "bool"},
   {"MAVlink-send-distance", "SEND-MAVLINK-DISTANCE",   "bool"},
   {"MAVlink-auto-origin",   "AUTO-MAVLINK-ORIGIN",     "bool"},
   {"Pool-Mode",             "SET-POOL-MODE",           "bool"},
   {"Speed-of-sound",        "SET-SPEED-OF-SOUND",      "scalar"},
   {"Velocity-adjustment",   "SET-VELOCITY-ADJUSTMENT", "scalar"},
   {"Sensor-Orientation",    "SET-SENSOR-ORIENTATION",  "euler"},
   {"Host-IP-address",       "HOST-ADDRESS",            "ip5"},
   {"Baud-rate",             "BAUD-RATE",               "scalar"},
   {"DVEXT-messages",        "SEND-DVEXT",              "bool"},
   {"Freeform-messages",     "SEND-FREEFORM",           "bool"},
   {"DVKFA-messages",        "SEND-DVKFA",              "bool"},
   {"DVKFB-messages",        "SEND-DVKFB",              "bool"},
   {"Magnetic-declination",  "DECLINATION",             "scalar"},
   {"Retweet-GPS",           "RETWEET-GPS",             "bool"},
   {"Retweet-IMU",           "RETWEET-IMU",             "bool"},
};

//==============================================================================
void do_dvlparameters(int fd, string path, string options)
{
   string prior = t650_dvnvm;

   // try to prevent re-posting
   static long int seq_seen = 0;
   string seq_str = extract("seq", options, "&");
   long int seq = std::stol(seq_str);
   if (seq <= seq_seen)
   {
      do_repost(fd);
      return;
   }
   seq_seen = seq;

   options = replace(options, '+', ' ');

   standard_header(fd, "Updating Device Options");

   for (int i = 0; i < (int)COUNT(genericDVLopts); i++)
   {
      string param = "";
      string option = genericDVLopts[i].option;
      string command = genericDVLopts[i].command;
      string typeis = genericDVLopts[i].typeis;

      if (typeis == "ip5")
      {
         param = extract(option + ".0", options, "&") +
            "." + extract(option + ".1", options, "&") +
            "." + extract(option + ".2", options, "&") +
            "." + extract(option + ".3", options, "&") +
            ":" + extract(option + ".4", options, "&");
      }
      else if (typeis == "ip4")
      {
         param = extract(option + ".0", options, "&") +
            "." + extract(option + ".1", options, "&") +
            "." + extract(option + ".2", options, "&") +
            "." + extract(option + ".3", options, "&");
      }
      else if ((typeis == "bool") || (typeis == "scalar"))
      {
         param = extract(option, options, "&");
      }
      else if (typeis == "euler")
      {
         param = extract(option + ".0", options, "&") +
            "," + extract(option + ".1", options, "&") +
            "," + extract(option + ".2", options, "&");
      }

      prior = replace(prior, ' ', '-');
      string was = extract(option, prior, ",");

      if (option == "Sensor-Orientation")
         was = replace(was, ';', ',');

      if (was != param)
      {
         string ip_address = extract("ip_address", options, "&");
         string port_address = extract("port_address", options, "&");
         string request = command + " " + param;

         send_message_to_udp_port(-1, request, ip_address, port_address);

         wprintf(fd, "<p>Updating %s to %s</p>\r\n", command.c_str(), param.c_str());

         delay(5000);
      }
   }

   wprintf(fd, "<p>Completed updates.</p>\r\n");

   br(fd, 1);

   wprintf(fd, "<b><a href='/discover?%s'>Continue...</a></b>\r\n", mangle_opt().c_str());

   standard_footer(fd);
}


//==============================================================================
void do_rovl(int fd, string path, string options)
{
   standard_header(fd, "Configure the USBL/ROVL Device");

   wprintf(fd, "<form action='rovl2' method='POST'>\r\n");

   wprintf(fd, "Querying configuration...  ");

   br(fd, 2);

   rovl_reset_status();

   send_port_message(PORTS::ROVL_RX, "?\n");
   delay(1500);

   wprintf(fd, "USBL/ROVL systems check:\r\n");
   br(fd, 1);

   wprintf(fd, "- USBL/ROVL subsystem %s\r\n", rovl_valid_rx ? "is OK" : "Failed to connect");
   br(fd, 2);

   if (rovl_valid_rx)
   {
      wprintf(fd, "USBL/ROVL subsystem firmware is %s\r\n", rovl_firmware_rx.c_str());
      br(fd, 1);

      wprintf(fd, "<label for='new-rovl-sos'>Using speed of sound: </label>\r\n");
      wprintf(fd, "<input type=number id='new-rovl-sos' name='new-rovl-sos' value='%1.0f' style='text-align:center; "
         "width:2em required min='1000' max='2200' title='1000 .. 2200'>\r\n", rovl_speed_of_sound);

      wprintf(fd, "<input type='hidden' id='old-rovl-sos' name='old-rovl-sos' value='%1.0f'>\r\n", rovl_speed_of_sound);
      br(fd, 2);

      wprintf(fd, "<label for='new-rovl-sos'>Using magnetic declination: </label>\r\n");
      wprintf(fd, "<input type=number id='new-rovl-declination' name='new-rovl-declination' value='%1.1f' style='text-align:center; "
         "width:2em required min='1000' max='2200'  step=0.1 title='1000 .. 2200'>\r\n", rovl_magnetic_declination);

      wprintf(fd, "<input type='hidden' id='old-rovl-declination' name='old-rovl-declination' value='%1.1f'>\r\n", rovl_magnetic_declination);
      br(fd, 2);

      bool checked = (rovl_polling_ids_mask != 0);
      checkbox(fd, "Use IDs when polling", "new-rovl-use-ids", checked, 1);
      wprintf(fd, "<input type='hidden' id='old-rovl-use-ids' name='old-rovl-use-ids' value='%s'>\r\n", checked ? "on" : "off");

      wprintf(fd, "<label for='new-rovl-poll-mask'>Polling bitmask: </label>\r\n");
      wprintf(fd, "<input type=number id='new-rovl-poll-mask' name='new-rovl-poll-mask' value='%04X' style='text-align:center; "
         "width:2em required min='1' max='65535' title='must be non-zero'>\r\n", rovl_polling_ids_mask);
      br(fd, 1);

      wprintf(fd, "Polling ID sequence: %s\r\n", rovl_polling_ids_human_readable.c_str());
      br(fd, 2);
   }
   else
   {
      wprintf(fd, "Unable to connect to USBL/ROVL\r\n");
      br(fd, 2);
   }

   br(fd, 2);

   commit_reset(fd, "Commit");

   wprintf(fd, "</form>\r\n");

   standard_footer(fd);

   br(fd, 2);

//   wprintf(fd, "<h2>USBL/ROVL Firmware</h2>\r\n");

   list_firmware_files(fd, "USBL/ROVL", "ROVL-RX", "ROVL-RX", "Mk III", rovl_firmware_rx, "&device=mkIII");


//   post_completed = false;
}

//==============================================================================
void do_rovl2(int fd, string path, string options)
{
   // try to prevent re-posting
   static long int seq_seen = 0;
   string seq_str = extract("seq", options, "&");
   long int seq = std::stol(seq_str);
   if (seq <= seq_seen)
   {
      do_repost(fd);
      return;
   }
   seq_seen = seq;

   standard_header(fd, "Configure the USBL/ROVL Device");

   wprintf(fd, "Updating configuration...\r\n");
   br(fd, 2);

   string old_sos = extract("old-rovl-sos", options, "&");
   string new_sos = extract("ew-rovl-sos", options, "&");
   if (old_sos != new_sos)
   {
      wprintf(fd, "Updating speed of sound to %s\r\n", new_sos.c_str());
      br(fd, 2);
      send_port_message(PORTS::ROVL_RX, "X" + new_sos + "\n");
      delay(3000);
   }

   string old_use_ids = extract("old-rovl-use-ids", options, "&");
   string new_use_ids = extract("new-rovl-use-ids", options, "&");
   string new_poll_mask = extract("new-rovl-poll-mask", options, "&");
   if (new_use_ids == "") new_use_ids = "off";
   if (new_use_ids == "on")
   {
      wprintf(fd, "Updating IDs to %s\r\n", new_poll_mask.c_str());
      br(fd, 2);
      send_port_message(PORTS::ROVL_RX, "L" + new_poll_mask + "\n");
      delay(3000);
   }
   else if (old_use_ids != new_use_ids)
   {
      wprintf(fd, "Turning IDs off\r\n");
      br(fd, 2);
      send_port_message(PORTS::ROVL_RX, "LOFF\n");
      delay(3000);
   }

   string old_declination = extract("old-rovl-declination", options, "&");
   string new_declination = extract("ew-rovl-declination", options, "&");
   if (old_declination != new_declination)
   {
      wprintf(fd, "Updating speed of sound to %s\r\n", new_declination.c_str());
      br(fd, 2);
      send_port_message(PORTS::ROVL_RX, "Z" + new_declination + "\n");
      delay(3000);
   }

   wprintf(fd, "Configuration completed\r\n");
   br(fd, 2);

   wprintf(fd, "<b><a href='/?%s'>Continue...</a></b>\r\n", mangle_opt().c_str());

   standard_footer(fd);
}

#if false
//==============================================================================
struct option_command genericROVLopts[] =
{
   {"operating-channel-t",  "I",        "value"},
   {"operating-channel-r",  "I",        "value"},
   {"transmit-id",          "L",        "value"},
   {"speed-of-sound",       "Z",        "value"},
   {"mag-declination",      "J",        "value"},
   {"imu-mode",             "",         "value"},
   {"receive-polling",      "L",        "value"},
};

//==============================================================================
void do_rovl3(int fd, string path, string options)
{
   static long int seq_seen = 0;
   string seq_str = extract("seq", options, "&");
   long int seq = std::stol(seq_str);
   if (seq <= seq_seen)
   {
      do_repost(fd);
      return;
   }
   seq_seen = seq;
   int rfd = -1;

   string filename = "/dev/" + extract("filename", options, "&");
   int baud_rate = std::stoi(extract("baud-rate", options, "&"));

   sudo_mode();
   rfd = openSerialDevice(filename, true, baud_rate, true, false, false, true);
   user_mode();

   standard_header(fd, "Updating Device Options");

   for (int i = 0; i < (int)COUNT(genericROVLopts); i++)
   {
      string param = "";
      string option = genericROVLopts[i].option;
      string command = genericROVLopts[i].command;

      param = extract("new-" + option, options, "&");
      string was = extract("old-" + option, options, "&");

      if (was != param)
      {
         if (option == "imu-mode")
         {
            if (param == "COMPASS")
               param = "T COMPASS\r\nYCIMU OFF";
            else if (param == "NDOF")
               param = "T NDOF\r\nYCIMU OFF";
            else if (param == "CIMU")
               param = "YCIMU ON";

         }

         string request = "\n" + command + param + "\n";

         write(rfd, request.c_str(), request.length());

         wprintf(fd, "<p>Updating %s to %s</p>\r\n", option.c_str(), param.c_str());

         delay(2000);
      }
   }

   if (rfd != -1)
      close(rfd);

   //   post_completed = true;

   wprintf(fd, "<p>Completed updates.</p>\r\n");
   br(fd, 1);

   wprintf(fd, "<b><a href='/?%s'>Continue...</a></b>\r\n", mangle_opt().c_str());

   standard_footer(fd);
}

#endif
//==============================================================================
void do_burn_file(int fd, string path, string options)
{
   standard_header(fd, "Firmware Installation");

   string device = extract("device", options, "&");
   string name = replace(extract("name", options, "&"), '+', ' ');
   string ip = extract("ip", options, "&");
   string port = extract("port", options, "&");
   string mask = extract("mask", options, "&");


//   bipipe_control_block bcb;


   if (contains("mkII", device))
   {
      // doing an ROVL
      wprintf(fd, "<p>Writing to a USBL/ROVL via serial port</p>");
      bool result = STbootloader_main(config.lookup("userhome") + "/firmware/" + name, fd);

      if( result )
         wprintf(fd, "<p>Firmware update successful</p>");
      else
         wprintf(fd, "<p>Firmware update failed</p>");
   }

   else if (device == "dvlmaster")

   {
      wprintf(fd, "<p>Writing to a DVL Master CPU via Ethernet</p>");

//      bipipe(bcb, "r", arg0name, 5, "dvlfw", (config.lookup("userhome") + "/firmware/" + name).c_str(),
//         ip.c_str(), port.c_str(), "1,0,0,0,0");
   }
   else if( device == "dvlchannel")
   {
      wprintf(fd, "<p>Writing to four DVL Channel CPUs via Ethernet</p>");

//      bipipe(bcb, "r", arg0name, 5, "dvlfw", (config.lookup("userhome") + "/firmware/" + name).c_str(),
//         ip.c_str(), port.c_str(), "0,1,1,1,1");
   }
   else if (device == "tracker650")
   {
      wprintf(fd, "<p>Writing to Tracker 650 via Ethernet</p>");
      bool result = fwdvl_main(config.lookup("userhome") + "/firmware/" + name, fd);

      if (result)
         wprintf(fd, "<p>Firmware update successful</p>");
      else
         wprintf(fd, "<p>Firmware update failed</p>");

//      bipipe(bcb, "r", arg0name, 5, "dvlfw", (config.lookup("userhome") + "/firmware/" + name).c_str(),
//         ip.c_str(), port.c_str(), "1,0,0,0,0");
   }
   else
   {
      log_severe("Unknown device type '%s'", device.c_str());
      standard_footer(fd);
      return;
   }

//   LINE_BUFFER lb(bcb.my_read_fd, true);

   while (true)
   {
//      string line = lb.next_line();

//      if (line == "")
//         break;
//      wprintf(fd, "<p>%s</p>", line.c_str());
//      if (contains("completed", line) || contains("failed", line))
         break;
   }

//   system(("sudo kill " + std::to_string(bcb.p)).c_str()); // try to get rid of the zombies

   wprintf(fd, "<p>The device may take an additional 30 seconds or so to complete its reboot before it registers on the network again</p>");
   wprintf(fd, "<a href='/?%s'><font size = '4'>Continue...</font></a>\r\n", mangle_opt().c_str());

   standard_footer(fd);
}

//==============================================================================
void do_update(int fd, string path, string optiopns)
{
   standard_header(fd, "Update Firmware Cache");

   wprintf(fd, "<p>Checking internet connection...</p>\r\n");

   bool pinged = false;

   if (ping("github.com", get_my_second_ip()))
   {
      pinged = true;
   }
   else
   {
      pinged = ping("github.com", get_my_static_ip());
   }

   if (!pinged)
   {
      wprintf(fd, "<p>Cannot reach the remote website. You need an internet connection\r\n");
      wprintf(fd, "to make this work. Please resolve the issue and try again. You will probably need to\r\n");
      wprintf(fd, "reboot after making the connection to allow " MYNAME " to be assigned an\r\n");
      wprintf(fd, "address on the network.</p>\r\n");
      br(fd, 1);
      wprintf(fd, "<a href='/?%s'><font size = '4'>Continue...</font></a>\r\n", mangle_opt().c_str());
      br(fd, 1);

      wprintf(fd, "<a href='reboot?%s'><font size = '4'>Reboot now...</font></a>\r\n", mangle_opt().c_str());
   }
   else
   {

      wprintf(fd, "<p>Found internet.</p>\r\n");

      br(fd);

      wprintf(fd, "<form action='update2' method='POST'>");

      wprintf(fd, "<p>Select file categories for updating:</p>\r\n");

      checkbox(fd, "Update " MYNAME " drivers and files", "omnitrack-files", true, 1);
      checkbox(fd, "Update USBL/ROVL files", "rovl-files", true, 1);
      checkbox(fd, "Update Tracker 650 files", "tracker650-files", true, 1);
      checkbox(fd, "Update DVL-75 files", "dvl75-files", true, 1);

      br(fd, 2);

      commit_reset(fd, "Search & Download");
   }

   standard_footer(fd);

//   post_completed = false;
}

//==============================================================================
void do_update2(int fd, string path, string options)
{
   // try to prevent re-posting
   static long int seq_seen = 0;
   string seq_str = extract("seq", options, "&");
   long int seq = std::stol(seq_str);
   if (seq <= seq_seen)
   {
      do_repost(fd);
      return;
   }
   seq_seen = seq;

   standard_header(fd, "Updating Cached Files");

   update_firmware(fd, path, options);

   wprintf(fd, "<p>Completed updates.</p>\r\n");

   br(fd, 1);

   wprintf(fd, "<b><a href='/?%s'>Continue...</a></b>\r\n", mangle_opt().c_str());

   standard_footer(fd);

//   post_completed = true;
}

#if true
//==============================================================================
void do_sys_state(int fd, string path, string options)
{
   // try to prevent re-posting
   static long int seq_seen = 0;
   string seq_str = extract("seq", options, "&");
   long int seq = std::stol(seq_str);
   if (seq <= seq_seen)
   {
      do_repost(fd);
      return;
   }
   seq_seen = seq;

   standard_header(fd, "System State");
   br(fd, 2);

   wprintf(fd, "MAVLlink/Ardusub: %s\r\n", mav_comm_active ? "Comm connected" : "Comm Not Connected");
   if (mav_comm_active)
   {
      if (mav_global_origin_valid)
         wprintf(fd, "<span padding=2></span>(Global Origin valid, lat=%1.10f, lon=%1.10f)\r\n",
            mav_global_origin_lat, mav_global_origin_lon);
      else
         wprintf(fd, "<span padding=2></span>(Global Origin not valid)\r\n");
   }
   br(fd, 1);

   wprintf(fd, "GNSS: %s\r\n", gnss_comm_active ? "Comm connected" : "Comm Not Connected");
   br(fd, 1);

   if (gnss_comm_active)
   {
      wprintf(fd, "<span padding=2></span>GNSS Heading Valid: %s\r\n", gnss_heading_valid ? "True" : "False");
      br(fd, 1);

      wprintf(fd, "<span padding=2></span>Position Valid: %s\r\n", gnss_position_valid ? "True" : "False");
      br(fd, 1);

      if (gnss_heading_valid)
      {
         wprintf(fd, "<span padding=2></span>Mag Valid: %s\r\n", gnss_mag_valid ? "True" : "False");
         br(fd, 1);

         wprintf(fd, "<span padding=2></span>Calibrating: %s\r\n", gnss_calibrating ? "True" : "False");
         br(fd, 1);

         wprintf(fd, "<span padding=2></span>Calibration Valid: %s\r\n", gnss_calibration_valid ? "True" : "False");
         br(fd, 1);

         wprintf(fd, "<span padding=2></span>Base OK: %s\r\n", gnss_base_ok ? "True" : "False");
         br(fd, 1);

         wprintf(fd, "<span padding=2></span>Rover OK: %s\r\n", gnss_rover_ok ? "True" : "False");
         br(fd, 1);

         wprintf(fd, "<span padding=2></span>Base Lock: %s\r\n", gnss_base_lock ? "True" : "False");
         br(fd, 1);

         wprintf(fd, "<span padding=2></span>Rover Lock: %s\r\n", gnss_rover_lock ? "True" : "False");
         br(fd, 1);
      }
   }
  

   wprintf(fd, "USBL/ROVL: %s\r\n", rovl_comm_active ? "Comm connected" : "Comm Not Connected");
   if (rovl_comm_active)
      wprintf(fd, "<span padding=2></span>%s\r\n", (rovl_usrth.ping_group_valid) ? "(Receiving from ROV TX" : "(Not receiving from ROV TX)");
   br(fd, 1);

   wprintf(fd, "Tracker 650/DVL: %s\r\n", t650_comm_active ? "Comm connected" : "Comm Not Connected");
   if (t650_comm_active)
      wprintf(fd, "<span padding=2></span>%s\r\n", (t650_dvpdx.confidence > 0) ? "(Has bottom lock)" : "(No bottom lock)");
   br(fd, 1);

   //==
 //  wprintf(fd, "dummy\r\n");
 //  br(fd, 1);



   br(fd, 2);
   wprintf(fd, "<a href = 'system_state?%s'>Refresh Values</a>\r\n", mangle_opt().c_str());
   standard_footer(fd);
}
#endif

//==============================================================================

namespace {
   typedef void routingfn(int fd, string path, string options);

   struct rr
   {
      string request;
      routingfn* destination_get;
      routingfn* destination_put;
   }
   req_router[] =
   {
      {"/index.html",      do_index,      do_index },
      {"/",                do_index,      do_index },
      {"/system_state",    do_sys_state,  do_sys_state },
      {"/rovl",            do_rovl,       do_rovl},
      {"/rovl2",           do_rovl2,      do_rovl2},
      {"/burn_file",       do_burn_file,  do_burn_file},
      {"/logger",          do_logger,     do_optionsinput},
      {"/discover",        do_discover,   do_discover},
      {"/chgremoteip",     do_chgremoteip,do_newipaddress},
      {"/setup",           do_setup,      do_setupprocessing},
      {"/setremoteparams", do_setremoteparams, do_dvlparameters },
      {"/newipaddress",    do_newipaddress,do_newipaddress},
      {"/reboot",          do_reboot,     do_reboot},
      {"/restart",         do_restart,    do_restart},
      {"/update",          do_update,     do_update},
      {"/update2",         do_update2,    do_update2},
      {"/mavlink",         do_mavlink,    do_optionsinput},
      {"/savelog",         do_savelog,    do_savelog},
      {"/replaylog",       do_replaylog,  do_replaylog},
   };
} // namespace



void do_file(int fd, string inpath, string request_type, string options)
{
   string path = head_of(inpath, "?");

   if( options == "")
      options = inpath; // the leftover

   for( struct rr r : req_router )
      if (r.request == path)
      {
         if (request_type == "GET")
            r.destination_get(fd, path, options);
         else if (request_type == "POST")
            r.destination_put(fd, path, options);
         else
            log_severe("Routing error, request = %s", request_type.c_str());
         return;
      }

   if ((path == "/api/v1/about"))
   {
      string path_head = slash + path;
      if (!send_file(fd, (char*)path_head.c_str(), 
         (char *)"HTTP/1.1 200 Ok\r\nContent-Type: application/json\r\n\r\n"))
      {
         wprintf(fd, "HTTP/1.1 404 Not Found\r\n");
         return;
      }
   }
   else // tgry for a file from disk
   {
      string path_head = slash + path;
      if (!send_file(fd, (char*)path_head.c_str(), 
         (char *)"Content-Type: text/plain\r\n\r\n"))
      {
         wprintf(fd, "HTTP/1.1 404 Not Found\r\n");
         // pmessage("File sent: %s\n", path.c_str());
         return;
      }
   }
}


const std::map<string, string> extensions = {
   {"", ""},
   {"ico", "Content-Type: image/ico\r\n\r\n"},
   {"jpg", "Content-Type: image/jpeg\r\n\r\n"},
   {"ttf","Content-Type: font/ttf\r\n\r\n"},
   {"js","Content-Type: text/javascript\r\n\r\n"},
   {"css","Content-Type: text/css\r\n\r\n"},
   {"wof","Content-Type: font/woff\r\n\r\n"},
   {"m3u8","Content-Type: application/vnd.apple.mpegurl\r\n\r\n"},
   {"ts","Content-Type: video/mp2t\r\n\r\n"},
   {"log","\r\n"},
};

//==============================================================================
bool web_main()
{
   log_event("Web server starting");
   prctl(PR_SET_NAME, "webZombie");

   struct sigaction sa = { SIG_IGN };
   sigaction(SIGPIPE, &sa, NULL); // ignore SIGPIPE

   slash = (config.lookup("userhome") + "/web");

   int server_fd, new_socket;
   struct sockaddr_in address;
   int addrlen = sizeof(address);

   // Creating socket file descriptor
   if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
   {
      log_severe("Web socket failure. Error is: %s", strerror(errno));
      exit(EXIT_FAILURE);
   }

   int optval = 1;
   setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

   address.sin_family = AF_INET;
   address.sin_addr.s_addr = INADDR_ANY;
   address.sin_port = htons(PORT);

   memset(address.sin_zero, '\0', sizeof address.sin_zero);

   sudo_mode();

   if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0)
   {
      log_severe("Bind web failure! Error is: %s", strerror(errno));
      ztclose(server_fd);
      exit(EXIT_FAILURE);
   }

   user_mode();

   if (listen(server_fd, 10) < 0)
   {
      log_severe("Listen web failure! Error is: %s", strerror(errno));
      exit(EXIT_FAILURE);
   }
 
   while (true)
   {
      if ((new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0)
      {
         log_severe("Accept web connection failure! Error is: %s", strerror(errno));
         exit(EXIT_FAILURE);
      }

      if (new_socket == 0)
      {
         log_severe("newsocket is %d", new_socket);
         continue; // leave assigned
      }

      /* Set the timeout option active */
      int optval = 1;
      socklen_t optlen = sizeof(optval);
      if (setsockopt(new_socket, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0)
         log_warning("setsockopt()");

      char buffer[3000] = { 0 };
      int offset = 0;
      int valread = 0;

      while (true)
      {
         int thistime = (int)read(new_socket, buffer + offset, sizeof(buffer) - 1 - offset);
         if (thistime < 1)
            break;

         set_fd_nonblocking(new_socket);
         offset += thistime;
         valread += thistime;
         delay(200);
      }

      if (valread == 0)
         break;

      buffer[valread] = '\0';

      string sbuffer(buffer, valread);

      string method = head_of(sbuffer, " \r\n\t");

      string path = head_of(sbuffer, " \r\n\t");

      sbuffer = sbuffer.substr(sbuffer.find("\r\n\r\n") + 4); // sbuffer should be the body, if any

      if (!contains("/about", path)) // ignore annoying request from Mavlink
         log_event("Client %s asked for: %s", address_image((struct sockaddr_in*)&address).c_str(), path.c_str());

      string ext3 = path;
      string ext2 = head_of(ext3, ".?");
      string ext = ext3;

      string reply_header = http_ok_header;

      if (method == "GET")
      {

         string content_type = "";

         try {
            content_type = extensions.at(tolower(ext));
         }
         catch (...) {}

         if (content_type != "")
         {
            string path_head = slash;
            path_head += path;
            reply_header += content_type;
            send_file(new_socket, (char*)path_head.c_str(), (char*)reply_header.c_str());
         }
         else
         {
            //send other file 
            do_file(new_socket, path, "GET", "");
         }
         //log_event("\n------------------Server sent----------------------------------------------------\n");
      }
      else if (method == "POST")
      {
         do_file(new_socket, path, "POST", sbuffer);
      }
      else
      {
         // error unimplemented request type
      }

      ztclose(new_socket);
   }
   ztclose(new_socket);
   return false;
}


