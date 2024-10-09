#include "utilities.h"
#include "configuration.h"

// Linux headers
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

extern int main(int argc, char* argv[]);

using std::string;

TIMING mission_start_time = Clock().now();

//==========================================================================================
string keyvalkey(const string s, const string splitter)
{
   size_t e = s.find_first_of(splitter);
   if (e == string::npos)
      return s;

   return s.substr(0, e);
}

//==========================================================================================
string keyvalvalue(const string s, const string splitter)
{
   size_t e = s.find_first_of(splitter);
   if (e == string::npos)
      return "";

   return s.substr(e + 1, s.size());
}

#if false
//============================================================================================================
LINE_BUFFER::LINE_BUFFER(int fd, bool ignore_blanks)
{
   m_ignore_blanks = ignore_blanks;
   m_fd = fd;
}

//============================================================================================================
LINE_BUFFER::~LINE_BUFFER()
{
   // close(m_fd);
}

//============================================================================================================
void LINE_BUFFER::flush()
{
   char buffer[256];

   while (next_line((char *)buffer, sizeof(buffer)) > 0)
      ;
}

//============================================================================================================
int LINE_BUFFER::next_line(char* buffer, int max_length)
{
   if (m_fd < 0)
      return 0;

   for (;;)
   {
      if (m_staging_count <= 0)
      {
         // get another batch of characters
         m_staging_count = read(m_fd, m_staging, sizeof(m_staging));
         if (m_staging_count < 0)
         {
            m_staging_count = 0;
            if (errno == 11)
               return 0;   // resource temporarily not available
            log_warning("Line read attempt failed in next_line: %s", strerror(errno));
            return 0;
         }

         m_staging_ptr = 0; // reset to start of what we just read
      }

      if (m_staging_count == 0)
         return 0;

      if (m_staging_count < 0)
         log_warning("Bad value in m_staging_count = %d", m_staging_count);

      char c = m_staging[m_staging_ptr++];
      m_staging_count--;

      // ignore embedded nulls
      if (c == '\0')
         continue;

      // ignore c/r
      if (c == '\r')
         continue;

      // ignore blank lines
      if (m_ignore_blanks)
         if ((m_ptr == 0) && (c == '\n'))
            continue;

      if (m_ptr == (max_length - 1))
      {
         m_buffer[m_ptr] = '\0';
         m_ptr = 0;
         strcpy((char*)buffer, m_buffer);
         return max_length;
      }

      if (c == '\n')
      {
         m_buffer[m_ptr] = '\0';
         int rv = m_ptr;
         m_ptr = 0;
         strcpy((char*)buffer, m_buffer);
         return rv;
      }

      m_buffer[m_ptr++] = c;
   }
}

//============================================================================================================
string LINE_BUFFER::next_line()
{
   char temp[1600];

   int c = next_line(temp, sizeof(temp));

   if (c == 0)
      return "";

   string result = temp;

   return result;
}


#endif

//==========================================================================================
/* Returns a list of files in a directory (except the ones that begin with a dot) */
STRINGLIST GetFilesInDirectory(string directory, string pattern, bool justname)
{
   DIR* dir;
   class dirent* ent;
   class stat st;

   STRINGLIST result; // = new STRINGLIST(10);

   dir = opendir(directory.c_str());
   if (dir == 0)
      return result;

   while ((ent = readdir(dir)) != NULL)
   {
      const string file_name = ent->d_name;

      string full_file_name = directory + "/" + file_name;

      if (file_name[0] == '.')
         continue;

      if (stat(full_file_name.c_str(), &st) == -1)
         continue;

      const bool is_directory = (st.st_mode & S_IFDIR) != 0;

      if (is_directory)
         continue;

      string starting = file_name.substr(0, pattern.size());

      if (starting != pattern)
         continue;

      if( justname )
         result.push_back(file_name);
      else
         result.push_back(full_file_name);
   }
   closedir(dir);
   return result;
} // GetFilesInDirectory


//==========================================================================================
/* Returns a list of files in a directory (except the ones that begin with a dot) */
void GetFilesInDirectory(list<string>& result, string directory, string pattern, bool justname)
{
   DIR* dir;
   class dirent* ent;
   class stat st;

   dir = opendir(directory.c_str());
   if (dir == 0)
      return;

   while ((ent = readdir(dir)) != NULL)
   {
      const string file_name = ent->d_name;

      string full_file_name = directory + "/" + file_name;

      if (file_name[0] == '.')
         continue;

      if (stat(full_file_name.c_str(), &st) == -1)
         continue;

      const bool is_directory = (st.st_mode & S_IFDIR) != 0;

      if (is_directory)
         continue;

      string starting = file_name.substr(0, pattern.size());

      if (starting != pattern)
         continue;

      if (justname)
         result.emplace_back(file_name);
      else
         result.emplace_back(full_file_name);
   }
   closedir(dir);
   return;
} // GetFilesInDirectory



//==========================================================================================
namespace {
   const string months[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
} // namespace

bool later(int y1, int m1, int d1, int y2, int m2, int d2)
{
   if (y1 > y2)
      return true;
   if (y1 < y2)
      return false;

   if (m1 > m2)
      return true;
   if (m1 < m2)
      return false;

   if (d1 > d2)
      return true;
   if (d1 < d2)
      return false;

   return true;
}


//==========================================================================================
string fw_version(string date)
{
   char month[10];
   int day, year;

   int one, two, three;

   if(sscanf(date.c_str(), "%d.%d.%d ", &one, &two, &three) == 3)
   {
      return std::to_string(one) + "." + std::to_string(two) + "." + std::to_string(three);
   }

   if (sscanf(date.c_str(), "%3c %2d %4d ", month, &day, &year) != 3)
   {
      log_severe("F/W version error scanning date");
      return "";
   }

   int i = 0;
   for (; i < 12; i++)
      if (months[i] == (string)month)
         break;

   if (i == 12)
      return "";

   return std::to_string(year - 2022) + "." + std::to_string(i + 1) + "." + std::to_string(day);
}


//==========================================================================================
string fw_version_from_yyyy_mm_dd(string date)
{
   int year = std::stoi(head_of(date, "-"));
   int month = std::stoi(head_of(date, "-"));
   int day = std::stoi(head_of(date, "-"));

   return std::to_string(year - 2022) + "." + std::to_string(month) + "." + std::to_string(day);
}


//==========================================================================================
bool newer_than(string v1, string v2)
{
   string y1 = head_of(v1, ".");
   string m1 = head_of(v1, ".");
   string d1 = head_of(v1, ".");

   string y2 = head_of(v2, ".");
   string m2 = head_of(v2, ".");
   string d2 = head_of(v2, ".");

   if ((y1 == "") || (y2 == "") || (m1 == "") || (m2 == "") || (d1 == "") || (d2 == ""))
      return false;

   if (std::stoi(y1) > std::stoi(y2)) return true;
   if (std::stoi(y1) < std::stoi(y2)) return false;

   if (std::stoi(m1) > std::stoi(m2)) return true;
   if (std::stoi(m1) < std::stoi(m2)) return false;

   if (std::stoi(d1) > std::stoi(d2)) return true;
   
   return false;
}

//==========================================================================================
int openSerialDevice(string filename, bool raw_mode, int baud_rate, 
   bool line_mode, bool odd_parity, bool useCTSRTS, bool noblock)
{
   int serial_port = open(filename.c_str(), O_RDWR );

   // Check for errors
   if (serial_port < 0)
   {
      log_severe("Error in tty file open %s: %s", filename.c_str(), strerror(errno));
      return -1;
   }

   tcflush(serial_port, TCIOFLUSH);

   struct termios tty;

   // Read in existing settings, and handle any error
   // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
   // must have been initialized with a call to tcgetattr() overwise behavior
   // is undefined
   if (tcgetattr(serial_port, &tty) != 0)
   {
      log_severe("Error %i from tcgetattr: %s", errno, strerror(errno));
      ztclose(serial_port);
      return -1;
   }

   if (odd_parity)
   {
      tty.c_cflag |= PARENB; // Set parity bit, enabling parity (not common)
      tty.c_cflag &= ~PARODD; // even parity
      tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
   }
   else
   {
      tty.c_cflag &= ~PARENB; // Reset parity bit, disabling parity
      tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
   }

   tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
   //   tty.c_cflag |= CS5; // 5 bits per byte
   //   tty.c_cflag |= CS6; // 6 bits per byte
   //   tty.c_cflag |= CS7; // 7 bits per byte
   tty.c_cflag |= CS8; // 8 bits per byte
   tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
   tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

   if (line_mode)
      tty.c_lflag |= ICANON; // !!!! ICANON sets line by line input mode !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   else
      tty.c_lflag &= ~ICANON; // !!!! ICANON sets line by line input mode !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

   tty.c_lflag &= ~ECHO; // Disable echo
   tty.c_lflag &= ~ECHOE; // Disable erasure
   tty.c_lflag &= ~ECHONL; // Disable new-line echo
   tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
   tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
   if (raw_mode) // this actually always sets raw mode regardless of input
   {
      tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
   }
   else
   {
      tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR); // Disable any special handling of received bytes ex newline
   }
   tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
   tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

   if (noblock)
   {
      tty.c_cc[VTIME] = 0;    // (in deciseconds), 0 = no wait
      set_fd_nonblocking(serial_port);
   }
   else
   {
      tty.c_cc[VTIME] = 255;    // (in deciseconds), 0 = no wait
   }

   tty.c_cc[VMIN] = 1;

   if (useCTSRTS)
      tty.c_cflag |= CRTSCTS; // Enable RTS/CTS hardware flow control (less common)
   else
      tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)

   // Save tty settings, also checking for error
   if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      log_severe("Error %i from tcsetattr: %s", errno, strerror(errno));
      ztclose(serial_port);
      return -1;
   }

   int os = cfsetospeed(&tty, baud_rate); // B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
   int is = cfsetispeed(&tty, baud_rate); // 0 == same as output

   UNUSED(is);
   UNUSED(os);   // unused

   if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      log_severe("Error %i from tcsetattr: %s", errno, strerror(errno));
      ztclose(serial_port);
      return -1;
   }

   tcgetattr(serial_port, &tty);

   return serial_port;
}


//============================================================================================================
bool setSerialDevice(int fd, string baud_rate, string data_bits, string parity, string stop_bits) 
{
   struct termios tty;

   // Read in existing settings, and handle any error
   // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
   // must have been initialized with a call to tcgetattr() overwise behavior
   // is undefined
   if (tcgetattr(fd, &tty) != 0)
   {
      log_severe("Error %i from tcgetattr: %s", errno, strerror(errno));
      return false;
   }

   if (parity == "even")
   {
      tty.c_cflag |= PARENB; // Set parity bit, enabling parity (not common)
      tty.c_cflag &= ~PARODD; // even parity
   }
   else if( parity == "none")
   {
      tty.c_cflag &= ~PARENB; // Reset parity bit, disabling parity
   }
   else if (parity == "even")
   {
      tty.c_cflag |= PARENB;  // Set parity bit, enabling parity (not common)
      tty.c_cflag |= PARODD;   // odd parity
   }
   else
   {
      log_severe("Error in parity spec: %s", parity.c_str());
      return false;
   }

   if (stop_bits == "1")
   {
      tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
   }
   else if (stop_bits == "2")
   {
      tty.c_cflag |= CSTOPB; // Set stop field, two stop bits used in communication (less common)
   }
   else
   {
      log_severe("Error in stop bits spec: %s", parity.c_str());
      return false;
   }

   tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
   if (data_bits == "8")
   {
      tty.c_cflag |= CS8; // 8 bits per byte
   }
   else if (data_bits == "7")
   {
      tty.c_cflag |= CS7; // 7 bits per byte
   }

   // Save tty settings, also checking for error
   if (tcsetattr(fd, TCSANOW, &tty) != 0) 
   {
      log_severe("Error %i from tcsetattr: %s", errno, strerror(errno));
      return false;
   }

   int ibaud_rate = B9600;
   if (baud_rate == "9600")
      ibaud_rate = B9600;
   else if (baud_rate == "19200")
      ibaud_rate = B19200;
   else if (baud_rate == "38400")
      ibaud_rate = B38400;
   else if (baud_rate == "57600")
      ibaud_rate = B57600;
   else if (baud_rate == "115200")
      ibaud_rate = B115200;
   else if (baud_rate == "460800")
      ibaud_rate = B460800; 
   else if (baud_rate == "921600")
      ibaud_rate = B921600;

   int os = cfsetospeed(&tty, ibaud_rate); // B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
   int is = cfsetispeed(&tty, ibaud_rate); // 0 == same as output

   UNUSED(is);
   UNUSED(os);   // unused

   if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      log_severe("Error %i from tcsetattr: %s", errno, strerror(errno));
      return false;
   }

   tcgetattr(fd, &tty);

   return true;
}


#if true
//============================================================================================================
STRINGLIST* split(const string s, list<char> delimiters)
{
   STRINGLIST* result = new STRINGLIST();

   size_t ssize = s.size();
   string ss = s;

   // first replace all the delimiters with nulls
   for (list<char>::iterator it = delimiters.begin(); it != delimiters.end(); it++)
   {
      for (int i = 0; i < (int)ssize; i++)
         if (ss[i] == *it)
            ss[i] = '\0';
   }

   int start = 0;
   int end = 0;
   for (end = start; (end < (int)ssize); end++)
   {
      if (ss[end] == '\0')
      {
         string t = ss.substr(start, end - start);
         start = end + 1;
         result->push_back(t);
      }
   }
   string t = ss.substr(start, end - start);
   result->push_back(t);
   return result;
}
#endif

void delay(int milliseconds)
{
   //   if (milliseconds > 1000)
   //      milliseconds = 10;

   std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}



// =========================================================================
string time_image(string format)
{
   std::time_t t = std::time(nullptr);
   char time_buffer[100];
   strftime(time_buffer, sizeof(time_buffer), format.c_str(), std::localtime(&t));
   return time_buffer;
}


// =========================================================================
string address_image(struct sockaddr_in* addr)
{
   uint32_t ip = (uint32_t)ntohl(addr->sin_addr.s_addr);
   uint16_t port = (uint16_t)ntohs(addr->sin_port);

   string result = std::to_string((ip >> 24) & 0xff) + "." +
      std::to_string((ip >> 16) & 0xff) + "." +
      std::to_string((ip >> 8) & 0xff) + "." +
      std::to_string((ip >> 0) & 0xff) + ":" +
      std::to_string(port);

   return result;
}

// =========================================================================
string mission_time()
{
   double t = elapsed(mission_start_time);

   char temp[25];

   snprintf(temp, sizeof(temp), "%02.0f:%02.0f:%02.0f.%03.0f", t / 3600.0, fmod(t / 60.0, 60.0), fmod(t, 60), (t - trunc(t)) * 1000.0);

   string time_buffer = temp;

   return time_buffer;
}

// =========================================================================
string lookup_usb(int number)
{
   FILE* f;

   static char response[132] = { '\0' };

   if (response[0] == '\0')
   {

      string cmd = "ls --format=across /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | grep '/dev'";

      f = popen(cmd.c_str(), "r"); /* Unix */
      if (f == NULL)
      {
         log_severe("Unable to open ls process");
         return "";
      }

      char ans[132];
      char* n = fgets(ans, sizeof(ans), f);

      pclose(f);

      if (n == NULL)
         return "";

      strcpy(response, ans);
   }

   char fn1[80] = { '\0' }, fn2[80] = { '\0' }, fn3[80] = { '\0' }, fn4[80] = { '\0' };

   int n = sscanf(response, " %s %s %s %s ", fn1, fn2, fn3, fn4);
   string result = "";

   if (n == 0)
      return "";

   if ((n > 0) && (number == 1)) result = (string)fn1;
   if ((n > 0) && (number == 2)) result = (string)fn2;
   if ((n > 0) && (number == 3)) result = (string)fn3;

   string final = head_of(result, "\n");

   return final;
}

// =========================================================================
string lookup_serial(int number)
{
   if (number == 1)
      return "/dev/ttyAMA3";
   else if (number == 2)
      return "/dev/ttyAMA4";
   return "error in lookup";
}

// =========================================================================
bool ping(const string adress, const string ip_from)
{
   FILE* f;
   int ch;

   string cmd = "ping -c 1 2> /dev/null -I " + ip_from + " " + adress;

   f = popen(cmd.c_str(),"r"); /* Unix */
   if (f == NULL)
   {
      log_severe("Unable to open ping process");
      return false;
   }

   string result = "";

   while ((ch = fgetc(f)) != EOF)
      result += (char)ch;
   pclose(f);

   char* p = (char *)result.c_str();
   for (int i = 0; i < 4; i++)
   {
      while ((*p != '\0') && (*p != '\n'))
      {
         if (*p == '\0')
            return false;
         p++;
      }
      p++;
   }

   // here p should point at the x transmitted, x received part of the ping return

   int t = 0, r = 0;

   int n = sscanf(p, "%d packets transmitted, %d received", &t, &r);

   if (n != 2)
      return false;

   if (r == 0)
      return false;

   return true;
}

// =========================================================================
void kill_the_zombies()
{
   FILE* f = popen("ps -A | grep Zombie", "r");
   if (f == NULL)
   {
      log_severe("Unable to open ps process");
      return;
   }

   char line[132];

   while (fgets(line, sizeof(line), f) != NULL)
   {
      char procnum[80];
      if (sscanf(line, " %s ", procnum) != 1)
      {
         printf("Error reading process number\n");
         break;
      }

      int pid = getpid(); // dont kill myself

      if (pid != std::stod(procnum))
      {
         string p = (string)"sudo kill " + procnum;
         system(p.c_str());
      }
   }
}

// =========================================================================
void kill_my_children()
{
   int mypid = getpid();

   FILE* f = popen("ps xao pid,ppid", "r");
   if (f == NULL)
   {
      log_severe("Unable to open ps process");
      return;
   }

   char line[132];

   while (fgets(line, sizeof(line), f) != NULL)
   {
      int thispid, thisppid;
      if (sscanf(line, " %d %d ", &thispid, &thisppid) != 2)
         continue;  // probably the header line

      if ((thisppid == mypid) && (mypid != thispid))
      {
         string p = (string)"sudo kill " + std::to_string(thispid);
         system(p.c_str());
      }
   }
}

#if false
// =========================================================================
bool bipipe(struct bipipe_control_block& pcb, const char* mode, const char* command_file, int vacount, ...)
{

   // We make two pipes 
   // First pipe to send data from parent to child
   // Second pipe to send data from child to parent
   int parent_sends_fd[2] = { -1, -1 };  // Used to store two ends of first pipe  parent -> child
   int child_sends_fd[2] = { -1, -1 };  // Used to store two ends of second pipe child -> parent

   if ((strcmp(mode, "w") == 0) || (strcmp(mode, "rw") == 0))
      if (pipe(parent_sends_fd) == -1)
      {
         log_severe("Parent -> child pipe Failed");
         return false;
      }

   if ((strcmp(mode, "r") == 0) || (strcmp(mode, "rw") == 0))
      if (pipe(child_sends_fd) == -1)
      {
         log_severe("Child -> parent pipe failed");
         return false;
      }

   // make sure mode arg is valid
   assert((strcmp(mode, "r") == 0) || (strcmp(mode, "rw") == 0) || (strcmp(mode, "w") == 0));

   if (pcb.save_write_end != NULL)
   {
      *pcb.save_write_end = parent_sends_fd[PIPE_WRITING_END];
      printf("saving write fd %d %x\n", parent_sends_fd[PIPE_WRITING_END], pcb.save_write_end);
   }

   pcb.p = fork(); 

   if (pcb.p < 0)
   {
      log_severe("Fork failed");
      printf("Fork failed");
      return false;
   }

   // Parent process 
   else if (pcb.p > 0)
   {
      pcb.my_read_fd = child_sends_fd[PIPE_READING_END];
      pcb.my_write_fd = parent_sends_fd[PIPE_WRITING_END];

      return true;
   }

   // child process 
   else
   {
      prctl(PR_SET_NAME, "forkZombie"); // temporary name

      char* argv[20] = { (char *)"" };

      // in and out pipe fds will be tacked onto the end of argument list

      static string infd = std::to_string(parent_sends_fd[PIPE_READING_END]);
      static string outfd = std::to_string(child_sends_fd[PIPE_WRITING_END]);

//      printf("Forking: I/O = %d,%d\n", parent_sends_fd[PIPE_READING_END], child_sends_fd[PIPE_WRITING_END]);

      va_list args;
      va_start(args, vacount);
      for (int i = 0; i < vacount; ++i)
         argv[i + 1] = va_arg(args, char*);
      argv[0] = (char*)command_file;
      argv[vacount + 1] = (char *)infd.c_str();
      argv[vacount + 2] = (char *)outfd.c_str();
      argv[vacount + 3] = NULL;

      // pass on our privileges
      sudo_mode();

      main(vacount + 3, argv);


//      execvp(command_file, argv);

      log_severe("returned from exec: %s\n", strerror(errno));
      exit(-1); 
   }
}

// =========================================================================
bool other_end_closed(int fd)
{
   int pfd[2] = { -1, -1 };
   pipe(pfd);
   struct pollfd poll_list[1];

   poll_list[0].fd = fd;
   poll_list[0].events = POLLRDHUP | POLLNVAL;

   poll(poll_list, 1, 0);

   return (poll_list[0].revents & (POLLRDHUP | POLLNVAL)) != 0;
}

// =========================================================================
void _pmessage(int out_pipe_fd, const char* comp_id, const char* fmt, ...)
{
   va_list args{};
   va_start(args, fmt);

   char buffer[256]{ 0 };
   char buffer2[256]{ 0 };

   vsnprintf(buffer, sizeof(buffer), fmt, args);

   snprintf(buffer2, sizeof(buffer2), "%s,%s,%s\n", time_image("20%y/%m/%d %H:%M:%S").c_str(),
      comp_id, buffer);

   pprintf(out_pipe_fd, "%s", buffer2);

   va_end(args);
}

// =========================================================================
void pprintf(int pipefd, const char* fmt, ...)
{
   va_list args{};
   va_start(args, fmt);

   char temp[1024];

   try
   {
      ssize_t w = 0;
      size_t c = vsnprintf(temp, sizeof(temp), fmt, args);

      if (pipefd == 1)
         w = printf("%s", temp); // stdout can't be treated like a socket
      else
         w = write(pipefd, temp, std::min(c, sizeof(temp)));

      if (w < 0)
         printf("Send failed: <%s>, pipe=%d failed with error %s\n", temp, pipefd, strerror(errno));
   }
   catch (...)
   {
      printf("Possible broken pipe!");
   }

   va_end(args);
}


#endif

// =========================================================================
void wprintf(int pipefd, const char* fmt, ...)
{
   va_list args{};
   va_start(args, fmt);

   char temp[1024];

   try
   {
      ssize_t w = 0;
      size_t c = vsnprintf(temp, sizeof(temp), fmt, args);

      if (pipefd == 1)
         w = printf("%s", temp); // stdout can't be treated like a socket
      else
         w = send(pipefd, temp, std::min(c, sizeof(temp)), MSG_NOSIGNAL);

      if (w < 0)
         printf("Send failed: <%s>, pipe=%d failed with error %s\n", temp, pipefd, strerror(errno));
   }
   catch (...)
   {
      printf("Possible broken pipe!");
   }

   va_end(args);
}


// =========================================================================
void set_fd_nonblocking(int fd)
{
   int opt;
   opt = fcntl(fd, F_GETFL);
   if (opt < 0)
   {
      log_severe("fcntl(F_GETFL) fail, fd = %d", fd);
      return;
   }
   opt |= O_NONBLOCK;
   if (fcntl(fd, F_SETFL, opt) < 0)
      log_severe("fcntl(F_SETFL) fail, fd = %d", fd);
}

// =========================================================================
void set_fd_blocking(int fd)
{
   int opt;
   opt = fcntl(fd, F_GETFL);
   if (opt < 0)
   {
      log_severe("fcntl(F_GETFL) fail, fd = %d", fd);
      return;
   }
   opt &= ~O_NONBLOCK;
   if (fcntl(fd, F_SETFL, opt) < 0)
      log_severe("fcntl(F_SETFL) fail, fd = %d", fd);
}

// **************************************************************************************
int create_udp_socket()
{
   int sockfd;

   // Creating socket file descriptor 
   if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
   {
      log_severe("Opening socket! Error is: %s", strerror(errno));
      return false;
   }

   int optval = 1;
   setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
   setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));

   return sockfd;
}


// =========================================================================
void print_args(int argc, char* argv[])
{
   printf("Argument list:\n");
   for (int i = 0; i < argc; i++)
      printf("%2d:  <%s>\n", i, argv[i]);
   printf("End of argument list:\n");
}

// =========================================================================
string get_my_mac_address()
{
   FILE* f;

   string cmd = "sudo ifconfig | grep ether";

   f = popen(cmd.c_str(), "r"); /* Unix */
   if (f == NULL)
      return "'Unable to open ifconfig process'";
 
   char res[192];

   char *result = fgets(res, sizeof(res), f);

   if (result == NULL)
      return "'Unable to read from ifconfig process'";

   pclose(f);

   string rp = res;

   size_t pos = rp.find("ether ");
   if( pos == string::npos)
      return "'Unable to parse ifconfig output'";

   string reply = rp.substr(pos + 6, 17);

   return reply;
 }


// =========================================================================
string substitute(string& s, string pattern, string replacement)
{
   string res = s.substr(0, s.find(pattern));
   if (res == s)
      return s;
   res += replacement;
   res += s.substr(s.find(pattern) + pattern.length(), string::npos);

   return res;
}


// =========================================================================
string get_hostname_variant(string cmd, bool second = false)
{
   FILE* f;

   f = popen(cmd.c_str(), "r"); /* Unix */

   if (f == NULL)
      return "Unable to open hostname process";

   delay(100);

   char res[192];
   char* result = NULL;
   if (!feof(f))
      result = fgets(res, sizeof(res), f);

   if (result == NULL)
      return "Unable to read from hostname process";

   pclose(f);

   string rp = res;

   rp = replace(rp, '\n', ' ');
   if(second)
       rp = substitute(rp, config.lookup("static-ip"), "");
   rp = substitute(rp, "255.255.255.1", "");

   return head_of(rp, " ", true);
}

// =========================================================================
string get_my_static_ip()
{
   //return config.lookup("static-ip");
   return get_hostname_variant("hostname -I", false);
}


string get_my_second_ip()
{
   return get_hostname_variant("hostname -I", true);
}

string get_my_hostname()
{
   return get_hostname_variant("hostname", false);
}




// =========================================================================
bool user_mode()
{
   string username = config.lookup("user");

   uid_t uid;

   if (username == "")
      uid = 1000;
   else
   {
      struct passwd* name = getpwnam(username.c_str());
      uid = name->pw_uid;
   }

   // set euid down so we don't accidentally kill something.
   int result = seteuid(uid);
   if (result != 0)
   {
      fprintf(stderr, "Failed setting euid to '%s'", username.c_str());
      return false;
   }
   return true;
}

// =========================================================================
bool sudo_mode()
{
   int result = seteuid(0);
   if (result != 0)
   {
      fprintf(stderr, "Failed setting euid to root");
      return false;
   }
   return true;
}


// =========================================================================
string head_of(string& line, string symbol, bool trim_leading_whitespace)
{
   size_t pos = 0;

   string result = "";

   if (trim_leading_whitespace)
   {
      pos = line.find_first_not_of(" \t\r\n"); // skipping white space
      if (pos != string::npos)
         line = line.substr(pos, string::npos);
   }

   pos = line.find_first_of(symbol);

   result = line.substr(0, pos);

   if (pos != string::npos)
      line = line.substr(pos + 1, string::npos);
   else
      line = "";

   return result;
}

// =========================================================================
string free_split(string pattern, string target, string terminal)
{
   size_t n = 0;
   if ((n = target.find(pattern)) != string::npos)
   {
      string to_end = target.substr(n + pattern.length(), string::npos);
      n = to_end.find(terminal);

      if (n != string::npos)
         to_end = to_end.substr(0, n);

      return to_end;
   }
   return "";
}

// =========================================================================
string extract(string pattern, string optionlist, string delimiter)
{
   while (optionlist != "")
   {
      string keyval = head_of(optionlist, delimiter);

      if (keyval.find(pattern) != string::npos)
      {
         head_of(keyval, "=");
         return keyval;
      }
   }
   return "";
}

// =========================================================================
uint32_t ip_from_string(string ip)
{
   uint32_t result = 0;

   for (int i = 24; i >= 0; i -= 8)
   {
      string octet = head_of(ip, ".");

      result |= (stoi(octet) << i);
   }
   return result;
}

// =========================================================================
bool send_message_to_udp_port(int sockfd, string request, string ip_address, string port_address)
{
   bool closeport = false;

   //	sudo_mode();
   if (sockfd < 0)
   {
       static int generalFd = -1;

       if (generalFd > -1)
       {
           sockfd = generalFd;
       }
       else
       {
           // Creating socket file descriptor 
           if ((generalFd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
           {
               log_severe("Opening socket! Error is: %s", strerror(errno));
               return false;
           }
           //closeport = true;

           int optval = 1;
           setsockopt(generalFd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
           setsockopt(generalFd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));


           struct sockaddr_in myaddr;
           memset(&myaddr, 0, sizeof(myaddr));

           myaddr.sin_family = AF_INET; // IPv4 
           uint32_t hal = ip_from_string(get_my_static_ip());
           myaddr.sin_addr.s_addr = htonl(hal);
           myaddr.sin_port = htons(0);

           bind(generalFd, (sockaddr*)&myaddr, sizeof(myaddr));
           sockfd = generalFd;
       }
   }

   struct sockaddr_in cliaddr;
   memset(&cliaddr, 0, sizeof(cliaddr));

   cliaddr.sin_family = AF_INET; // IPv4 
   uint32_t hal = ip_from_string(ip_address);
   cliaddr.sin_addr.s_addr = htonl(hal);
   cliaddr.sin_port = htons((uint16_t)stoi(port_address)); 


   socklen_t len;
   len = sizeof(cliaddr);  //len is value/result 
   
   int r = (int)sendto(sockfd, request.c_str(), request.length(), MSG_CONFIRM, (const struct sockaddr*)&cliaddr, len);
   if (r < 0)
   {
      log_warning("Error sending UDP message <%s>!\Error is: %s, user is %d, address is %s",
         request.c_str(), strerror(errno), geteuid(), address_image(&cliaddr).c_str());
      if (closeport)
         ztclose(sockfd);
      return false;
   }

   if (closeport)
      ztclose(sockfd);

   return true;
}


// =========================================================================
string query_udp(string request, string ip_address, string port_address, int timeout_ms)
{
   int sockfd;
   char buffer[1500] = { 0 };
   struct sockaddr_in servaddr;
   memset(&servaddr, 0, sizeof(servaddr));

   if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
   {
      log_severe("Opening socket! Error is: %s", strerror(errno));
      return "";
   }

   int optval = 1;
   setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
   setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));

   struct sockaddr_in cliaddr;
   memset(&cliaddr, 0, sizeof(cliaddr));

   // Filling server information 
   servaddr.sin_family = AF_INET; // IPv4 
   servaddr.sin_addr.s_addr = INADDR_ANY;
   servaddr.sin_port = 0; // ANY

   set_fd_nonblocking(sockfd);

   // Bind the socket with the server address 
   if (bind(sockfd, (const struct sockaddr*)&servaddr,
      sizeof(servaddr)) < 0)
   {
      log_severe("Binding socket! Error is: %s", strerror(errno));
      ztclose(sockfd);
      return "failed";
   }

   while (request != "")
   {
      string cmd = head_of(request, ";");
      send_message_to_udp_port(sockfd, cmd, ip_address, port_address);

      if (request != "")
         delay(300);
   }

   delay(timeout_ms); // wait for reply

   string replies = "";

   while (true)
   {
      socklen_t len = sizeof(cliaddr);  //len is value/result 
      int n = (int)recvfrom(sockfd, (char*)buffer, sizeof(buffer), 0, (struct sockaddr*)&cliaddr, &len);
      // pmessage("recvfrom errno! Error is: %s", std::strerror(errno));

      if (n < 0)
      {
         ztclose(sockfd);
         return replies;
      }

      buffer[n] = '\0';

      string s = buffer;

      replies += ((string)buffer + "\n");
   }
}


// =========================================================================
char script[] = "\n"
"#! /usr/bin/bash\n"
"\n"
"# THIS SCRIPT IS AUTOMATICALLY GENERATED BY " MYNAME "DISCOVERY\n"
"\n"
"# sorts out the network manager configuration so we can have\n"
"# both a DHCP IP address and a static IP address if connected\n"
"# to a WAN, or just a static IP address is connected to a LAN.\n"
"\n"
"sleep 10\n"
"\n"
"# kill the networking and see if we can restart it with DHCP\n"
"sudo nmcli -t connection down ethernet-eth0\n"
"sudo nmcli -t connection delete ethernet-eth0\n"
"sudo nmcli -t connection add ifname eth0 type ethernet ipv4.method auto\n"
"sudo nmcli -t -w 5 connection up ethernet-eth0\n"
"# see if it worked\n"
"ip=`hostname -I`\n"
"\n"
"echo test IP $ip\n"
"\n"
"# shut down network again\n"
"sudo nmcli connection down ethernet-eth0\n"
"\n"
"if [ -z $ip ]\n"
"	then\n"
"	echo no DHCP assigned\n"
"	sudo nmcli -t connection delete ethernet-eth0\n"
"	sudo nmcli -t connection add ifname eth0 type ethernet ipv4.method manual +ipv4.addresses %s/24\n"
"  #add a fake IP so broadcasts work on the static interface\n"
"	sudo nmcli connection modify ethernet-eth0 +ipv4.addresses 255.255.255.1/24\n"
"\n"
"else\n"
"	echo DHCP assigned $ip\n"
"	sudo nmcli connection modify ethernet-eth0 +ipv4.addresses %s/24\n"
"	fi\n"
"\n"
"sudo nmcli connection up ethernet-eth0\n"
"\n"
"ip2=`hostname -I`\n"
"\n"
"echo 'Network address(es) set to' $ip2\n"
"";

char scriptout[sizeof(script) + 100] = { 0 };

// =========================================================================
void write_ip_boot_script(string ip_change)
{
   log_event("IP change = %s", (char*)ip_change.c_str());
   sprintf(scriptout, script, (char*)ip_change.c_str(), (char*)ip_change.c_str(), (char*)ip_change.c_str());

   sudo_mode();
   int sfd = open(config.lookup("ip-boot-script-location").c_str(), O_CREAT | O_WRONLY, 0444);
   if (sfd >= 0)
   {
      int n = (int)write(sfd, scriptout, strlen(scriptout));
      if (n < 0)
         log_warning("Writing network fixup script failed! Error is: %s", strerror(errno));

      ztclose(sfd);
      system("chmod +x /root/fixup_network.sh");

      if (config.lookup("reboot-on-change-static-ip") == "on")
         system("sudo reboot");
   }
   user_mode();
}

// =========================================================================
void _ztclose(int fd, const char* file, int line)
{
   if (fd < 3)
      log_severe("Closing fd with value %d, line %d, file %s", fd, line, file);
   else
      close(fd);
}

// =========================================================================
string NMEA_checksum(string input)
{
   uint8_t cs = 0;

   for (char c : input)
   {
      if ((c != '$') && (c != '*'))
         cs ^= (uint8_t)c;
   }

   char image[8];

   sprintf(image, "%02X", cs);
   return (string)image;
}

// =========================================================================
bool NMEA_checksum_valid(string input)
{
   uint8_t cs = 0;

   if (input.length() < 6)
      return false;

   if (input[0] != '$')
      return false;

   // delete any crlf
   if (input[input.length() - 1] == '\n')
      input = input.substr(0, input.length() - 2);

   string body = input.substr(1, input.length() - 4);

   for (char c : body)
         cs ^= (uint8_t)c;

   int csi = 0;
   sscanf(input.substr(input.length() - 2, 2).c_str(), "%x", &csi);

//   if (csi != cs)
//      printf("checksum check failed, len=%d: %s\n", input.length(), input.c_str());

   return (csi == cs);
}

// =========================================================================
void parseNMEA(string s, string format, ...)
{
   va_list vl;
   va_start(vl, format);

   int ptrin = 0, ptrout = 0;

   bool should_exit = false;

   for (auto t : format)
   {
      // find end of current section
      while (true)
      {
         ptrout++;
         if ((ptrout == (int)s.length()) || (s[ptrout] == '*'))
         {
            should_exit = true;
            break;
         }
         if (s[ptrout] == ',')
            break;
      }

      // stop at end of format string, either end of format or '*'
      if (should_exit)
         break;

      // get the next segment to decode 
      string segment = s.substr(ptrin, ptrout - ptrin);
      ptrin = ptrout + 1;

      switch (t)
      {
      case 'f':
      {
         float* fp = va_arg(vl, float*);
         bool* validptr = va_arg(vl, bool*);
         bool v = (1 == sscanf(segment.c_str(), "%f", fp));
         if (!v)
            *validptr = false;
         break;
      }

      case 'i':
      {
         int* ip = va_arg(vl, int*);
         bool* validptr = va_arg(vl, bool*);
         bool v = (1 == sscanf(segment.c_str(), "%d", ip));
         if (!v)
            *validptr = false;
         break;
      }

      case 's':
      {
         string* sp = va_arg(vl, string*);
         bool* validptr = va_arg(vl, bool*);
         *sp = segment;
         *validptr = true;
         break;
      }

      case '*':
         should_exit = true;
         break;

      default:
         break;
      }
   }

   va_end(vl);
}


