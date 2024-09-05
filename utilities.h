#pragma once
#include <list>
#include <iterator>
#include <cerrno>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <chrono>
#include <sys/prctl.h>

#include <thread>

#include "logger.h"

#include "queue.h"

//#include "MAVlink.h"

using std::string;
using std::list;

#define MYNAME "OmniTrack"
#define MYOUT "ot"

#if false
// Provides line buffering to a line-oriented input file. 
// This supports non-blocking reading if the input fd is set to
// non-blocking.
// Should be used only in single-threaded applications
class LINE_BUFFER
{
private:
   int			m_ptr = 0;
   char			m_buffer[1600];
   int			m_fd = 0;
   char			m_staging[1600];
   ssize_t	   m_staging_count = 0;
   int			m_staging_ptr = 0;
   bool        m_ignore_blanks = false;

public:
   LINE_BUFFER(int fd, bool ignore_blanks = false);
   ~LINE_BUFFER();

   int next_line(char* buffer, int max_length);
   string next_line();

   void flush();
};

#endif

// returns the value from a key/value pair
string keyvalkey(const string s, const string splitter);

// returns the key from a key/value pair
string keyvalvalue(const string s, const string splitter);

// type defining a list of strings
typedef  list<string> STRINGLIST;

/* Returns a list of files in a directory (except the ones that begin with a dot) */
STRINGLIST GetFilesInDirectory(string directory, string pattern, bool just_the_name = false);

/* Returns a list of files in a directory (except the ones that begin with a dot) */
void GetFilesInDirectory(list<string>& result, string directory, string pattern, bool just_the_name = false);


// some clock types used in timing utilities
typedef std::chrono::steady_clock Clock;
typedef std::chrono::_V2::steady_clock::time_point TIMING; // type used for elapsed time

#define TIMENOW (Clock::now())
extern TIMING mission_start_time; // for logging with timestamp starting at zero

// elapsed time in seconds
typedef std::chrono::duration<double> MISSION_DURATION;
inline double elapsed(TIMING start)
{
	std::chrono::duration<double> elapsed_seconds = Clock::now() - start;
	return elapsed_seconds.count();
}

// gets the time now formatted per format string
string time_image(string format); // e.g., format = "%y-%m-%d-%H-%M-%S" for e.g, 22-01-10-10-55-22

string address_image(struct sockaddr_in *addr);

uint32_t ip_from_string(string ip);

// returns a string of mission time elapsed "HH:MM:SS"
string mission_time();	

// Millisecond delay (sleeps during delay)
void delay(int milliseconds);

#define UNUSED(x) ((void)x)

// Returns 'true' if we can successfully ping 'address'
bool ping(const string address, const string ip_from);

// size (element count) of a C array
#define COUNT(x) (sizeof(x)/sizeof(x[0]))

extern std::thread discovery_thread;
extern std::thread ports_thread;
extern std::thread web_thread;

#if false
// ================================================================================
// pipe and child support help

extern int in_pipe_fd; // you want to set these from the child's command line at 
extern int out_pipe_fd; // the start of a process.
extern char component_id[30]; // you want to set this at the start of a child process
                              // to attach the child process ID to the start of pmessage

// printf clone to write non-buffered output to a pipe.
void pprintf(int pipefd, const char* fmt, ...);

// sends a message back to the parent through a pipe. You need to set 'out_pipe-fd'
// from the child's command line when the child process begins.
#define pmessage(fmt, ...) _pmessage(out_pipe_fd, (const char *)component_id, fmt, ##__VA_ARGS__)

// helper for pmessage
void _pmessage(int out_pipe_fd, const char* comp_id, const char* fmt, ...);

// for the parent of a pipe-connected child process.
struct bipipe_control_block {
   // We use two pipes 
   // First pipe to send input string from parent 
   // Second pipe to send concatenated string from child 
   pid_t p = 0;     // child's pid 
   int my_read_fd = -1;
   int my_write_fd = -1;
   int* save_write_end = NULL;
};

#define PIPE_WRITING_END 1 // index for pipe fd pairs
#define PIPE_READING_END 0 // index for pipe fd pairs

// Fork off a process with two pipes, one each direction. Pipes 
// // start out as 'blocking.' Returns false
// if the fork didn't work right.
// 
// mode: "r" read-only, child sends messages back to parent thru pipe
// mode: "w" write-only, parent sends messages to child thru pipe
// mode: "rw" read-write, parent and child can send messages to each other
// mode: anything else triggers assert()
// 
// Examples:
//      bipipe(pcb, "r", "ls", 2, "-l", "/home/cerulean");
//      bipipe(pcb, "rw", "cat", 0);

bool bipipe(struct bipipe_control_block& pcb, const char* mode, const char* command_file, int var_arg_count, ...);

bool other_end_closed(int fd); // for seeing if writng end of pipe is closed or broken


#endif

// printf clone to write non-buffered output to a socket.
void wprintf(int sockfd, const char* fmt, ...);

// Logging macros -- writes to the log file if it's open
#define log_message(fmt, ...) log_write("MESSAGE," fmt, ##__VA_ARGS__)
#define log_data(fmt, ...) log_write("DATA," fmt, ##__VA_ARGS__)
#define log_event(fmt, ...) log_write("EVENT," fmt, ##__VA_ARGS__)
#define log_warning(fmt, ...) log_write("WARNING," fmt, ##__VA_ARGS__)
#define log_severe(fmt, ...) log_write("SEVERE," fmt, ##__VA_ARGS__)

// needed because the debugger leaves inconvenient zombie processes running
void kill_the_zombies();

// =========================================================================

// Makes an input fd non-blocking
void set_fd_nonblocking(int fd); 

// Makes an input fd non-blocking
void set_fd_blocking(int fd);

// debugging helps
void print_args(int argc, char* argv[]);

// Gets the MAC address foe the 'ether' interface.
string get_my_mac_address();

// gets the static IP address for the 'ether' interface
string get_my_static_ip();

string get_my_second_ip();

string get_my_hostname();

string lookup_usb(int number);
string lookup_serial(int number);

int create_udp_socket();

// =========================================================================
bool user_mode();

bool sudo_mode();

// =========================================================================
// Opens a serial port with various parameters set
int openSerialDevice(string filename, bool raw_mode, int baud_rate, bool line_mode, bool odd_parity, bool useCTSRTS, bool noblock = false);

// Changes the serial parameters in an open serial port. Returns false on error.
bool setSerialDevice(int fd, string baud_rate, string data_bits, string parity, string stop_bits);

// =========================================================================

inline string toupper(string buf) { for (auto& c : buf) c = (char)toupper(c); return buf; }
inline string tolower(string buf) { for (auto& c : buf) c = (char)tolower(c); return buf; }
inline string replace(string buf, char target, char substitute) { for (auto& c : buf) if(c == target) c = substitute; return buf; }
inline bool contains(string pattern, string target) { return target.find(pattern) != string::npos; }
string substitute(string& s, string pattern, string replacement);
string free_split(string pattern, string target, string teminal = "*");
string fw_version(string date); // date is in 'Mon dd yyyy' format ( C __DATE__ macro )
string fw_version_from_yyyy_mm_dd(string date); // date is in  yyyy-mm-dd format
bool newer_than(string v1, string v2); // versions are in x.y.z format

string head_of(string& line, string symbol, bool trim_leading_whitespace = true);

string extract(string pattern, string optionlist, string delimiter);

void parseNMEA(string s, string format, ...);

// if sockfd < 0, will open/close a new socket.
bool send_message_to_udp_port(int sockfd, string request, string ip_address, string port_address);

// request is list of UDP messages separated by semicolon, 
string query_udp(string request, string ip_address, string port_address, int timeout_ms = 1000);

void write_ip_boot_script(string ip_change);

#define ztclose(fd) _ztclose(fd, __FILE__, __LINE__)
void _ztclose(int fd, const char* file, int line);

void kill_my_children();

// 2-digit NMEA checksum
string NMEA_checksum(string input);
bool NMEA_checksum_valid(string input);

//extern MAVlink* mavlink;

#define ROVL_RX_TTY "/dev/ttyAMA3"
//#define ROVL_TX_TTY "/dev/ttyAMA2"
#define GNSS_BASE_TTY "/dev/ttyAMA5"
#define GNSS_ROVER_TTY "/dev/ttyAMA4"

#define degrees(x) (x * 180.0 / M_PI)
