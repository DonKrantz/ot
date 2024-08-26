#include <stdlib.h>
#include <string>
#include <stdio.h>
#include "utilities.h"
#include "GPIO.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termio.h>
#include <termios.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <net/if.h>


using std::string;

namespace {

	const uint8_t ACK = 0x79;
	const uint8_t NACK = 0x1F;


struct firmware_control_block {
	// required inputs 
	string filename = "";
	string serialport = "";
	int baud_rate = B115200; 
	int reset_gpio = 24;	// 0 = reset
	int boot_gpio = 27;	// 0 = no boot


	// possible inputs
	uint32_t nvm_offset = 0x08200000 - 1024 * 128;
	size_t nvm_size = 1024 * 2;
	string ip = "0.0.0.0";
	string port = "0";
	string mask = "";

	// need cleanup
	uint8_t* image = NULL;
	uint8_t* nvm = NULL; // saved values
	int tty_fd = -1;

	// could be inputs
	uint32_t base_offset = 0x08000000;
	size_t max_size = 1024 * 128 * 4;	// four banks
	size_t actual_size = 0;
} fcb;

struct socket_control_block {
	int dvlfd = -1;
	LINE_BUFFER* lb = NULL;
	struct sockaddr_in cliaddr_s;
	struct sockaddr_in cliaddr_r;
} scb;

// **************************************************************************************
int Get32(FILE* f, long offset)
{
	uint8_t bytes[4];

	fseek(f, offset, 0);
	fread(bytes, (size_t)1, (size_t)4, f);
	return bytes[0] | bytes[1] << 8 | bytes[2] << 16 | bytes[3] << 24;
}

// **************************************************************************************
int Get16(FILE* f, long offset)
{
	uint8_t bytes[2];

	fseek(f, offset, 0);
	fread(bytes, (size_t)1, (size_t)2, f);
	return bytes[0] | bytes[1] << 8;
}

#if false
// **************************************************************************************
int Get8(FILE* f, long offset)
{
	uint8_t bytes[1];
	int numBytesRead = 0;

	fseek(f, offset, 0);
	fread(bytes, 1, 1, f);
	return bytes[0];
}
#endif

// **************************************************************************************
bool CopySegment(FILE *f, int segment_offset, struct firmware_control_block& fcb)
{
	int segment_size = Get32(f, segment_offset + 0x10);
	uint32_t file_offset = Get32(f, segment_offset + 0x04);
	uint32_t physical_address = Get32(f, segment_offset + 0x0C);
	if ((physical_address < fcb.base_offset) || (physical_address > fcb.base_offset + fcb.max_size))
		return true; // address is not in flash

	fseek(f, file_offset, 0);
	ssize_t n = fread(fcb.image + (physical_address - fcb.base_offset), 1, segment_size, f);
	if (n != (ssize_t)segment_size)
		return false;
	return true;
}



// **************************************************************************************
bool load_elf(struct firmware_control_block& fcb)
{
	FILE* f = fopen(fcb.filename.c_str(), "r");
	if (f == NULL)
		return false;

	int pfentries = Get16(f, 0x2C);
	int phsize = Get16(f, 0x2A);
	int phoffset = Get32(f, 0x1C);

	for (int i = 0; i < pfentries; i++)
	{
		int segoffset = phoffset + i * phsize;
		if (!CopySegment(f, segoffset, fcb))
		{
			log_severe("Reading ELF file");
			return false;
		}
	}

	// find the highest point modified
	for( int i = (int)fcb.max_size - 1; i >= 0; i--)
		if (fcb.image[i] != 0xff)
		{
			fcb.actual_size = i + 1;
			break;
		}

	return true;
}

// **************************************************************************************
uint8_t GetAck(int fd, float timeout = 0.1f)
{

	TIMING start = TIMENOW;

	uint8_t p;

	while (elapsed(start) < timeout)
	{
		ssize_t c = read(fd, &p, 1);

		if (c == 1)
			return p;

		delay(10);
	}

	return 0; // zero is timeout value
}


// **************************************************************************************
bool DoAutobaud(int fd)
{
	log_event("Starting autobaud");

	ioctl(fcb.tty_fd, TCFLSH, 0); // flush receive
	ioctl(fcb.tty_fd, TCFLSH, 1); // flush transmit

	uint8_t autobaud = 0x7F;
	uint8_t reply = 0;

	for (int i = 0; i < 3; i++)
	{
		write(fd, &autobaud, 1);

		if ((reply = GetAck(fd)) == ACK)
			return true;

		delay(50);
	}

	log_severe("Autobaud could not establish communication with the target");
	return false;
}



// **************************************************************************************
bool connect_to_target(struct firmware_control_block& fcb)
{
	if (fcb.boot_gpio != 0) // hardwaired boot and reset pins
	{
		// get ST into bootloader mode
		if (!setup_gpio())
		{
			log_severe("Autobaud GPIO init failure");
			return false;
		}
		else
		{
			set_gpio_to_output(fcb.reset_gpio);
			set_gpio_to_output(fcb.boot_gpio);

			set_gpio(fcb.reset_gpio, 0);  // into reset
			delay(20);
			set_gpio(fcb.boot_gpio, 1);  // boot pin on
			delay(2);
			set_gpio(fcb.reset_gpio, 1);  // release reset
			delay(100);			// wait for it to come up in bootloader mode
			set_gpio(fcb.boot_gpio, 0);  // boot pin off
		}
	}
	else
	{
		// not hardwired boot or reset pins

		if (fcb.baud_rate == 9600) fcb.baud_rate = B9600;
		else if(fcb.baud_rate == 115200) fcb.baud_rate = B115200;

		sudo_mode();
		fcb.tty_fd = openSerialDevice(fcb.serialport, true, fcb.baud_rate, false, false, false);
		user_mode();
		if (fcb.tty_fd < 0)
			return false;

		char bootstring[] = "\nBOOT\n";

		log_event("Halting CPU");
		write(fcb.tty_fd, bootstring, sizeof(bootstring) - 1); // put the device into boot mode
		delay(2000);

		ioctl(fcb.tty_fd, TCFLSH, 0); // flush receive
		ioctl(fcb.tty_fd, TCFLSH, 1); // flush transmit

		ztclose(fcb.tty_fd);
	}

	sudo_mode();
	fcb.tty_fd = openSerialDevice(fcb.serialport, true, 115200, false, true, false, true);
	user_mode();
	if (fcb.tty_fd < 0)
		return false;

	return DoAutobaud(fcb.tty_fd);
}

// **************************************************************************************
void WriteChecksum(int fd, uint8_t* buffer, int count, uint8_t sum = 0)
{
	for( int i = 0; i < count; i++)
		sum ^= buffer[i];

	write(fd, &sum, 1);
}


// **************************************************************************************
bool ReadFlash(struct firmware_control_block& fcb)
{
	uint32_t flash_address = fcb.nvm_offset;       // block of NV RAM area for programs
	uint8_t page[256] = { 0 };
	int fd = fcb.tty_fd;

	delay(10);
	ioctl(fcb.tty_fd, TCFLSH, 0); // flush receive
	ioctl(fcb.tty_fd, TCFLSH, 1); // flush transmit

	for (int i = 0; i < (int)fcb.nvm_size / (int)sizeof(page); i++)
	{
		uint8_t read_command[2] = { 0x11, 0xEE }; // includes checksum
		write(fd, read_command, sizeof(read_command));

		if (GetAck(fd) != ACK)
		{
			log_severe("Target refused read command");
			return false;
		}

		uint32_t address = flash_address + i * (uint32_t)sizeof(page);

		uint8_t read_address[4] = { (uint8_t)((address >> 24) & 0xFF), (uint8_t)((address >> 16) & 0xFF),
			 (uint8_t)((address >> 8) & 0xFF), (uint8_t)((address >> 0) & 0xFF) };

		write(fd, read_address, sizeof(read_address));
		WriteChecksum(fd, read_address, sizeof(read_address));

		if (GetAck(fd) != ACK)
		{
			log_severe("Read flash address failure at page %d", i);
			return false;
		}

		// byte count
		uint8_t byte_count[2] = { (uint8_t)(sizeof(page) - 1), (uint8_t)(~(sizeof(page) - 1)) };

		write(fd, byte_count, sizeof(byte_count));

		if (GetAck(fd) != ACK)
		{
			log_severe("Read flash count request failure at page %d", i);
			return false;
		}

		int tries = 10;
		ssize_t count = 0;
		while ((tries--) > 0)
		{
			count += read(fd, page + count, sizeof(page) - count);   // get a characters
			if (count == sizeof(page))
				break;
			delay(250);
		}

		if (count != sizeof(page))
		{
			log_severe("Read flash count return failure at page %d", i);
			return false;
		}

		// save page
		memcpy(fcb.nvm + i * sizeof(page), page, sizeof(page));
	}

	log_message("Read %d bytes of NVM", fcb.nvm_size);

	return true;
}


// **************************************************************************************
bool read_nvm(struct firmware_control_block& fcb)
{
	if (fcb.nvm_size == 0)
	{
		log_event("Skipping NVM read");
		return true;
	}

	delay(10);
	ioctl(fcb.tty_fd, TCFLSH, 0); // flush receive
	ioctl(fcb.tty_fd, TCFLSH, 1); // flush transmit

	fcb.nvm = (uint8_t*)malloc(fcb.nvm_size);
	if (fcb.nvm == NULL)
	{
		log_severe("Failure allocating memory in read_nvm()");
		return false;
	}

	return ReadFlash(fcb);
}


// **************************************************************************************
// Erase flash memory 
bool EraseFlash(struct firmware_control_block& fcb)
{
	delay(10);

	uint8_t reply;

	uint8_t erase_command[2] = { 0x44, 0xBB }; // erase command with checksum
	write(fcb.tty_fd, erase_command, sizeof(erase_command));

	if ((reply = GetAck(fcb.tty_fd, 1.0f)) != ACK)
	{
		log_severe("CPU refused erase command");
		return false;
	}

	uint8_t extended_erase_bank1[2] = { 0xFF, 0xFF };   // both banks
	write(fcb.tty_fd, extended_erase_bank1, sizeof(extended_erase_bank1));
	WriteChecksum(fcb.tty_fd, extended_erase_bank1, sizeof(extended_erase_bank1));

	if (GetAck(fcb.tty_fd, 20.0) != ACK)
	{
		log_severe("Target CPU erase attempt failure");
		return false;
	}
	return true;
}


// **************************************************************************************
bool WriteImage(struct firmware_control_block& fcb, string kind)
{
	uint32_t target_base = 0;
	uint8_t* source_base = 0;
	size_t count = 0;

	if (kind == "main firmware")
	{
		target_base = fcb.base_offset;
		source_base = fcb.image;
		count = fcb.actual_size;
	}
	else if( kind == "nvm" )// is nvm
	{
		if (fcb.nvm_offset == 0)
			return true;	// no NVM to do

		target_base = fcb.nvm_offset;
		source_base = fcb.nvm;
		count = fcb.nvm_size;
	}
	else
	{
		log_severe("Memory kind selector in WriteImage is bad");
		return false;
	}

	const int page_size = 256;

	size_t pages = (count + (size_t)page_size - 1) / (size_t)page_size;

	for (int i = 0; i < (int)pages; i++)
	{
		if ((i % 50) == 0)
			log_message("Writing page %d", i);

		bool dirty = false;

		for (int j = 0; j < page_size; j++)
			if (source_base[i * page_size + j] != 0xFF)
			{
				dirty = true;
				break;
			}

		if (!dirty)
			continue;

		int retry = 0;
		for (; retry < 5; retry++)
		{
			uint8_t write_command[2] = { 0x31, 0xCE }; // write command with checksum
			write(fcb.tty_fd, write_command, sizeof(write_command));

			if (GetAck(fcb.tty_fd, 0.5) != ACK)
			{
				log_severe("Retry write flash command failure at page %d", i);
				continue;
			}

			uint32_t address = target_base + i * page_size;
			uint8_t load_address[4] = { (uint8_t)((address >> 24) & 0xFF), (uint8_t)((address >> 16) & 0xFF),
			(uint8_t)((address >> 8) & 0xFF), (uint8_t)((address >> 0) & 0xFF) };
			write(fcb.tty_fd, load_address, sizeof(load_address));
			WriteChecksum(fcb.tty_fd, load_address, sizeof(load_address));

			if (GetAck(fcb.tty_fd, 0.5) != ACK)
			{
				log_warning("Retry write flash address failure at page %d", i);
				continue;
			}

			// byte count
			uint8_t bytes_to_write[1] = { page_size - 1 };
			write(fcb.tty_fd, bytes_to_write, sizeof(bytes_to_write));

			write(fcb.tty_fd, source_base + i * page_size, page_size);
			WriteChecksum(fcb.tty_fd, source_base + i * page_size, page_size, bytes_to_write[0]);

			if (GetAck(fcb.tty_fd, 0.5) != ACK)
			{
				log_severe("Retry write flash page failure at page %d", i);
				continue;
			}
			break;
		}

		if (retry == 5)
		{
			log_severe("Flash page write failure at page %d", i);
			return false;
		}
	}

	return true;
}

// **************************************************************************************
bool Launch(struct firmware_control_block& fcb)
{
	uint8_t go_command[2] = { 0x21, 0xDE };
	write(fcb.tty_fd, go_command, sizeof(go_command));

	if (GetAck(fcb.tty_fd) == ACK)
	{
		uint8_t vector_table[4] = { 0x08, 0x00, 0x00, 0x00 };

		write(fcb.tty_fd, vector_table, sizeof(vector_table));
		WriteChecksum(fcb.tty_fd, vector_table, sizeof(vector_table));
		if (GetAck(fcb.tty_fd, 0.1f) != ACK)
		{
			log_warning("Target did not acknowledge launch request, you may need to cycle power.");
			return false;
		}

		return true;
	}
	return false;
}

// **************************************************************************************
bool cleanup_STbootloader(struct firmware_control_block& fcb, bool result)
{
	if (fcb.image != NULL)
		free(fcb.image);
	fcb.image = NULL;

	if (fcb.nvm != NULL)
		free(fcb.nvm);
	fcb.nvm = NULL;

	if (fcb.tty_fd >= 0)
	{
		ztclose(fcb.tty_fd);
		fcb.tty_fd = -1;
	}

	return result;
}

#if false
// **************************************************************************************
bool print_usage()
{
	log_warning("Internal error, command line");
	return false;
}
#endif

// **************************************************************************************
bool read_command_line(struct firmware_control_block& fcb, int argc, char* argv[])
{
#if false
	if (argc != 11)
		return print_usage();

	int reset = std::stoi(argv[2]);
	if ((reset < 0) || (reset > 50))
		return print_usage();

	int boot = std::stoi(argv[3]);
	if ((boot < 0) || (boot > 50))
		return print_usage();

	uint32_t nvm_loc = 0;
	size_t nvm_size = 0;

	nvm_size = std::stoi(argv[7]);
	int n = sscanf(argv[6], "0x%x", &nvm_loc);
	if (n != 1)
	{
		nvm_loc = 0;
		nvm_size = 0;
	}

	fcb.baud_rate = std::stoi(argv[8]);

	fcb.filename = (string)argv[4];
	fcb.serialport = argv[5];
	fcb.boot_gpio = boot;
	fcb.reset_gpio = reset;
	fcb.nvm_offset = nvm_loc;
	fcb.nvm_size = nvm_size;
#endif

	return true;
}

} // namespace

// **************************************************************************************
void write_cmd_line(int argc, char* argv[])
{
	for (int i = 0; i < argc; i++)
		log_message("arg %d: %s", i, argv[i]);
}

// **************************************************************************************
bool STbootloader_main(int argc, char* argv[])
{
	write_cmd_line(argc, argv);

	if (! read_command_line(fcb, argc, argv))
		return false;

	fcb.image = (uint8_t*)malloc(fcb.max_size);
	fcb.nvm = (uint8_t*)malloc(fcb.nvm_size);
	if ((fcb.image == NULL) || (fcb.nvm == NULL))
	{
		log_severe("Could not get memory in STbootloader()");
		return false;
	}
	memset(fcb.image, 0xff, fcb.max_size);

	if (!load_elf(fcb))
	{
		{
			log_severe("Unable to load ELF %s", fcb.filename.c_str());
			return cleanup_STbootloader(fcb, false);
		}
	}

	log_message("Read %d bytes from %s", fcb.actual_size, fcb.filename.c_str());

	if (!connect_to_target(fcb))
	{
		{
			log_severe("Failure connecting to target");
			return cleanup_STbootloader(fcb, false);
		}
	}

	log_event("Connected to target, checking for NV RAM");

	if (!read_nvm(fcb))
		return cleanup_STbootloader(fcb, false);

	log_event("NV RAM complete, starting erase");

	if (!EraseFlash(fcb))
		return cleanup_STbootloader(fcb, false);

	log_event("Erase complete, starting code write");

	if (!WriteImage(fcb, "main firmware"))
		return cleanup_STbootloader(fcb, false);

	log_event("Code write complete, starting NVM write");

	if (!WriteImage(fcb, "nvm"))
		return cleanup_STbootloader(fcb, false);

	log_event("NVM write complete, launching target");

	if (!Launch(fcb))
	{
		return cleanup_STbootloader(fcb, false);
	}

	log_event("Target launched, firmware update complete");

	return cleanup_STbootloader(fcb, true);
}


namespace {

#if false
	// **************************************************************************************
	int socketid()
	{
		int sockfd;
		struct sockaddr_in servaddr;

		// Creating socket file descriptor 
		if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
			log_severe("Opening socket! Error is: %s", strerror(errno));
			return false;
		}

		set_fd_nonblocking(sockfd);

		memset(&servaddr, 0, sizeof(servaddr));

		// Filling server information 
		servaddr.sin_family = AF_INET; // IPv4 
		servaddr.sin_addr.s_addr = INADDR_ANY;
		servaddr.sin_port = htons(0);

		int optval = 1;
		setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
		setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));

		// Bind the socket with the server address 
		if (bind(sockfd, (const struct sockaddr*)&servaddr,
			sizeof(servaddr)) < 0)
		{
			log_severe("Binding socket! Error is: %s", strerror(errno));
			ztclose(sockfd);
			return false;
		}

		return sockfd;
	}
#endif

#if false
	// **************************************************************************************
	bool print_usage_fwdvl()
	{
		printf(
			"Usage: ot dvlfw elf_file ip port mask inpipe_fd outpipe_fd\n"
			"   elf_file     .ELF filename\n"
			"   ip				IP address of device\n"
			"   port				UDP port of device\n"
			"   mask				burn mask, master=1,0,0,0,0 channel=0,1,1,1,1\n"
			"   inpipe_fd    file descriptor for read end of incoming pipen\n"
			"   outpipe_fd   file descriptor for write end of outgoing pipe\n"
			"Example: ot dvlfw /home/cerulean/t650.elf 192.168.2.3 50000 '1,0,0,0,0' 0 1\n\n"
		);
		return false;
	}
#endif

	// **************************************************************************************
	bool read_command_line_fwdvl(struct firmware_control_block& fcb, 
		struct socket_control_block& scb, int argc, char* argv[])
	{
#if false
		if (argc != 8)
			return print_usage_fwdvl();

		fcb.filename = argv[2];

		fcb.ip = argv[3];
		fcb.port = argv[4];
		scb.dvlfd = socketid();

		memset(&(scb.cliaddr_s), 0, sizeof(scb.cliaddr_s));
		scb.cliaddr_s.sin_family = AF_INET; // IPv4 
		uint32_t hal = ip_from_string(fcb.ip);
		scb.cliaddr_s.sin_addr.s_addr = htonl(hal);
		scb.cliaddr_s.sin_port = htons((uint16_t)std::stoi(fcb.port));

		fcb.mask = argv[5];

		scb.lb = new LINE_BUFFER(scb.dvlfd);

#endif
		return true;
	}


#if false
	// **************************************************************************************
	bool flush_incoming(struct socket_control_block& scb)
	{
		TIMING entry = TIMENOW;

		char buffer[1600];
		memset(&scb.cliaddr_r, 0, sizeof(scb.cliaddr_r));
		socklen_t len;

		int count = 0;

		while (true)
		{
			len = sizeof(scb.cliaddr_r);  //len is value/result 
			ssize_t n = recvfrom(scb.dvlfd, (char*)buffer, sizeof(buffer), 0, (struct sockaddr*)&scb.cliaddr_r, &len);
			buffer[n] = '\0';

			if (elapsed(entry) > 2.0f)
				break;

			if ((n == -1) && (errno == 11))
				continue;

			if (n == -1)
				continue;

			count++;
		}

		return true;
	}
#endif

	// **************************************************************************************
	int write_to_dvl(struct socket_control_block& scb, const char* m)
	{
		return (int)sendto(scb.dvlfd, m, strlen(m), MSG_CONFIRM, (const sockaddr*)&scb.cliaddr_s, sizeof(scb.cliaddr_s));
	}

#if false
	// **************************************************************************************
	int dump_dvl(struct socket_control_block& scb, float time_limit = 0.5)
	{
		TIMING start = TIMENOW;
		int count = 0;
		while (true)
		{
			string s = scb.lb->next_line();
			if (s != "")
			{
				log_message("%s", s.c_str());
				count++;
			}

			if (elapsed(start) >= time_limit)
				return count;
		}
	}
#endif

} // namespace


// **************************************************************************************
bool fwdvl_main(int argc, char* argv[])
{
	write_cmd_line( argc, argv);

	if (!read_command_line_fwdvl(fcb, scb, argc, argv))
	{
		log_severe("Command line errors, %d arguments", argc);
		for (int i = 0; i < argc; i++)
			log_data("arg %d is %s", i, argv[i]);
		return false;
	}

	fcb.image = (uint8_t*)malloc(fcb.max_size);
	if (fcb.image == NULL)
	{
		log_severe("Could not get memory in fwdvl()");
		return false;
	}
	memset(fcb.image, 0xff, fcb.max_size);

	if (!load_elf(fcb))
	{
		{
			log_severe("Unable to load ELF in fwdvl()");
			return false;
		}
	}

	log_event("Read %d bytes from %s in fwdvl", fcb.actual_size, fcb.filename.c_str());

#if false
	flush_incoming(scb);

	if (scb.cliaddr.sin_addr.s_addr == 0)
	{
		log_warning("Could not see DVL, is it on line and active?");
		return false;
	}
#endif
	log_event("Connected to DVL, starting download");

	write_to_dvl(scb, "\n");
	delay(200);

	write_to_dvl(scb, "UNICAST-TO-ME\n");
	delay(200);
	log_event("Captured communications");

	write_to_dvl(scb, "PAUSE\n");
	delay(200);


	char buffer[1600];
	sprintf(buffer, "BEGIN-BOOTLOAD,%s,%d\n", fcb.mask.c_str(), fcb.actual_size); // no bootload
	write_to_dvl(scb, buffer);

	TIMING watchdog = TIMENOW;

	int packet = 0;
	size_t fwpointer = 0;

	while (true)
	{
		if (elapsed(watchdog) > 3.0)
		{
			log_severe("Timed out downloading firmware at packet %d", packet);
			return false;
		}

		string s = scb.lb->next_line(); //=======================================
		if (s == "")
			continue;

		watchdog = TIMENOW;

		string expecting = "BINARY-HANDSHAKE";
		if (s.substr(0, expecting.length()) != expecting)
			continue;

		if (fwpointer >= fcb.actual_size)
			break;

		size_t next_count = std::min(fcb.actual_size - fwpointer, (size_t)1000);

		sprintf(buffer, "BINARY-PACKET > %d", next_count);
		memcpy(buffer + strlen(buffer) + 1, fcb.image + fwpointer, next_count);

		sendto(scb.dvlfd, buffer, strlen(buffer) + next_count + 1, MSG_CONFIRM, 
			(const sockaddr*)&scb.cliaddr_s, sizeof(scb.cliaddr_s));

		fwpointer += next_count;

		packet++;
	}

	write_to_dvl(scb, "END-BINARY\n");

	write_to_dvl(scb, "UNICAST-TO-ME OFF\n");
	log_event("Relinguished communications");

	write_to_dvl(scb, "RESUME\n");

	ztclose(scb.dvlfd);

	return true;
}
