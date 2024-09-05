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

#include "serializer_main.h"

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

	// **************************************************************************************
	bool CopySegment(FILE* f, int segment_offset, struct firmware_control_block& fcb)
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
		for (int i = (int)fcb.max_size - 1; i >= 0; i--)
			if (fcb.image[i] != 0xff)
			{
				fcb.actual_size = i + 1;
				break;
			}

		return true;
	}

	// **************************************************************************************
	uint8_t GetAck(int timeout = 100)
	{
		uint8_t p;
		TIMING t = TIMENOW;
		while (true)
		{
			int n = receive_from(PORTS::ROVL_RX, &p, 1, timeout);

			if ((n > 0))
				return p;

			if (elapsed(t) > ((float)timeout) / 1000.0)
				break;

			delay(1);
		}
		return 0; // zero is timeout value
	}


	// **************************************************************************************
	bool DoAutobaud()
	{
		log_event("Starting autobaud");

		uint8_t autobaud = 0x7F;
		uint8_t reply = 0;

		for (int i = 0; i < 3; i++)
		{
			send_port_binary(PORTS::ROVL_RX, &autobaud, 1);

			if ((reply = GetAck()) == ACK)
				return true;

			delay(50);
		}

		log_severe("Autobaud could not establish communication with the target");
		return false;
	}

	// **************************************************************************************
	bool connect_to_target(struct firmware_control_block& fcb)
	{
		log_event("Halting USBL/ROVL CPU");

		send_port_message(PORTS::ROVL_RX, "\nBOOT\n");
		delay(2000);

		uint8_t junk[1024];
		while (receive_from(PORTS::ROVL_RX, junk, sizeof(junk), 100) != 0)
			;

		//port_set_parity(PORTS::ROVL_RX, "even");

		port_set_canonical(PORTS::ROVL_RX, false, true);

		return DoAutobaud();
	}

	// **************************************************************************************
	void WriteChecksum(uint8_t* buffer, int count, uint8_t sum = 0)
	{
		for (int i = 0; i < count; i++)
			sum ^= buffer[i];

		send_port_binary(PORTS::ROVL_RX, &sum, 1);
	}


	// **************************************************************************************
	bool ReadFlash(struct firmware_control_block& fcb)
	{
		uint32_t flash_address = fcb.nvm_offset;       // block of NV RAM area for programs
		uint8_t page[256] = { 0 };

		delay(10);

		for (int i = 0; i < (int)fcb.nvm_size / (int)sizeof(page); i++)
		{
			uint8_t read_command[2] = { 0x11, 0xEE }; // includes checksum
			send_port_binary(PORTS::ROVL_RX, read_command, sizeof(read_command));

			uint8_t reply;
			UNUSED(reply);

			if ((reply = GetAck(100)) != ACK)
			{
				log_severe("Target refused read command");
				return false;
			}

			uint32_t address = flash_address + i * (uint32_t)sizeof(page);

			uint8_t read_address[4] = { (uint8_t)((address >> 24) & 0xFF), (uint8_t)((address >> 16) & 0xFF),
				 (uint8_t)((address >> 8) & 0xFF), (uint8_t)((address >> 0) & 0xFF) };

			send_port_binary(PORTS::ROVL_RX, read_address, sizeof(read_address));
			WriteChecksum(read_address, sizeof(read_address));

			if ((reply = GetAck(100)) != ACK)
			{
				log_severe("Read flash address failure at page %d", i);
				return false;
			}

			// byte count
			uint8_t byte_count[2] = { (uint8_t)(sizeof(page) - 1), (uint8_t)(~(sizeof(page) - 1)) };

			send_port_binary(PORTS::ROVL_RX, byte_count, sizeof(byte_count));

			if ((reply = GetAck(100)) != ACK)
			{
				log_severe("Read flash count request failure at page %d", i);
				return false;
			}

			int tries = 10;
			ssize_t count = 0;
			while ((tries--) > 0)
			{
				count += receive_from(PORTS::ROVL_RX, page + count, sizeof(page) - count, 100);   // get a characters
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

		tcflush(fcb.tty_fd, TCIOFLUSH);

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
		send_port_binary(PORTS::ROVL_RX, erase_command, sizeof(erase_command));

		if ((reply = GetAck(1000)) != ACK)
		{
			log_severe("CPU refused erase command");
			return false;
		}

		uint8_t extended_erase_bank1[2] = { 0xFF, 0xFF };   // both banks
		send_port_binary(PORTS::ROVL_RX, extended_erase_bank1, sizeof(extended_erase_bank1));
		WriteChecksum(extended_erase_bank1, sizeof(extended_erase_bank1));

		if (GetAck(20000) != ACK)
		{
			log_severe("Target CPU erase attempt failure");
			return false;
		}
		return true;
	}


	// **************************************************************************************
	bool WriteImage(struct firmware_control_block& fcb, string kind, int fd)
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
		else if (kind == "nvm")// is nvm
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
			{
				log_message("Writing page %d", i);
				wprintf(fd, "<p>Writing page %d</p>\r\n", i);
			}

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
				send_port_binary(PORTS::ROVL_RX, write_command, sizeof(write_command));

				if (GetAck(500) != ACK)
				{
					log_severe("Retry write flash command failure at page %d", i);
					continue;
				}

				uint32_t address = target_base + i * page_size;
				uint8_t load_address[4] = { (uint8_t)((address >> 24) & 0xFF), (uint8_t)((address >> 16) & 0xFF),
				(uint8_t)((address >> 8) & 0xFF), (uint8_t)((address >> 0) & 0xFF) };
				send_port_binary(PORTS::ROVL_RX, load_address, sizeof(load_address));
				WriteChecksum(load_address, sizeof(load_address));

				if (GetAck(500) != ACK)
				{
					log_warning("Retry write flash address failure at page %d", i);
					continue;
				}

				// byte count
				uint8_t bytes_to_write[1] = { page_size - 1 };
				send_port_binary(PORTS::ROVL_RX, bytes_to_write, sizeof(bytes_to_write));

				send_port_binary(PORTS::ROVL_RX, source_base + i * page_size, page_size);
				WriteChecksum(source_base + i * page_size, page_size, bytes_to_write[0]);

				if (GetAck(500) != ACK)
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
		send_port_binary(PORTS::ROVL_RX, go_command, sizeof(go_command));

		if (GetAck() == ACK)
		{
			uint8_t vector_table[4] = { 0x08, 0x00, 0x00, 0x00 };

			send_port_binary(PORTS::ROVL_RX, vector_table, sizeof(vector_table));
			WriteChecksum(vector_table, sizeof(vector_table));
			if (GetAck(100) != ACK)
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

		//port_set_parity(PORTS::ROVL_RX, "none");

		port_set_canonical(PORTS::ROVL_RX, true, false);

		port_reconnect(PORTS::ROVL_RX);

		send_port_message(PORTS::INTERNAL_RX, "jog");

		return result;
	}

} // namespace


// **************************************************************************************
bool STbootloader_main(string filename, int fd)
{
	fcb.filename = filename;
	fcb.nvm_size = 2048;
	fcb.nvm_offset = 0x0807F800;
	fcb.image = (uint8_t*)malloc(fcb.max_size);
	fcb.nvm = (uint8_t*)malloc(fcb.nvm_size);
	fcb.max_size = 384 * 1024 - 2048;

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

	wprintf(fd, "<p>Firmware file loaded</p>\r\n");

	port_disconnect(PORTS::ROVL_RX);
	send_port_message(PORTS::INTERNAL_RX, "jog");

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

	wprintf(fd, "<p>NV RAM reading complete, starting erase</p>\r\n");
	log_event("NV RAM complete, starting erase");

	if (!EraseFlash(fcb))
		return cleanup_STbootloader(fcb, false);

	log_event("Erase complete, starting code write");
	wprintf(fd, "<p>Erase complete, starting code write</p>\r\n");

	if (!WriteImage(fcb, "main firmware", fd) )
		return cleanup_STbootloader(fcb, false);

	log_event("Code write complete, starting NVM write");
	wprintf(fd, "<p>Code write complete, starting NVM write</p>\r\n");

	if (!WriteImage(fcb, "nvm", fd) )
		return cleanup_STbootloader(fcb, false);

	log_event("NVM write complete, launching target");

	if (!Launch(fcb))
	{
		return cleanup_STbootloader(fcb, false);
	}

	log_event("Target launched, firmware update complete");
	wprintf(fd, "<p>Target launched</p>\r\n");

	return cleanup_STbootloader(fcb, true);
}


// **************************************************************************************
bool fwdvl_main(string filename, int fd)
{
	fcb.filename = filename;
	fcb.max_size = 1024 * 1024;
	fcb.nvm_size = 0;

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

	port_disconnect(PORTS::TRACKER650);

	send_port_message(PORTS::TRACKER650, "\n");
	delay(200);
	string junk = receive_from(PORTS::TRACKER650, 100);

	send_port_message(PORTS::TRACKER650, "UNICAST-TO-ME\n");
	delay(200);
	log_event("Captured communications");
	junk = receive_from(PORTS::TRACKER650, 100);

	send_port_message(PORTS::TRACKER650, "PAUSE\n");
	delay(200);
	junk = receive_from(PORTS::TRACKER650, 100);

	char buffer[1600];
	send_port_message(PORTS::TRACKER650, "BEGIN-BOOTLOAD,1,0,0,0,0," + std::to_string(fcb.actual_size));
	junk = receive_from(PORTS::TRACKER650, 100);

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

		string s = receive_from(PORTS::TRACKER650, 1000);
		if (s == "")
			break;  // timed out waiting for message

		string expecting = "BINARY-HANDSHAKE";
		if (!contains(expecting, s))
			continue;

		if (fwpointer >= fcb.actual_size)
			break;

		size_t next_count = std::min(fcb.actual_size - fwpointer, (size_t)1000);

		sprintf(buffer, "BINARY-PACKET > %d", next_count);
		memcpy(buffer + strlen(buffer) + 1, fcb.image + fwpointer, next_count);

		send_port_binary(PORTS::TRACKER650, buffer, strlen(buffer) + next_count + 1);

		fwpointer += next_count;

		packet++;

		watchdog = TIMENOW;

		delay(20);

	}

	send_port_message(PORTS::TRACKER650, "END-BINARY\n");

	send_port_message(PORTS::TRACKER650, "UNICAST-TO-ME OFF\n");
	log_event("Relinguished communications");

	send_port_message(PORTS::TRACKER650, "RESUME\n");

	string s;
	while( (s = receive_from(PORTS::TRACKER650, 1000)) != "" )
		printf("dvl: %s\n", s.c_str());


	port_reconnect(PORTS::TRACKER650);

	return true;
}
