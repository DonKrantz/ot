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
#include <poll.h>

#include "utilities.h"

#include "discovery_main.h"
#include "configuration.h"

using std::string;

namespace 
{
#define SET_IP_FOR_MAC "SET-IP-FOR-MAC-ADDRESS "

	bool boot_ok = false;
	string dev_name = "";
	string ip_address = "";
	string ip_port = "";
	string mac_address = "";
	string fw_versions = "";

#if false
	// **************************************************************************************
	bool print_usage()
	{
		printf(
			"Usage: ot discdem inpipe_fd outpipe_fd\n"
			"   inpipe_fd   file descriptor for read end of incoming pipe\n"
			"   outpipe_fd   file descriptor for write end of outgoing pipe\n"
			"Example: ot discdem 0 1\n\n"
		);
		return false;
	}
#endif

	// **************************************************************************************
	string construct_reply_discovery(char* buffer)
	{
		if (strncmp(buffer, "Discovery", sizeof("Discovery")) != 0)
			return "";

		char reply[128] = { 0 };

		sprintf(reply, "%s\r\n%s\r\n"
			"MAC Address:-%s\r\n"
			"IP Address:-%s\r\n"
			"Port:-%s\r\n"
			"Firmware Version:-%s\r\n",
			dev_name.c_str(),
			"Cerulean Sonar",
			mac_address.c_str(),
			ip_address.c_str(),
			ip_port.c_str(),
			fw_versions.c_str());

		return (string)reply;
	}


	//========================================================================================================
	// implements SET-IP-FOR-MAC
	string set_ip_for_mac_address(char* buffer)
	{
		unsigned int maci[6];
		unsigned int ipi[4];

		if (sscanf(buffer, "SET-IP-FOR-MAC-ADDRESS MAC:%2x-%2x-%2x-%2x-%2x-%2x IP:%i.%i.%i.%i",
			maci, maci + 1, maci + 2, maci + 3, maci + 4, maci + 5,
			ipi, ipi + 1, ipi + 2, ipi + 3
		) != 10)
			return (string)"";

		string buf = buffer;
		buf = toupper(buf);

		if (buf.substr(0, sizeof(SET_IP_FOR_MAC) - 1) != SET_IP_FOR_MAC)
			return "";

		string mac = buf.substr( buf.find("MAC:") + 4, 17);
		string ip = buf.substr(buf.find("IP:") + 3);

		for (auto& c : mac_address) c = (char)toupper(c); // my MAC

		if (mac != mac_address)
			return "";

		return ip;
	}


	// **************************************************************************************
	bool discovery_daemon()
	{
		int sockfd;
		char buffer[1500] = { 0 };
		struct sockaddr_in servaddr, cliaddr;

		// Creating socket file descriptor 
		if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
			log_severe("Opening socket! Error is: %s", strerror(errno));
			return false;
		}

		memset(&servaddr, 0, sizeof(servaddr));
		memset(&cliaddr, 0, sizeof(cliaddr));

		// Filling server information 
		servaddr.sin_family = AF_INET; // IPv4 
		servaddr.sin_addr.s_addr = INADDR_ANY;
		servaddr.sin_port = htons(30303);

		int optval = 1;
		setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
		setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));

		// Bind the socket with the server address 
		if (bind(sockfd, (const struct sockaddr*)&servaddr,
			sizeof(servaddr)) < 0)
		{
			log_severe("Binding socket! Error is: %s", strerror(errno));
			close(sockfd);
			return false;
		}

		socklen_t len;
		int n;

		len = sizeof(cliaddr);  //len is value/result 

		log_event("Discovery daemon started on port 30303");

		while (true)
		{

			int pfd[2] = { -1, -1 };
			pipe(pfd);
			struct pollfd poll_list[2];

			poll_list[0].fd = sockfd;
			poll_list[0].events = POLLIN;

//			poll_list[1].fd = in_pipe_fd;	// read end of parent's write pipe
			poll_list[1].events = POLLRDHUP | POLLNVAL;

			poll(poll_list, 2, -1);

			if ((poll_list[1].revents & (POLLRDHUP | POLLNVAL)) != 0)
				exit(0);

			n = (int)recvfrom(sockfd, (char*)buffer, sizeof(buffer), 0, (struct sockaddr*)&cliaddr, &len);

			buffer[n] = '\0';

#if false // for unit testing
			string ff = (string)SET_IP_FOR_MAC + "MAC:" + mac_address + " IP:" + "192.168.2.19";
			strcpy(buffer, ff.c_str());
#endif

			string reply = construct_reply_discovery(buffer);

			if (reply != "")
			{
				sendto(sockfd, reply.c_str(), reply.length(), MSG_CONFIRM, (const struct sockaddr*)&cliaddr, len);
				log_event("Responded host %s, request %s", address_image(&cliaddr).c_str(), buffer);
			}

			string ip_change = set_ip_for_mac_address(buffer);

			if (ip_change != "")
			{
				write_ip_boot_script(ip_change);
			}
		}
	}

#if false
	// **************************************************************************************
	bool print_usage_r()
	{
		printf(
			"Usage: ot discover inpipe_fd outpipe_fd\n"
			"   inpipe_fd   file descriptor for read end of incoming pipe\n"
			"   outpipe_fd   file descriptor for write end of outgoing pipe\n"
			"Example: ot discover 0 1\n\n"
		);
		return false;
	}
#endif

#if false
	// **************************************************************************************
	bool read_command_line_r(int argc, char* argv[])
	{
		if (argc != 4)
			return print_usage_r();

		// get the bidirectional pipe fds 
		in_pipe_fd = std::stoi(argv[argc - 2]);
		out_pipe_fd = std::stoi(argv[argc - 1]);

		return true;
	}
#endif

} // namespace


bool send_message_to_discovery_port(string request, int sockfd)
{
	bool closeport = false;

	sudo_mode();
	if (sockfd < 0)
	{
		// Creating socket file descriptor 
		if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
		{
			log_severe("Opening socket! Error is: %s", strerror(errno));
			user_mode();
			return false;
		}
		closeport = true;
	}
	user_mode();

	int optval = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
	setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));

	struct sockaddr_in cliaddr;
	memset(&cliaddr, 0, sizeof(cliaddr));

	cliaddr.sin_family = AF_INET; // IPv4 
	cliaddr.sin_addr.s_addr = INADDR_BROADCAST;
	cliaddr.sin_port = htons(30303); // Discovery port (add this as cmd line param?)

	socklen_t len;
	len = sizeof(cliaddr);  //len is value/result 

	sudo_mode();
	int r = (int)sendto(sockfd, request.c_str(), request.length(), MSG_CONFIRM, (const struct sockaddr*)&cliaddr, len);

	if (r < 0)
	{
		log_severe("Sending discovery request! Error is: %s, user is %d, address is %s", strerror(errno), geteuid(),
			address_image(&cliaddr).c_str());
		close(sockfd);
		return false;
	}

	user_mode();

	if (closeport)
		close(sockfd);

	return true;
}

// **************************************************************************************
bool discovery_main()
{
//	if (!read_command_line_y(argc, argv))
//		return false;
	log_event("Discovery daemon starting");
	prctl(PR_SET_NAME, "discoveryZombie");

	dev_name = MYNAME;
	ip_address = "-";
	ip_port = "30303";
	mac_address = "-";
	fw_versions = "1.0.0";

	ip_address = get_my_static_ip();

	mac_address = get_my_mac_address();
	for (auto& c : mac_address) c = (c == ':' ? '-' : c); // change the ':' to '-'

	boot_ok = (config.lookup("reboot-on-change-static-ip") == "on");

	discovery_daemon();
	return 0;
}


// **************************************************************************************
// go out and look for other and report
bool discover_main(int argc, char* argv[])
{
	int sockfd;
	char buffer[1500] = { 0 };
	struct sockaddr_in servaddr;
	memset(&servaddr, 0, sizeof(servaddr));

	// Creating socket file descriptor 
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		log_severe("Opening socket! Error is: %s", strerror(errno));
		return false;
	}

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
		close(sockfd);
		return false;
	}

	if (!send_message_to_discovery_port("Discovery", sockfd))
		return false;

	delay(3000); // wait for everyone to reply

	while (true)
	{
		socklen_t len = sizeof(cliaddr);  //len is value/result 
		int n = (int)recvfrom(sockfd, (char*)buffer, sizeof(buffer), 0, (struct sockaddr*)&cliaddr, &len);

		if (n < 0)
		{
			log_event("poll completed, no more replies");
			close(sockfd);
			exit(0);
		}

		buffer[n] = '\0';

		string s = buffer;
		for (auto& c : s) c = (c == '\r' ? '|' : c); // change the '\r' to '|'
		for (auto& c : s) c = (c == '\n' ? '|' : c); // change the '\n' to '|'

		printf("discovered: %s\n", s.c_str());

//		log_data("discovered: %s", s.c_str());
	}
}
