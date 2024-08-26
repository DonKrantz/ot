#pragma once

// listening daemon
bool discovery_main();

// go out and look for other and report
bool discover_main(int argc, char* argv[]);

// Blast a broadcast string to anyone attached to the newtworks. 
// If you want to be able to receive replies procide a socket fd,
// otherwise one will be opened and closed.
bool send_message_to_discovery_port(string request, int sockfd = -1);
