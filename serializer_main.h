#pragma once

#if true 

bool ports_main();

enum class PORTS { INTERNAL_RX, ROVL_RX, GNSS, TRACKER650, GNSS_REPLICATE, 
	ROVL_REPLICATE, TRACKER650_REPLICATE, MAVLINK_REPLICATE, INTERNAL_RX_SENDING, MAVLINK_SENDING, MAVLINK_LISTENING };

enum class PORT_TYPE { SERIAL, UDP, TCP };

void port_disconnect(enum PORTS device);

void port_reconnect(enum PORTS device);

bool send_port_message(enum PORTS device, string message);

bool send_port_binary(enum PORTS device, void* message, size_t sizeis);

int receive_from(enum PORTS device, void* message, size_t sizeis, int timeout_ms);

string receive_from(enum PORTS device, int timeout_ms);

bool port_set_canonical(enum PORTS device, bool canonical, bool set_even_parity); 

#endif
