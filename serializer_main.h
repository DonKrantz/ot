#pragma once

#if true

bool ports_main();

enum class PORTS { INTERNAL_RX, ROVL_RX, GNSS, TRACKER650, GNSS_REPLICATE, 
	ROVL_REPLICATE, TRACKER650_REPLICATE, MAVLINK_REPLICATE, INTERNAL_RX_SENDING };

enum class PORT_TYPE { SERIAL, UDP, TCP };

void disconnect(enum PORTS device);

void reconnect(enum PORTS device);

bool send_port_message(enum PORTS device, string message);

bool send_port_binary(enum PORTS device, void* message, size_t sizeis);

int receive_from(enum PORTS device, void* message, size_t sizeis);

bool set_baud_rate(enum PORTS device, string baud_rate);

#endif
