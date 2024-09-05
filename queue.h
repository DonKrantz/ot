#if false

#pragma once
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string>
#include <netinet/in.h>

using std::string;

class Queue
{
private:
	int		m_rxfd = -1;
	int		m_txfd = -1;
	uint16_t	m_port = 0;
	struct sockaddr_in m_cliaddr;

public:

	Queue();
	~Queue();

	bool enqueue(const char* fmt, ...);

	string dequeue();

	int my_fd()					{ return m_rxfd; }
};
#endif
