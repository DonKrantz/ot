#if false

#include <netinet/in.h>

#include "queue.h"
#include "utilities.h"
#include <unistd.h> // write(), read(), close()
#include <sys/socket.h>

Queue::Queue()
{
   // socket
   if ((m_rxfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
   {
      log_severe("Opening queue rx socket! Error is: %s", strerror(errno));
      return;
   }

   if ((m_txfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
   {
      log_severe("Opening queue tx socket! Error is: %s", strerror(errno));
      return;
   }

   struct sockaddr_in servaddr;
   servaddr.sin_family = AF_INET; // IPv4 
   servaddr.sin_addr.s_addr = INADDR_ANY;
   servaddr.sin_port = htons(0);

   int optval = 1;
   setsockopt(m_rxfd, SOL_SOCKET, 0, &optval, sizeof(optval));

   // Bind the socket with the server address 
   if (bind(m_rxfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
   {
      log_severe("Binding socket! Error is: %s", strerror(errno));
      close(m_rxfd);
      return;
   }

   socklen_t len = sizeof(servaddr);
   int k = getsockname(m_rxfd, (struct sockaddr *)&servaddr, &len);
   UNUSED(k);

   m_port = ntohs(servaddr.sin_port);

   memset(&m_cliaddr, 0, sizeof(m_cliaddr));
   m_cliaddr.sin_family = AF_INET; // IPv4 
   m_cliaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
   m_cliaddr.sin_port = htons(m_port);
}

Queue::~Queue()
{
   close(m_rxfd);
}

bool Queue::enqueue(const char* fmt, ...)
{
   va_list args{};
   va_start(args, fmt);

   char temp[1500];

   try
   {
      size_t c = vsnprintf(temp, sizeof(temp), fmt, args);

      ssize_t n = sendto(m_txfd, temp, c, MSG_CONFIRM, (const sockaddr*)&m_cliaddr, sizeof(m_cliaddr));

      if (n != (ssize_t)c)
      {
         log_warning("data loss queuing: %s", strerror(errno));
         return false;
      }
   }
   catch (...)
   {
      log_warning("unexpected error queuing: %s", strerror(errno));
      return false;
   }

   va_end(args);
   return true;
}

string Queue::dequeue()
{
   char temp[1500];

   ssize_t i = recv(m_rxfd, temp, sizeof(temp), 0);

   // skip null used to start connection
   if ((i == 0) && (temp[0] == 0))
      return "";

   if (i < (ssize_t)(sizeof(temp) - 1))
      temp[i] = '\0';
   else
      temp[sizeof(temp) - 1] = '\0';

   return (string)temp;
}

#endif