
#if false

#include "dispatcher.h"
#include <assert.h>
#include <stdlib.h>

namespace {

#define MAX_POLLS 20

   struct pollfd poll_list[MAX_POLLS] = { -1 };

   callback callbacks[MAX_POLLS] = { NULL };

   LINE_BUFFER* linebufs[MAX_POLLS] = { NULL };

} // namespace

// =========================================================================
bool kill_event_loop = false;

// =========================================================================
bool register_fd(int fd, callback cb, LINE_BUFFER* lb)
{
   for( int i = 0; i < MAX_POLLS; i++ )
      if (poll_list[i].fd < 1)
      {
         poll_list[i].fd = fd;
         poll_list[i].events = POLLIN; // only look for data available
         callbacks[i] = cb;
         linebufs[i] = lb;
         return true;
      }
   assert(false); // ran out of slots in the poll list
}

// =========================================================================
bool unregister(int fd)
{
   for (int i = 0; i < MAX_POLLS; i++)
      if (poll_list[i].fd == fd)
      {
         poll_list[i].fd = -1;
         callbacks[i] = NULL;
         return true;
      }
   assert(false); // ran out of slots in the poll list
}

// =========================================================================
void do_events()
{
   kill_event_loop = false;
   while (true)
   {
      int n;
      for (n = MAX_POLLS; n > 0; n--)
      {
         if (poll_list[n - 1].fd > 0)
            break;
      }

      if (n == 0)
         break;

      poll(poll_list, n, 1000);

      for (int i = 0; i < n; i++)
      {
         if (poll_list[i].revents != 0)
            callbacks[i](poll_list[i].fd, poll_list[i].revents, linebufs[i]);
      }

      if (kill_event_loop)
      {
         return;
      }
   }

   assert(false); // no items in poll list
}

#endif
