#pragma once

#if false
#include "utilities.h"

#include <poll.h>

typedef void (*callback)(int, int, LINE_BUFFER*);

bool register_fd(int fd, callback cb, LINE_BUFFER* lb);

bool unregister(int fd);

void do_events();

extern bool kill_event_loop;
#endif
