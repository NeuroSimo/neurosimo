//
// Created by alqio on 5.9.2022.
//

#ifndef DATA_PROCESSOR_SCHEDULING_UTILS_H
#define DATA_PROCESSOR_SCHEDULING_UTILS_H

#include <pthread.h>
#include <sys/types.h>
#include <sched.h>
#include <string>
#include <memory>
#include <thread>
#include <cstring>
#include <stdexcept>

#ifdef __unix__
#define ON_UNIX
#endif

#define SCHEDULING_OPTIMIZATION

#define DEFAULT_SCHEDULING_POLICY SCHED_RR
#define DEFAULT_REALTIME_SCHEDULING_PRIORITY 98

void set_thread_scheduling(std::thread::native_handle_type thread, int policy, int sched_priority);

#endif //DATA_PROCESSOR_SCHEDULING_UTILS_H
