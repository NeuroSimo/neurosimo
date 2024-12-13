//
// Created by alqio on 23.8.2022.
//

#ifndef MTMS_DEVICE_BRIDGE_SCHEDULING_UTILS_H
#define MTMS_DEVICE_BRIDGE_SCHEDULING_UTILS_H

#include <pthread.h>
#include <sys/types.h>
#include <sched.h>
#include <string>
#include <memory>
#include <thread>
#include <cstring>

#define SCHEDULING_OPTIMIZATION

#define DEFAULT_SCHEDULING_POLICY SCHED_RR
#define DEFAULT_REALTIME_SCHEDULING_PRIORITY 98
#define DEFAULT_NORMAL_SCHEDULING_PRIORITY 20

void set_thread_scheduling(std::thread::native_handle_type thread, int policy, int sched_priority);

#endif //MTMS_DEVICE_BRIDGE_SCHEDULING_UTILS_H
