//
// Created by alqio on 22.8.2022.
// Adapted from https://github.com/ros-realtime/ros2-realtime-examples/tree/rolling/minimal_memory_lock
// Apache 2.0 License. No major changes
//


#ifndef MTMS_DEVICE_BRIDGE_MEMORY_UTILS_H
#define MTMS_DEVICE_BRIDGE_MEMORY_UTILS_H

#include <sys/types.h>
#include <sys/mman.h>
#include <malloc.h>
#include <string>
#include <cstring>
#include <stdexcept>
#include <unistd.h>
#include <iostream>

#define MEMORY_OPTIMIZATION


void preallocate_memory(size_t memory_size);
void lock_memory();
void set_default_thread_stacksize(size_t stacksize);

#endif //MTMS_DEVICE_BRIDGE_MEMORY_UTILS_H
