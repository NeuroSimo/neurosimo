#ifndef FPGA_H
#define FPGA_H

#include "NiFpga.h"

#ifdef __unix__
#define ON_UNIX
#endif

bool try_init_fpga();
void init_fpga();

bool is_fpga_ok();
bool close_fpga();

extern NiFpga_Session session;
extern NiFpga_Status status;
extern bool fpga_opened;

const uint16_t FPGA_OK_CHECK_INTERVAL_MS = 500;

#endif
