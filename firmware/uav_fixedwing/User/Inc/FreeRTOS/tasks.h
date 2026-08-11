#pragma once

#include "cmsis_os2.h"

extern osMutexId_t sys_io_mutex;

void autopilot_task(void *argument);
void rx_task(void *argument);