#pragma once

#include "cmsis_os2.h"

extern osThreadId_t autopilot_taskHandle;

void autopilot_task(void *argument);
void rx_task(void *argument);