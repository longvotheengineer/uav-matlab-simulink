#pragma once

#include "cmsis_os2.h"

#include "Comm/uart_rx.h"

extern osMutexId_t sys_io_mutex;

void freertos_init(void);

void uart_init(void);

void user_system_init(void);