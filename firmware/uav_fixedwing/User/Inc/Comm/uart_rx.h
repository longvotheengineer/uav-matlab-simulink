#pragma once

#include <stdint.h>
#include "cmsis_os2.h"
#include "usart.h"

#define RX_BUFFER_SIZE 128

extern osThreadId_t rx_taskHandle;

extern uint8_t rx_buffer[RX_BUFFER_SIZE];
extern volatile uint16_t rx_buffer_size;

void uart_rx_init(void);
void uart_rx_parse(void);