#pragma once

#include <stdint.h>
#include "usart.h"

#define RX_BUFFER_SIZE 128

extern uint8_t rx_buffer[RX_BUFFER_SIZE];

void uart_rx_init(void);
void uart_rx_handle(UART_HandleTypeDef *huart);
void uart_rx_parse(void);