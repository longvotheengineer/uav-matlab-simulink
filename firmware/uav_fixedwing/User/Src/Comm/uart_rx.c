#include <string.h>

#include "Comm/uart_rx.h"
#include "System/system_io.h"

// osThreadId_t rx_taskHandle; this is already defined in freertos.c

uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_buffer_size = 0;

void uart_rx_init(void) {
    // start UART/DMA/IDLE
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer,
                                 RX_BUFFER_SIZE);
    // disable half transfer interrupt
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if (huart->Instance == USART2) {
        rx_buffer_size = size;

        // trigger rx_task
        osThreadFlagsSet(rx_taskHandle, 0x01);
        
        // restart UART/DMA/IDLE
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer,
                                     RX_BUFFER_SIZE);
        // redisable half transfer interrupt
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}

void uart_rx_parse(void) {
    if (rx_buffer[0] == 0xAA && 
        rx_buffer[1] == 0xFF &&
        rx_buffer_size == sizeof(atpl_i_t)) {
        memcpy(&atpl_i, rx_buffer, sizeof(atpl_i_t));
    }
}