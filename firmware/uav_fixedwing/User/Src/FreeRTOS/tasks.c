#include "cmsis_os2.h"
#include "usart.h"

#include "Comm/uart_rx.h"
#include "FreeRTOS/tasks.h"
#include "System/system_io.h"

volatile HAL_StatusTypeDef tx_status = HAL_ERROR;

void autopilot_task(void *argument) {
    for (;;) {
        ctrl_cmd_delta.sync_1 = 0xAA;
        ctrl_cmd_delta.sync_2 = 0xFF;
        ctrl_cmd_delta.delta_a = 0.0f;
        ctrl_cmd_delta.delta_e = 0.0f;
        ctrl_cmd_delta.delta_r = 0.0f;
        ctrl_cmd_delta.delta_t = 0.5f;

        if (huart2.gState == HAL_UART_STATE_READY) {
            HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&ctrl_cmd_delta,
                                            sizeof(ctrl_cmd_t));
        }

        osDelay(10);
    }
}

void rx_task(void *argument) {
    for (;;) {
        // wait for rx_task to be triggered
        if (osThreadFlagsWait(0x01, osFlagsWaitAny,
                              osWaitForever) == 0x01) {
            uart_rx_parse();

            // echo
            // if (huart2.gState == HAL_UART_STATE_READY) {
            //     tx_status = HAL_UART_Transmit_DMA(&huart2, rx_buffer,
            //                                       rx_buffer_size);
            // }
        }
    }
}