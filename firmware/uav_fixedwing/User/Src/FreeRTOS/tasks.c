#include "cmsis_os2.h"
#include "usart.h"

#include "Autopilot/slc_controller.h"
#include "Comm/uart_rx.h"
#include "FreeRTOS/tasks.h"
#include "System/system_io.h"
#include "System/init.h"

void autopilot_task(void *argument) {
    slc_controller_init();

    for (;;) {
        if (osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever) == 0x01) {
            osMutexAcquire(sys_io_mutex, osWaitForever);

            slc_controller_update(&atpl_i, &ctrl_cmd_delta);

            osMutexRelease(sys_io_mutex);

            if (huart2.gState == HAL_UART_STATE_READY) {
                HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&ctrl_cmd_delta,
                                      sizeof(ctrl_cmd_t));
            }
        }
    }
}

void rx_task(void *argument) {
    for (;;) {
        if (osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever) == 0x01) {
            osMutexAcquire(sys_io_mutex, osWaitForever);
            
            uart_rx_parse();

            osMutexRelease(sys_io_mutex);

            // trigger autopilot_task
            osThreadFlagsSet(autopilot_taskHandle, 0x01);
        }
    }
}