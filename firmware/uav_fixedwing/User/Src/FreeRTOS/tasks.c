#include "cmsis_os2.h"
#include "usart.h"

#include "Comm/uart_rx.h"
#include "FreeRTOS/tasks.h"
#include "System/system_io.h"
#include "Autopilot/slc_controller.h"

osMutexId_t sys_io_mutex;

void autopilot_task(void *argument) {
    // slc_controller_init();

    for (;;) {
        // osMutexAcquire(sys_io_mutex, osWaitForever);

        // slc_controller_update(&atpl_i, &ctrl_cmd_delta);

        // osMutexRelease(sys_io_mutex);

        // if (huart2.gState == HAL_UART_STATE_READY) {
        //     HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&ctrl_cmd_delta,
        //                           sizeof(ctrl_cmd_t));
        // }

        osDelay(1000);
    }
}

// void rx_task(void *argument) {
//     for (;;) {
//         // wait for rx_task to be triggered
//         if (osThreadFlagsWait(0x01, osFlagsWaitAny,
//                               osWaitForever) == 0x01) {
//             osMutexAcquire(sys_io_mutex, osWaitForever);
            
//             uart_rx_parse();

//             osMutexRelease(sys_io_mutex);

//             // echo
//             // if (huart2.gState == HAL_UART_STATE_READY) {
//             //     HAL_UART_Transmit_DMA(&huart2, rx_buffer,
//             //                                       rx_buffer_size);
//             // }
//         }
//     }
// }

void rx_task(void *argument) {
    slc_controller_init();

    for (;;) {
        // Wait here forever until the UART DMA fires the IDLE line flag
        if (osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever) == 0x01) {
            
            // 1. Unpack the freshest 58 bytes from Simulink
            uart_rx_parse();

            // 2. Instantly calculate the aerodynamics
            slc_controller_update(&atpl_i, &ctrl_cmd_delta);

            // 3. Instantly blast the commands back to Simulink
            if (huart2.gState == HAL_UART_STATE_READY) {
                HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&ctrl_cmd_delta, sizeof(ctrl_cmd_t));
            }
        }
    }
}