#include "System/init.h"

osMutexId_t sys_io_mutex;

void freertos_init(void) {
    sys_io_mutex = osMutexNew(NULL);
}

void uart_init(void) {
    uart_rx_init();
}

void user_system_init(void) {
    freertos_init();
    uart_init();
}