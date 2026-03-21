#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "raven_log.h"
#include "raven_comm.h"

void app_main(void)
{
    raven_comm_init();
    for (;;) {
        raven_comm_send_message("RAVEN", "Hello World!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}