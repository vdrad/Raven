#include "raven_comm.h"
#include <stdio.h>
#include <stdbool.h>
#include "raven_log.h"
#include "ble_manager.h"

#define TAG "COMM"
static bool initialized = false;

void raven_comm_init(void) {
    if (initialized) return;

    ble_manager_init();
    initialized = true;
}

void raven_comm_send_message(const char *tag, const char *format, ...) {
    char buffer[RAVEN_COMM_MAX_MESSAGE_LEN];

    int prefix_len = snprintf(buffer, sizeof(buffer), "[%s] ", tag);
    if (prefix_len < 0 || prefix_len >= sizeof(buffer)) {
        RAVEN_LOGE(TAG, "TAG is too big.");
        return; 
    }
 
    va_list args;
    va_start(args, format);

    int msg_len = vsnprintf(buffer + prefix_len, sizeof(buffer) - prefix_len, format, args);
    va_end(args);

    if (msg_len > 0) {
        int total_len = prefix_len + msg_len;
        
        if (total_len + 2 < sizeof(buffer)) {
            buffer[total_len] = '\r';
            buffer[total_len + 1] = '\n';
            total_len += 2;
        } else {
            buffer[sizeof(buffer) - 3] = '\r';
            buffer[sizeof(buffer) - 2] = '\n';
            total_len = sizeof(buffer) - 1;
        }
        
        ble_manager_send_message((uint8_t *)buffer, (uint16_t)total_len);
    }
}