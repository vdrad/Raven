/**
 * @file raven_comm.c
 * @brief High-level communication manager for the robot.
 * * This module acts as the application-level wrapper for telemetry and data transmission.
 * It formats outgoing messages with a specific TAG and ensures they are safely appended 
 * with carriage return and line feed (\r\n) characters before routing them to the 
 * underlying BLE manager.
 */

#include "raven_comm.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h> 
#include "raven_log.h"
#include "ble_manager.h"

/* --- Macros --- */
#define TAG "COMM"

/* --- Global Variables --- */
static bool initialized = false;

/* --- Functions --- */

/**
 * @brief Initializes the high-level communication module.
 * * Sets up the underlying Bluetooth Low Energy (BLE) manager and readies
 * the system to transmit and receive telemetry data. Safely ignores repeated calls.
 */
void raven_comm_init(void) {
    if (initialized) return;

    ble_manager_init();
    
    initialized = true;
}

/**
 * @brief Formats and sends a tagged telemetry message over the active communication channel (BLE).
 * * This function behaves similarly to `printf` or `ESP_LOGI`. It safely combines the provided 
 * TAG and the formatted string into a single local buffer, appends the standard `\r\n` line 
 * ending, and transmits it. It includes buffer overflow protection to prevent system crashes 
 * if the resulting string is too long.
 * * @param tag    A short string identifying the source or type of the message (e.g., "BATTERY", "MOTOR").
 * @param format The format string (C-style, like in printf).
 * @param ...    Variable arguments matching the format specifiers.
 */
void raven_comm_send_message(const char *tag, const char *format, ...) {
    char buffer[RAVEN_COMM_MAX_MESSAGE_LEN];

    // 1. Write the TAG into the buffer formatted as "[TAG] "
    int prefix_len = snprintf(buffer, sizeof(buffer), "[%s] ", tag);
    
    // Safety check: ensure the tag isn't absurdly large and didn't cause an error
    if (prefix_len < 0 || prefix_len >= sizeof(buffer)) {
        RAVEN_LOGE(TAG, "TAG is too big.");
        return; 
    }
 
    // 2. Format the main message payload right after the TAG
    va_list args;
    va_start(args, format);

    // Write the variable arguments into the remaining space in the buffer
    int msg_len = vsnprintf(buffer + prefix_len, sizeof(buffer) - prefix_len, format, args);
    va_end(args);

    // 3. Append line endings and transmit
    if (msg_len > 0) {
        int total_len = prefix_len + msg_len;
        
        // Check if there is enough space to add the "\r\n" (2 bytes)
        if (total_len + 2 < sizeof(buffer)) {
            buffer[total_len] = '\r';
            buffer[total_len + 1] = '\n';
            total_len += 2;
        } else {
            // If the buffer is completely full, crush the last two characters 
            // to forcefully guarantee the safe line break transmission.
            buffer[sizeof(buffer) - 3] = '\r';
            buffer[sizeof(buffer) - 2] = '\n';
            total_len = sizeof(buffer) - 1;
        }
        
        // Route the fully assembled byte array to the BLE driver for transmission
        ble_manager_send_message((uint8_t *)buffer, (uint16_t)total_len);
    }
}